#include "MinJerk.h"

double MinJerk::initDSPDuration_;
double MinJerk::DSPDuration_;
double MinJerk::SSPDuration_;
double MinJerk::finalDSPDuration_;
double MinJerk::stepHeight_;
vector<Vector3d> MinJerk::footSteps_;
vector<double> MinJerk::footYaws_;
double MinJerk::dt_;
bool MinJerk::leftFirst_;
bool MinJerk::leftLast_;
int MinJerk::footStepCount_;
int MinJerk::trajSize_;

MinJerk::MinJerk(){}

MinJerk::~MinJerk(){}

Vector3d* MinJerk::poly6Interpolate(Vector3d x_ini, Vector3d x_mid, Vector3d x_f, double tf){
    /* 
        This function fits a 5th order (C2) Polynomial to given given inputs
        Inputs : X0, Xmid, Xf   !Xd, Xdd at begining and end are assumed to be zero!
    */
    
    Vector3d* a = new Vector3d[7];
    a[0] = x_ini;
    a[1] = Vector3d::Zero(3);
    a[2] = Vector3d::Zero(3);
    a[3] = -2/pow(tf,3) * (21 * x_ini + 11 * x_f - 32 * x_mid);
    a[4] = 3/pow(tf,4) * (37 * x_ini + 27 * x_f - 64 * x_mid);
    a[5] = -6/pow(tf,5) * (17 * x_ini + 15 * x_f - 32 * x_mid);
    a[6] = 3/pow(tf,4) * (x_ini + x_f - 2 * x_mid);
    return a;
}

Vector3d* MinJerk::ankle5Poly(Vector3d x0, Vector3d xf, double z_max, double tf, double final_height){
    Vector3d* ans = new Vector3d[6];
    // XY trajectory 5th order with Vel. and accl. B.C.
    ans[0] = x0;
    ans[1] = Vector3d::Zero(3);
    ans[2] = Vector3d::Zero(3);
    ans[3] = 10/pow(tf,3) * (xf - x0);
    ans[4] = -15/pow(tf,4) * (xf - x0);
    ans[5] = 6/pow(tf,5) * (xf - x0);
    // Z trajectory also 5th order with velocity B.C.
    ans[0](2) = x0(2);
    ans[1](2) = 0.0;
    ans[2](2) = 7 * final_height / pow(tf,2) + 16 * z_max / pow(tf,2);
    ans[3](2) = -34* final_height / pow(tf,3) - 32* z_max / pow(tf,3);
    ans[4](2) = 52 * final_height /pow(tf,4) + 16 * z_max /pow(tf,4);
    ans[5](2) = -24 * final_height / pow(tf,5);

    return ans;
}

void MinJerk::write2File(Vector3d* input, int size, string file_name="data"){
    ofstream output_file(file_name + ".csv");
    for(int i=0; i<size; i++){
         output_file << input[i](0) << " ,";
         output_file << input[i](1) << " ,";
         output_file << input[i](2) << " ,";
         output_file << "\n";
    }
    output_file.close();
}

Vector3d* MinJerk::minJerkInterpolate(Vector3d theta_ini, Vector3d theta_f, Vector3d theta_dot_ini, Vector3d theta_dot_f, double tf){
    /* 
        Returns Cubic Polynomial with the Given Boundary Conditions
        https://www.tu-chemnitz.de/informatik//KI/edu/robotik/ws2016/lecture-tg%201.pdf
    */
    Vector3d* coefs = new Vector3d[4]; // a0, a1, a2, a3
    coefs[0] = theta_ini;
    coefs[1] = theta_dot_ini;
    coefs[2] = 3/pow(tf,2) * (theta_f - theta_ini) - 1/tf * (2 * theta_dot_ini + theta_dot_f);
    coefs[3] = -2/pow(tf,3) * (theta_f - theta_ini) + 1/pow(tf,2) * (theta_dot_ini + theta_dot_f);
    return coefs;
}

void MinJerk::setConfigPath(string config_path){
    config_ = YAML::LoadFile(config_path);
    this->parseConfig(config_);
}

void MinJerk::parseConfig(YAML::Node config){
    initDSPDuration_ = config["init_dsp_duration"].as<double>();
    DSPDuration_ = config["dsp_duration"].as<double>();
    SSPDuration_ = config["ssp_duration"].as<double>();
    finalDSPDuration_ = config["final_dsp_duration"].as<double>();
    footStepCount_ = config["footsteps"].size();
    stepHeight_ = config["step_height"].as<double>();
    
    for(int i=0; i<footStepCount_; i++){
        Vector3d temp(config["footsteps"][i][0].as<double>(), config["footsteps"][i][1].as<double>(), config["footsteps"][i][2].as<double>());
        footSteps_.push_back(temp);
        footYaws_.push_back(config["footsteps"][i][3].as<double>());
    }

    trajSize_ = int((initDSPDuration_ + (footStepCount_ - 2) * (DSPDuration_ + SSPDuration_) + finalDSPDuration_) / dt_);
    
    if(footSteps_[0](1) > 0)
        leftFirst_ = true;
    else
        leftFirst_ = false;

    isLeftLast();
}

void MinJerk::setParams(double init_dsp, double dsp, double ssp, double final_dsp, double step_height){
    initDSPDuration_ = init_dsp;
    DSPDuration_ = dsp;
    SSPDuration_ = ssp;
    finalDSPDuration_ = final_dsp;
    stepHeight_ = step_height;

    trajSize_ = int((initDSPDuration_ + (footStepCount_ - 2) * (DSPDuration_ + SSPDuration_) + finalDSPDuration_) / dt_);
}

void MinJerk::setFootStepsData(vector<Vector3d> foot_steps, vector<double> foot_yaws){
    footSteps_ = foot_steps;
    footYaws_ = foot_yaws;
    footStepCount_ = footSteps_.size();

    if(footSteps_[0](1) > 0)
        leftFirst_ = true;
    else
        leftFirst_ = false;

    isLeftLast();
}

void MinJerk::isLeftLast(){
    int last_iter = footSteps_.size() - 1;
    double angle_rem = fmod(footYaws_[last_iter], 2 * M_PI);
    if(angle_rem < M_PI/2 || angle_rem > 3*M_PI/2){
        if(footSteps_[last_iter](1) - footSteps_[last_iter-1](1) > 0)
            leftLast_ = true;
        else
            leftLast_ = false;
    }else{
        if(footSteps_[last_iter](1) - footSteps_[last_iter-1](1) > 0)
            leftLast_ = false;
        else
            leftLast_ = true;
    }
}

int MinJerk::getTrajSize(){
    return trajSize_;
}

void MinJerk::cubicPolyTraj(const MatrixXd& way_points, const VectorXd& time_points, double dt, const MatrixXd& vel_points, MatrixXd& q){
    
    int n = way_points.rows();
    int p = way_points.cols();

    double time_span = time_points(p-1) - time_points(0);
    int len = time_span / dt;
    q = MatrixXd::Zero(n, len);
    
    int coef_dim = 4;
    MatrixXd coef_mat = MatrixXd::Zero((p-1)*n, coef_dim);
    
    for(int i=0; i<p-1; i++){
        double final_time = time_points(i+1) - time_points(i);
        for(int j=0; j<n; j++){
            int ridx = i * n + j;
            double points[2] = {way_points(j, i), way_points(j, i+1)};
            double vels[2] = {vel_points(j, i), vel_points(j, i+1)};
            coef_mat.block(ridx, 0, 1, 4) = genCubicCoeffs(points, vels, final_time).transpose();
            }
    }
    for(int i=0; i < len; i++){
        double t = i * dt;
        for(int j=0; j<p-1; j++){
            if((t < time_points(j+1) && t >= time_points(j)) || t == time_points(p-1)){
                double time = t - time_points[j];
                Vector4d time_vec{1, time, pow(time, 2), pow(time, 3)};
                q.col(i) = coef_mat.block(j*n, 0, n, 4) * (time_vec);
            }
        }
    }
}

Vector4d MinJerk::genCubicCoeffs(const double pos_pts[], const double vel_pts[], double final_time){
    
    Vector4d coeff_vec{pos_pts[0], vel_pts[0], 0, 0};
    MatrixXd t_mat(2, 2);
    t_mat << 1, final_time, 0, 1;

    Vector2d B = Vector2d(pos_pts[1], vel_pts[1]) - t_mat * coeff_vec.segment(0, 2);
    MatrixXd inv_t_mat(2, 2);
    inv_t_mat << 3.0 / pow(final_time, 2), -1.0 / final_time, -2.0 / pow(final_time, 3), 1.0 / pow(final_time, 2);

    coeff_vec.segment(2,2) = inv_t_mat * B;
    return coeff_vec;
}