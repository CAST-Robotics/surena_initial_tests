#include "DCM.h"

DCMPlanner::DCMPlanner(double deltaZ, double stepTime, double doubleSupportTime, double dt, int stepCount, double alpha, double theta){
    this->deltaZ_ = deltaZ;
    this->tStep_ = stepTime;
    this->tDS_ = doubleSupportTime;
    this->theta_ = theta;
    if(alpha > 0.0 && alpha < 1.0)
        this->alpha_ = alpha;
    else
        throw "Invalid Value for alpha";
    this->dt_ = dt;
    this->stepCount_ = stepCount;
    this->yawSign_ = 1;
    //cout << "DCMPlanner Object created" << endl;
}

void DCMPlanner::setFoot(Vector3d rF[], int sign){
    rF_ = new Vector3d[stepCount_];
    rF_ = rF;
    this->yawSign_ = sign;
}

Vector3d* DCMPlanner::getXiDot(){
    return xiDot_;
}

Vector3d* DCMPlanner::get_CoMDot(){
    return CoMDot_;
}

Vector3d* DCMPlanner::getXiTrajectory(){
    /*
        This function returns an array of positions with intervals of dt_
        It is best to call it at end of each step with new foot positions
    */
    this->updateVRP();
    this->updateXiEoS();
    this->updateSS();
    this->updateXiDSPositions();
    return xi_;
}

Vector3d* DCMPlanner::getCoM(){
    int length = int((stepCount_ * tStep_) / dt_);  // +1 second is for decreasing robot's height from COM_0 to deltaZ
    COM_ = new Vector3d[length];
    CoMDot_ = new Vector3d[length];
    // decreasing robot's height
    Vector3d COM_init(0.0, 0.0, deltaZ_);  // initial COM when robot start to walk
    /*
     Vector3d* coefs = this->minJerkInterpolate(COM_0, COM_init, Vector3d::Zero(3), Vector3d::Zero(3), 1);
    for (int i = 0; i < 1/dt_; i++){
        double time = dt_ * i;
        COM_[i] = coefs[0] + coefs[1] * time + coefs[2] * pow(time,2) + coefs[3] * pow(time,3);
        CoMDot_[i] = coefs[1] + 2 * coefs[2] * time + 3 * coefs[3] * pow(time, 2);
    }
    */
    // COM trajectory based on DCM
    Vector3d inte;
    for (int i = 0; i < length; i++){
        inte << 0.0,0.0,0.0;
        for(int j = 0; j < i; j++)
            inte += sqrt(K_G/deltaZ_) * ((xi_[j] * exp(j * dt_ * sqrt(K_G/deltaZ_))) + (xi_[j + 1] * exp((j + 1) * dt_ * sqrt(K_G/deltaZ_)))) * 0.5 * dt_;
        COM_[i] = (inte + COM_init) * exp(-i*dt_*sqrt(K_G/deltaZ_)); // COM_0 or COM_init ??
        COM_[i](2) = xi_[i](2);
        CoMDot_[i] = - sqrt(K_G/deltaZ_) * (COM_[i] - xi_[i]);
    }
    MinJerk::write2File(COM_, length, "COM");
    MinJerk::write2File(CoMDot_, length, "COMDot");
    return COM_;
}

void DCMPlanner::updateVRP(){
    // Updates Virtual Repelant Points !! should be called after setFoot() !!
    rVRP_ = new Vector3d[this->stepCount_];
    Vector3d deltaZ(0.0,0.0,deltaZ_);
    for (int i = 0 ; i < this->stepCount_; i ++){
        rVRP_[i] = rF_[i] + deltaZ;
    }
}

void DCMPlanner::updateSS(){
    // Generates DCM trajectory without Double Support Phase
    int length = 1/dt_ * tStep_ * stepCount_;
    xi_ = new Vector3d[length];
    xiDot_ = new Vector3d[length];
    int stepNum;
    double time;
    for (int i = 0; i < length; i ++){
        time = dt_ * i;
        stepNum = floor(time / tStep_);
        xi_[i] = rVRP_[stepNum] + exp(sqrt(K_G/deltaZ_) * (fmod(time,tStep_) - tStep_)) * (xiEOS_[stepNum] - rVRP_[stepNum]);
        xiDot_[i] = sqrt(K_G/deltaZ_) * (xi_[i] - rVRP_[stepNum]);
    }
}

void DCMPlanner::updateXiDSPositions(){
    /*
        This Function rounds single support trajectory bends
        for double support phase. 
        ! Double support starts and ends should be updated !
    */
    this->updateDS();
    int length = 1/dt_ * tStep_ * stepCount_;
    Vector3d xi_dot_i, xi_dot_e;
    for(int step = 0 ; step < stepCount_; step ++){
        if (step == 0){
            xi_dot_i = sqrt(K_G / deltaZ_) * (xiDSI_[step] - xi_[0]);
            xi_dot_e = sqrt(K_G / deltaZ_) * (xiDSE_[step] - rVRP_[0]);
            Vector3d* coefs = this->minJerkInterpolate(xiDSI_[step],xiDSE_[step],xi_dot_i, xi_dot_e, tDS_);
            for (int i = 0; i < (1/dt_) * tDS_ * (1 - alpha_); ++i){
                double time = dt_ * i;
                xi_[i] = coefs[0] + coefs[1] * time + coefs[2] * pow(time,2) + coefs[3] * pow(time,3);
                xiDot_[i] = coefs[1] + 2 * coefs[2] * time + 3 * coefs[3] * pow(time,2);
            }
            delete[] coefs;     
        }
        else{
            xi_dot_i = sqrt(K_G/deltaZ_) * (xiDSI_[step] - rVRP_[step - 1]);
            xi_dot_e = sqrt(K_G/deltaZ_) * (xiDSE_[step] - rVRP_[step]);
            Vector3d* coefs = this->minJerkInterpolate(xiDSI_[step],xiDSE_[step],xi_dot_i, xi_dot_e, tDS_);
            for (int i = (tStep_ * step)/dt_ - (tDS_ * alpha_ / dt_ ) + 1; i < ((tStep_ * step)/dt_) + (tDS_/dt_) * (1-alpha_); ++i){
                double time = fmod(i * dt_, tStep_ * step - tDS_ * alpha_);
                xi_[i] = coefs[0] + coefs[1] * time + coefs[2] * pow(time,2) + coefs[3] * pow(time,3);
                xiDot_[i] = coefs[1] + 2 * coefs[2] * time + 3 * coefs[3] * pow(time,2);
            }
            delete[] coefs;
        }   
    }
    MinJerk::write2File(xi_, length, "xi");
    MinJerk::write2File(xiDot_, length, "xiDot");
}

void DCMPlanner::updateDS(){
    /*
        This function updates Double support start and end positions
    */
    xiDSI_ = new Vector3d[stepCount_];
    xiDSE_ = new Vector3d[stepCount_];

    for (int index = 0 ; index < stepCount_; index ++){
        if(index == 0){
            xiDSI_[index] = xi_[0];
            xiDSE_[index] = rVRP_[index] + exp(sqrt(K_G/deltaZ_) * tDS_ * (1 - alpha_)) * (xi_[0] - rVRP_[index]);
        }
        else{
            xiDSI_[index] = rVRP_[index-1] + exp(-sqrt(K_G/deltaZ_) * tDS_ * alpha_) * (xiEOS_[index-1] - rVRP_[index-1]);
            xiDSE_[index] = rVRP_[index] + exp(sqrt(K_G/deltaZ_) * tDS_ * (1 - alpha_)) * (xiEOS_[index-1] - rVRP_[index]);
        }
    }
}

void DCMPlanner::updateXiEoS(){

    xiEOS_ = new Vector3d[stepCount_];
    for (int i = stepCount_- 1 ; i >= 0; i--){
        if (i == stepCount_ - 1)
            xiEOS_[i] = rVRP_[i];
        else
            xiEOS_[i] = rVRP_[i + 1] + exp(-sqrt(K_G/deltaZ_) * tStep_) * (xiEOS_[i+1] - rVRP_[i+1]);
    }
}

Vector3d* DCMPlanner::minJerkInterpolate(Vector3d theta_ini, Vector3d theta_f, Vector3d theta_dot_ini, Vector3d theta_dot_f, double tf){
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

Vector3d* DCMPlanner::getZMP(){
    /*
        This function returns desired ZMP. getXiTrajectory should be called first.
    */
    int length = 1/dt_ * tStep_ * stepCount_;
    ZMP_ = new Vector3d[length];
    for (int i = 0; i < length; i ++)
        ZMP_[i] = xi_[i] - xiDot_[i] * sqrt(deltaZ_/K_G);
    MinJerk::write2File(ZMP_, length, "ZMP");
    return ZMP_;
}

Matrix3d* DCMPlanner::yawRotGen(){
    yawRotation_ = new Matrix3d[int ((stepCount_*tStep_)/dt_)];
    double ini_theta;
    double end_theta;

    for (int j=0; j<tStep_/dt_; j++)
        yawRotation_[j] = AngleAxisd(0, Vector3d::UnitZ());

    for (int i=1; i<stepCount_ - 1; i++){
        ini_theta = this->yawSign_ * ((i-1)*theta_ + (i-2)*theta_)/2;//
        end_theta = this->yawSign_ * ((i-1)*theta_ + i*theta_)/2;//

        if (i==1)
            ini_theta = 0;
        if (i==stepCount_ - 2)
            end_theta = this->yawSign_ * (stepCount_ - 3) * theta_;//
        double* coef = MinJerk::cubicInterpolate<double>(ini_theta, end_theta, 0, 0, tStep_);
        for (int j=0; j<tStep_/dt_; j++){
            double theta_traj = coef[0] + coef[1] * j*dt_ + coef[2] * pow(j*dt_,2) + coef[3] * pow(j*dt_,3);
            yawRotation_[int((i)*tStep_/dt_ + j)] = AngleAxisd(theta_traj, Vector3d::UnitZ());
        }
    }
    
    for (int j=0; j<tStep_/dt_; j++){
        
        yawRotation_[int((stepCount_-1) * tStep_/dt_ +  j)] = yawRotation_[int((stepCount_-1) * tStep_/dt_ +  j)-1];
    }
    return yawRotation_;
}

DCMPlanner::~DCMPlanner(){
    delete[] rF_;
    delete[] rVRP_;
    delete[] xiEOS_;
    delete[] xi_;
    delete[] xiDot_;
    delete[] xiDSI_;
    delete[] xiDSE_;
    delete[] COM_;
    delete[] ZMP_;
    delete[] yawRotation_;
}