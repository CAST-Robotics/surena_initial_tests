#include "PreviewTraj.h"

PreviewTraj::PreviewTraj(ZMPPlanner* zmp_planner, double robot_height, int n, double dt) : ZMPPlanner_(zmp_planner), robotHeight_(robot_height), N_(n), dt_(dt) {
    
    g_ = 9.81;

    A_ << 1, dt_, pow(dt_, 2) / 2,
         0, 1, dt_,
         0, 0, 1;

    b_ << pow(dt_, 3) / 6,
         pow(dt_, 2) / 2,
         dt_;

    c_ << 1, 0, -(robotHeight_ / g_);
    
    Q_ = 1 * MatrixXd::Identity(1, 1);
    R_ = pow(10, -6) * MatrixXd::Identity(1, 1);

    x0_ << 0, 0, 0;
    P_ = MatrixXd::Zero(4, 4);
    Gd_ = VectorXd::Zero(N_);

    error_ = Vector3d(0, 0, 0);
    int traj_size = ZMPPlanner_->getTrajSize();
    x_ = new Vector3d[traj_size];
    y_ = new Vector3d[traj_size];
    x_[0] = x0_;
    y_[0] = x0_;
    CoMPos_.push_back(Vector3d(0, 0, robotHeight_));
}

void PreviewTraj::setInitCondition(const Vector3d& x0, const Vector3d& y0)
{
    x_[0] = x0;
    y_[0] = y0;

    if(CoMPos_.size() != 0)
        CoMPos_[0] = Vector3d(x0(0), y0(0), robotHeight_);
    else
        CoMPos_.push_back(Vector3d(x0(0), y0(0), robotHeight_));
}

PreviewTraj::~PreviewTraj(){
}

void PreviewTraj::setDt(double dt){
    dt_ = dt;
}

void PreviewTraj::computeWeight(){

    MatrixXd A_bar = MatrixXd::Zero(4, 4);
    A_bar(0, 0) = 1.0;
    A_bar.block(0, 1, 1, 3) = c_ * A_;
    A_bar.block(1, 1, 3, 3) = A_;

    MatrixXd B_bar = MatrixXd::Zero(4, 1);
    B_bar.block(0, 0, 1, 1) = c_ * b_;
    B_bar.block(1, 0, 3, 1) = b_;

    MatrixXd Q_bar = MatrixXd::Zero(4, 4);
    Q_bar.block(0, 0, 1, 1) = Q_;

    solveRiccatiIterationD(A_bar, B_bar, Q_bar, R_, P_);

    MatrixXd I_bar = MatrixXd::Zero(4, 1);
    I_bar(0, 0) = 1.0;

    MatrixXd F_bar = MatrixXd::Zero(4, 3);
    F_bar.block(0, 0, 1, 3) = c_ * A_;
    F_bar.block(1, 0, 3, 3) = A_;

    Gl_ = (R_ + B_bar.transpose() * P_ * B_bar).inverse() * B_bar.transpose() * P_ * I_bar;
    Gx_ = (R_ + B_bar.transpose() * P_ * B_bar).inverse() * B_bar.transpose() * P_ * F_bar;

    MatrixXd Ac_bar = A_bar - B_bar * (R_ + B_bar.transpose() * P_ * B_bar).inverse() * B_bar.transpose() * P_ * A_bar;
    MatrixXd X_bar = -Ac_bar.transpose() * P_ * I_bar;
    
    Gd_(0) = -Gl_(0);
    for(int i=1; i < N_; i++) {
        Gd_(i) = ((R_ + B_bar.transpose() * P_ * B_bar).inverse() * B_bar.transpose() * X_bar)(0);
        X_bar = Ac_bar.transpose() * X_bar;
    }
}

void PreviewTraj::computeTraj(){

    for(int i=1; i<ZMPPlanner_->getTrajSize(); i++){

        error_(0) += c_ * x_[i - 1] - ZMPPlanner_->getZMP(i - 1)(0);
        error_(1) += c_ * y_[i - 1] - ZMPPlanner_->getZMP(i - 1)(1);

        Vector3d preview_u(0, 0, 0);
        for(int j=0; j<N_; j++){
            preview_u += double(Gd_(j)) * (ZMPPlanner_->getZMP(i + j));
        }
        
        double ux = ((-Gl_ * error_(0) - Gx_ * x_[i - 1])(0) - preview_u(0));
        double uy = ((-Gl_ * error_(1) - Gx_ * y_[i - 1])(0) - preview_u(1));
        x_[i] = A_ * x_[i - 1] + ux * b_;
        y_[i] = A_ * y_[i - 1] + uy * b_;

        // double p = c_ * x_[i - 1];
        // cout << x_[i](0) << ", " << x_[i](1) << ", " << x_[i](2) << ", " << p << ", ";
        // p = c_ * y_[i - 1];
        // cout << y_[i](0) << ", " << y_[i](1) << ", " << y_[i](2) << ", " << p << endl;

        CoMPos_.push_back(Vector3d(x_[i](0), y_[i](0), robotHeight_));
    }
}

void PreviewTraj::planYawTraj(){
    vector<double> yaws = ZMPPlanner_->getFootYaws();
    int size = yaws.size();
    double step_time = ZMPPlanner_->getSSPDuration() + ZMPPlanner_->getDSPDuration();
    double init_yaw, final_yaw;

    for(int i=0; i<int(ZMPPlanner_->getInitDSPDuration() / dt_); i++){
        init_yaw = (yaws[0] + yaws[1]) / 2;
        CoMRot_.push_back(Matrix3d(AngleAxisd(init_yaw, Vector3d::UnitZ())));
    }

    for(int i=1; i<size-1; i++){
        final_yaw = (yaws[i] + yaws[i+1]) / 2;
        MatrixXd yaw_way_points(1,2);
        yaw_way_points << init_yaw, final_yaw;
        MatrixXd yaw_vel_points = MatrixXd::Zero(1, 2);
        VectorXd yaw_time_points(2);
        yaw_time_points << 0.0, step_time;
        MatrixXd yaw_traj;
        ZMPPlanner_->cubicPolyTraj(yaw_way_points, yaw_time_points, dt_, yaw_vel_points, yaw_traj);
        
        for(int j=0; j<int(step_time / dt_); j++){
            CoMRot_.push_back(Matrix3d(AngleAxisd(yaw_traj(0, j), Vector3d::UnitZ())));
        }
        init_yaw = final_yaw;
    }

    for(int i=0; i<int(ZMPPlanner_->getFinalDSPDuration() / dt_); i++){
        final_yaw = (yaws[size-1] + yaws[size-2]) / 2;
        CoMRot_.push_back(Matrix3d(AngleAxisd(final_yaw, Vector3d::UnitZ())));
    }
}