#include "AnkleTraj.h"


AnkleTraj::AnkleTraj(){
}

void AnkleTraj::planStance(Vector3d r_foot, Vector3d l_foot, double yaw, double time){
    int n = time / dt_;
    // we assume that left and right yaw angles are identical in the stance phase
    //TODO: different yaw in the stance
    Matrix3d rot = Matrix3d(AngleAxisd(yaw, Vector3d::UnitZ()));
    for(int i=0; i<n; i++){
        plannedLAnklePos_.push_back(l_foot);
        plannedRAnklePos_.push_back(r_foot);
        plannedLAnkleRot_.push_back(rot);
        plannedRAnkleRot_.push_back(rot);
    }
}

void AnkleTraj::planInitialDSP(){
    if(leftFirst_)
        this->planStance(footSteps_[1], footSteps_[0], footYaws_[0], initDSPDuration_);
    else
        this->planStance(footSteps_[0], footSteps_[1], footYaws_[0], initDSPDuration_);
}

void AnkleTraj::planFinalDSP(){
    if(leftLast_)
        this->planStance(footSteps_[footSteps_.size()-2], footSteps_[footSteps_.size()-1], footYaws_[footYaws_.size()-1], finalDSPDuration_);
    else
        this->planStance(footSteps_[footSteps_.size()-1], footSteps_[footSteps_.size()-2], footYaws_[footYaws_.size()-1], finalDSPDuration_);
}

void AnkleTraj::planSteps(){
    vector<Vector3d> first_ankle_pos;
    vector<Matrix3d> first_ankle_rot;
    vector<Vector3d> second_ankle_pos;
    vector<Matrix3d> second_ankle_rot;
    
    for(int i=0; i<footSteps_.size()-2; i++){
        // MatrixXd xy_way_points(2, 5);
        MatrixXd z_way_points(3, 3);
        // xy_way_points.row(0) << footSteps_[i](0), footSteps_[i](0), (footSteps_[i+2](0) + footSteps_[i](0)) / 2, footSteps_[i+2](0), footSteps_[i+2](0);
        // xy_way_points.row(1) << footSteps_[i](1), footSteps_[i](1), (footSteps_[i+2](1) + footSteps_[i](1)) / 2, footSteps_[i+2](1), footSteps_[i+2](1);
        z_way_points.row(0) << footSteps_[i](0), (footSteps_[i+2](0) + footSteps_[i](0)) / 2, footSteps_[i+2](0);
        z_way_points.row(1) << footSteps_[i](1), (footSteps_[i+2](1) + footSteps_[i](1)) / 2, footSteps_[i+2](1);
        z_way_points.row(2) << footSteps_[i](2), footSteps_[i](2) + stepHeight_, footSteps_[i+2](2);
        
        // MatrixXd xy_vel_points = MatrixXd::Zero(2, 5);
        MatrixXd z_vel_points = MatrixXd::Zero(3, 3);
        z_vel_points(0, 1) = (footSteps_[i+2](0) - footSteps_[i](0)) / SSPDuration_;

        // VectorXd xy_time_points(5);
        VectorXd z_time_points(3);
        // xy_time_points << 0.0, 0.2 * SSPDuration_, 0.5 * SSPDuration_, 0.8 * SSPDuration_, SSPDuration_;
        z_time_points << 0.0, 0.5 * SSPDuration_, SSPDuration_;
        // MatrixXd xy_traj;
        MatrixXd z_traj;
        // this->cubicPolyTraj(xy_way_points, xy_time_points, dt_, xy_vel_points, xy_traj);
        this->cubicPolyTraj(z_way_points, z_time_points, dt_, z_vel_points, z_traj);

        MatrixXd yaw_way_points(1,2);
        yaw_way_points << footYaws_[i], footYaws_[i+2];
        MatrixXd yaw_vel_points = MatrixXd::Zero(1, 2);
        VectorXd yaw_time_points(2);
        yaw_time_points << 0.0, SSPDuration_;
        MatrixXd yaw_traj;
        this->cubicPolyTraj(yaw_way_points, yaw_time_points, dt_, yaw_vel_points, yaw_traj);
        // first ankle swing
        if(i%2==0){
            // single support phase
            for(int j=0; j<int(SSPDuration_ / dt_); j++){
                first_ankle_pos.push_back(Vector3d(z_traj(0, j), z_traj(1, j), z_traj(2, j)));
                first_ankle_rot.push_back(Matrix3d(AngleAxisd(yaw_traj(0, j), Vector3d::UnitZ())));
                second_ankle_pos.push_back(footSteps_[i+1]);
                second_ankle_rot.push_back(Matrix3d(AngleAxisd(footYaws_[i+1], Vector3d::UnitZ())));
            }
            // double support phase
            for(int j=0; j<int(DSPDuration_ / dt_); j++){
                first_ankle_pos.push_back(footSteps_[i+2]);
                first_ankle_rot.push_back(Matrix3d(AngleAxisd(footYaws_[i+2], Vector3d::UnitZ())));
                second_ankle_pos.push_back(footSteps_[i+1]);
                second_ankle_rot.push_back(Matrix3d(AngleAxisd(footYaws_[i+1], Vector3d::UnitZ())));
            }
        }
        // second ankle swing
        else{
            // single support phase
            for(int j=0; j<int(SSPDuration_ / dt_); j++){
                second_ankle_pos.push_back(Vector3d(z_traj(0, j), z_traj(1, j), z_traj(2, j)));
                second_ankle_rot.push_back(Matrix3d(AngleAxisd(yaw_traj(0, j), Vector3d::UnitZ())));
                first_ankle_pos.push_back(footSteps_[i+1]);
                first_ankle_rot.push_back(Matrix3d(AngleAxisd(footYaws_[i+1], Vector3d::UnitZ())));
            }
            // double support phase
            for(int j=0; j<int(DSPDuration_ / dt_); j++){
                second_ankle_pos.push_back(footSteps_[i+2]);
                second_ankle_rot.push_back(Matrix3d(AngleAxisd(footYaws_[i+2], Vector3d::UnitZ())));
                first_ankle_pos.push_back(footSteps_[i+1]);
                first_ankle_rot.push_back(Matrix3d(AngleAxisd(footYaws_[i+1], Vector3d::UnitZ())));
            }
        }
    }

    if(leftFirst_){
        for(int i=0; i< first_ankle_pos.size(); i++){
            plannedLAnklePos_.push_back(first_ankle_pos[i]);
            plannedLAnkleRot_.push_back(first_ankle_rot[i]);
            plannedRAnklePos_.push_back(second_ankle_pos[i]);
            plannedRAnkleRot_.push_back(second_ankle_rot[i]);
        }
    }else{
        for(int i=0; i< first_ankle_pos.size(); i++){
            plannedLAnklePos_.push_back(second_ankle_pos[i]);
            plannedLAnkleRot_.push_back(second_ankle_rot[i]);
            plannedRAnklePos_.push_back(first_ankle_pos[i]);
            plannedRAnkleRot_.push_back(first_ankle_rot[i]);
        }
    }
}