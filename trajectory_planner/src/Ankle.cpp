#include "Ankle.h"

Ankle::Ankle(double step_time, double ds_time, double height, double alpha, short int num_step, double dt, double theta){
    this->tStep_ = step_time;
    this->tDS_ = ds_time;
    this->alpha_ = alpha;
    this->stepCount_ = num_step;
    this->height_ = height;
    this->dt_ = dt;
    this->theta_ = theta;
    this->yawSign_ = 1;

    //cout << "ankle Trajectory Planner initialized\n";
}

void Ankle::updateFoot(Vector3d foot_pose[], int sign){
    /*
        begining and end steps (Not included in DCM foot planner)
        should be given too. 
    */
    this->footPose_ = new Vector3d[stepCount_ + 2];
    footPose_ = foot_pose;
    if(foot_pose[0](1) > 0)
        leftFirst_= true;   // First Swing foot is left foot
    else
        leftFirst_ = false;  // First Swing foot is right foot
    this->yawSign_ = sign;
}

Vector3d* Ankle::getTrajectoryL(){
    return lFoot_;
}

Vector3d* Ankle::getTrajectoryR(){
    return rFoot_;
}

Matrix3d* Ankle::getRotTrajectoryR(){
    return rFootRot_;
}

Matrix3d* Ankle::getRotTrajectoryL(){
    return lFootRot_;
}

void Ankle::generateTrajectory(){

    int length = int(((stepCount_ + 2) * tStep_) / dt_ + 100);
    lFoot_ = new Vector3d[length];
    rFoot_ = new Vector3d[length];
    rFootRot_ = new Matrix3d[length];
    lFootRot_ = new Matrix3d[length];

    if(leftFirst_)
        updateTrajectory(true);
    else
        updateTrajectory(false);
}


void Ankle::updateTrajectory(bool left_first){
    int index = 0;
    int length = int(((stepCount_ + 2) * tStep_) / dt_ + 100);

    for (int i = 0; i < (tStep_)/dt_; i++){
        double time = dt_ * i;
        if(footPose_[0](1) > footPose_[1](1)){
            lFoot_[index] = footPose_[0];
            rFoot_[index] = footPose_[1];
            lFootRot_[index] = AngleAxisd(0, Vector3d::UnitZ());
            rFootRot_[index] = AngleAxisd(0, Vector3d::UnitZ());
        }else{
            lFoot_[index] = footPose_[1];
            rFoot_[index] = footPose_[0];
            lFootRot_[index] = AngleAxisd(0, Vector3d::UnitZ());
            rFootRot_[index] = AngleAxisd(0, Vector3d::UnitZ());
        }
        index ++;
    }


    if (left_first){
        for (int step = 1; step < stepCount_ + 1 ; step ++){
            double theta_ini = (step-2)*theta_*yawSign_;
            double theta_end = (step)*theta_*yawSign_;
            if (step == 1)
                theta_ini = 0;
            if (step == stepCount_)
                theta_end = (step-1)*theta_*yawSign_;
            if (step % 2 == 0){     // Left is support, Right swings
                for (double time = 0; time < (1 - alpha_) * tDS_; time += dt_){
                    lFoot_[index] = footPose_[step];
                    lFootRot_[index] = lFootRot_[index - 1];
                    rFoot_[index] = footPose_[step - 1];
                    rFootRot_[index] = rFootRot_[index - 1];
                    index ++;
                }
                Vector3d* coefs = ankle5Poly(footPose_[step-1],footPose_[step+1], height_,tStep_-tDS_);
                double* theta_coefs = cubicInterpolate<double>(theta_ini, theta_end, 0, 0, tStep_ - tDS_);
                for(double time = 0.0; time < tStep_ - tDS_; time += dt_){
                    lFoot_[index] = footPose_[step];
                    lFootRot_[index] = lFootRot_[index - 1];
                    rFoot_[index] = coefs[0] + coefs[1] * time + coefs[2] * pow(time,2) + coefs[3] * pow(time,3) + coefs[4] * pow(time,4) + coefs[5] * pow(time,5);
                    rFootRot_[index] = AngleAxisd(theta_coefs[0] + theta_coefs[1] * time + theta_coefs[2]* pow(time,2) + theta_coefs[3] * pow(time,3), Vector3d::UnitZ());
                    index ++;
                }
                for (double time = 0; time < (alpha_) * tDS_; time += dt_){
                    lFoot_[index] = footPose_[step];
                    lFootRot_[index] = lFootRot_[index - 1];
                    rFoot_[index] = footPose_[step + 1];
                    rFootRot_[index] = rFootRot_[index - 1];
                    index ++;
                }
            }
            else{                  // Right is support, Left swings
                for (double time = 0; time < (1 - alpha_) * tDS_; time += dt_){
                    lFoot_[index] = footPose_[step - 1];
                    lFootRot_[index] = lFootRot_[index - 1];
                    rFoot_[index] = footPose_[step];
                    rFootRot_[index] = rFootRot_[index - 1];
                    index ++;
                }
                Vector3d* coefs = ankle5Poly(footPose_[step-1],footPose_[step+1], height_,tStep_-tDS_);
                double* theta_coefs = cubicInterpolate<double>(theta_ini, theta_end, 0, 0, tStep_ - tDS_);
                for(double time = 0.0; time < tStep_ - tDS_; time += dt_){
                    rFoot_[index] = footPose_[step];
                    rFootRot_[index] = rFootRot_[index - 1];
                    lFoot_[index] = coefs[0] + coefs[1] * time + coefs[2] * pow(time,2) + coefs[3] * pow(time,3) + coefs[4] * pow(time,4) + coefs[5] * pow(time,5);
                    lFootRot_[index] = AngleAxisd(theta_coefs[0] + theta_coefs[1] * time + theta_coefs[2]* pow(time,2) + theta_coefs[3] * pow(time,3), Vector3d::UnitZ());
                    index ++;
                }
                for (double time = 0; time < (alpha_) * tDS_; time += dt_){
                    lFoot_[index] = footPose_[step + 1];
                    lFootRot_[index] = lFootRot_[index - 1];
                    rFoot_[index] = footPose_[step];
                    rFootRot_[index] = rFootRot_[index - 1];
                    index ++;
                }
            }
        }
    }

    else{       // Right Foot Swings first
        for (int step = 1; step < stepCount_ + 1 ; step ++){

            double theta_ini = (step-2)*theta_*yawSign_;
            double theta_end = (step)*theta_*yawSign_;
            if (step == 1)
                theta_ini = 0;
            if (step == stepCount_)
                theta_end = (step-1)*theta_*yawSign_;
            
            if (step % 2 != 0){     // Left is support, Right swings
                for (double time = 0; time < (1 - alpha_) * tDS_; time += dt_){
                    lFoot_[index] = footPose_[step];
                    lFootRot_[index] = lFootRot_[index - 1];
                    rFoot_[index] = footPose_[step - 1];
                    rFootRot_[index] = rFootRot_[index - 1];
                    index ++;
                }
                Vector3d* coefs = ankle5Poly(footPose_[step-1],footPose_[step+1], height_,tStep_-tDS_);
                double* theta_coefs = cubicInterpolate<double>(theta_ini, theta_end, 0, 0, tStep_ - tDS_);
                for(double time = 0.0; time < tStep_ - tDS_; time += dt_){
                    lFoot_[index] = footPose_[step];
                    lFootRot_[index] = lFootRot_[index - 1];
                    rFoot_[index] = coefs[0] + coefs[1] * time + coefs[2] * pow(time,2) + coefs[3] * pow(time,3) + coefs[4] * pow(time,4) + coefs[5] * pow(time,5);
                    rFootRot_[index] = AngleAxisd(theta_coefs[0] + theta_coefs[1] * time + theta_coefs[2]* pow(time,2) + theta_coefs[3] * pow(time,3), Vector3d::UnitZ());
                    index ++;
                }
                for (double time = 0; time < (alpha_) * tDS_; time += dt_){
                    lFoot_[index] = footPose_[step];
                    lFootRot_[index] = lFootRot_[index - 1];
                    rFoot_[index] = footPose_[step + 1];
                    rFootRot_[index] = rFootRot_[index - 1];
                    index ++;
                }
            }
            else{                  // Right is support, Left swings
                for (double time = 0; time < (1 - alpha_) * tDS_; time += dt_){
                    lFoot_[index] = footPose_[step - 1];
                    lFootRot_[index] = lFootRot_[index - 1];
                    rFoot_[index] = footPose_[step];
                    rFootRot_[index] = rFootRot_[index - 1];
                    index ++;
                }
                Vector3d* coefs = ankle5Poly(footPose_[step-1],footPose_[step+1], height_,tStep_-tDS_);
                double* theta_coefs = cubicInterpolate<double>(theta_ini, theta_end, 0, 0, tStep_ - tDS_);
                for(double time = 0.0; time < tStep_ - tDS_; time += dt_){
                    rFoot_[index] = footPose_[step];
                    rFootRot_[index] = rFootRot_[index - 1];
                    lFoot_[index] = coefs[0] + coefs[1] * time + coefs[2] * pow(time,2) + coefs[3] * pow(time,3) + coefs[4] * pow(time,4) + coefs[5] * pow(time,5);
                    lFootRot_[index] = AngleAxisd(theta_coefs[0] + theta_coefs[1] * time + theta_coefs[2]* pow(time,2) + theta_coefs[3] * pow(time,3), Vector3d::UnitZ());
                    index ++;
                }
                for (double time = 0; time < (alpha_) * tDS_; time += dt_){
                    lFoot_[index] = footPose_[step + 1];
                    lFootRot_[index] = lFootRot_[index - 1];
                    rFoot_[index] = footPose_[step];
                    rFootRot_[index] = rFootRot_[index - 1];
                    index ++;
                }
            }
        }
    }

    Vector3d temp_left = lFoot_[index - 1];
    Vector3d temp_right = rFoot_[index - 1];
    for (int i = 0; i < (tStep_)/dt_; i++){
        double time = dt_ * i;
        lFoot_[index] = temp_left;
        lFootRot_[index] = lFootRot_[index - 1]; 
        rFoot_[index] = temp_right;
        rFootRot_[index] = rFootRot_[index - 1];
        index ++;
    }
    //cout << index << endl;
    MinJerk::write2File(lFoot_, length, "lFoot");
    MinJerk::write2File(rFoot_, length, "rFoot");
}

Ankle::~Ankle(){
    delete[] footPose_;
    delete[] lFoot_;
    delete[] rFoot_;
    delete[] rFootRot_;
    delete[] lFootRot_;
}