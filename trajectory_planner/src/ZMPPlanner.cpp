#include "ZMPPlanner.h"

ZMPPlanner::ZMPPlanner(){
}

ZMPPlanner::~ZMPPlanner(){
}


void ZMPPlanner::planInitialDSPZMP(){
    Vector3d init_zmp = (footSteps_[0] + footSteps_[1]) / 2;
    Vector3d final_zmp = footSteps_[1];
    Vector3d* coefs = this->minJerkInterpolate(init_zmp, final_zmp, Vector3d(0, 0, 0), Vector3d(0, 0, 0), initDSPDuration_);
    for(int i=0; i<int(initDSPDuration_ / dt_); i++){
        plannedZMP_.push_back(init_zmp + (i * dt_ / initDSPDuration_) * (final_zmp - init_zmp));
        // cout << plannedZMP_[i](0) << ", " << plannedZMP_[i](1) << endl;
    }
}

void ZMPPlanner::planStepsZMP(){
    for(int i=2; i<footStepCount_ - 1; i++){
        // single support phase
        Vector3d init_zmp = plannedZMP_[plannedZMP_.size() - 1];
        for(int j=0; j<int(SSPDuration_ / dt_); j++){
            plannedZMP_.push_back(init_zmp);
            // cout << init_zmp(0) << ", " << init_zmp(1) << endl;
        }
        // double support phase
        init_zmp = plannedZMP_[plannedZMP_.size() - 1];
        Vector3d final_zmp = footSteps_[i];
        Vector3d* coefs = this->minJerkInterpolate(init_zmp, final_zmp, Vector3d(0, 0, 0), Vector3d(0, 0, 0), DSPDuration_);
        for(int j=0; j<int(DSPDuration_ / dt_); j++){
            plannedZMP_.push_back(coefs[3] * pow(j * dt_, 3) + coefs[2] * pow(j * dt_, 2) + coefs[1] * j * dt_ + coefs[0]);
            // cout << plannedZMP_[plannedZMP_.size() - 1](0) << ", " << plannedZMP_[plannedZMP_.size() - 1](1) << endl;
        }
    }
}

void ZMPPlanner::planFinalDSPZMP(){
    Vector3d init_zmp = plannedZMP_[plannedZMP_.size() - 1];
    Vector3d final_zmp = (footSteps_[footStepCount_ - 1] + init_zmp) / 2;
    Vector3d* coefs = this->minJerkInterpolate(init_zmp, final_zmp, Vector3d(0, 0, 0), Vector3d(0, 0, 0), finalDSPDuration_);

    for(int j=0; j<int(SSPDuration_ / dt_); j++){
        plannedZMP_.push_back(init_zmp);
        // cout << init_zmp(0) << ", " << init_zmp(1) << endl;
    }

    for(int i=0; i<int(finalDSPDuration_ / dt_); i++){
        plannedZMP_.push_back(init_zmp + (i * dt_ / finalDSPDuration_) * (final_zmp - init_zmp));
        // cout << plannedZMP_[plannedZMP_.size() - 1](0) << ", " << plannedZMP_[plannedZMP_.size() - 1](1) << endl;
    }
}

Vector3d ZMPPlanner::getZMP(int iter){
    if(iter < plannedZMP_.size()){
        return plannedZMP_[iter];
    }
    else{
        //cout << "ZMPPlanner::getZMP: iter out of range" << endl;
        return plannedZMP_[plannedZMP_.size() - 1];
    }
}