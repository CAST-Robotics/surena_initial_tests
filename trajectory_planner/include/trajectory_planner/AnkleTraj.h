#pragma once

#include "MinJerk.h"

class AnkleTraj : public MinJerk {
public:
    AnkleTraj();
    virtual ~AnkleTraj(){}

    void planStance(Vector3d r_foot, Vector3d l_foot, double yaw, double time);
    void planSteps();
    inline vector<Vector3d> getRAnklePos(){return plannedRAnklePos_;}
    inline vector<Vector3d> getLAnklePos(){return plannedLAnklePos_;}
    inline vector<Matrix3d> getRAnkleRot(){return plannedRAnkleRot_;}
    inline vector<Matrix3d> getLAnkleRot(){return plannedLAnkleRot_;}

    void planInitialDSP();
    void planFinalDSP();
    
private:
    vector<Vector3d> plannedRAnklePos_;
    vector<Matrix3d> plannedRAnkleRot_;
    vector<Vector3d> plannedLAnklePos_;
    vector<Matrix3d> plannedLAnkleRot_;
};