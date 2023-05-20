#pragma once

#include "MinJerk.h"

class ZMPPlanner : public MinJerk {
public:
    ZMPPlanner();
    virtual ~ZMPPlanner();
    void planInitialDSPZMP();
    void planStepsZMP();
    void planFinalDSPZMP();
    Vector3d getZMP(int iter);
    inline vector<Vector3d> getZMPTraj() { return plannedZMP_; }
    
private:
    vector<Vector3d> plannedZMP_;
};