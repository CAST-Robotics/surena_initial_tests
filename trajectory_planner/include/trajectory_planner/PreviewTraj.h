#pragma once

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Geometry"

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>

#include "riccati_solver.h"
#include "ZMPPlanner.h"

using namespace std;
using namespace Eigen;

class PreviewTraj {
    public:
        PreviewTraj(ZMPPlanner* zmp_planner, double robot_height, int n=320, double dt=0.005);
        ~PreviewTraj();

        void setDt(double dt);
        void setInitCondition(const Vector3d& x0, const Vector3d& y0);
        void computeWeight();
        void computeTraj();
        void planYawTraj();
        inline vector<Vector3d> getCoMPos(){return CoMPos_;}
        inline vector<Matrix3d> getCoMRot(){return CoMRot_;}

    private:
        double dt_;
        double g_;
        double robotHeight_;
        Matrix3d A_;
        Vector3d b_;
        RowVector3d c_;
        MatrixXd Q_;
        MatrixXd R_;
        int N_;

        Vector3d x0_;
        Vector3d* x_;
        Vector3d* y_;
        MatrixXd P_;
        MatrixXd Gl_;
        MatrixXd Gx_;
        VectorXd Gd_;
        vector<Vector3d> CoMPos_;
        vector<Vector3d> CoMVel_;
        vector<Vector3d> CoMAcc_;
        vector<Matrix3d> CoMRot_;

        Vector3d error_;
        ZMPPlanner* ZMPPlanner_;
};