#pragma once

#include "Eigen/Dense"
#include "Eigen/Core"
#include <Eigen/Geometry>
#include "iostream"

using namespace Eigen;
using namespace std;

class Estimator{
    public:
        Estimator(){}
        void atitudeEulerEstimator(Vector3d &base_atitude, Vector3d gyro);
        void poseVelEstimator(Vector3d &base_vel, Vector3d &base_pos, Vector3d acc);
        inline void setDt(double dt){dt_ = dt;}

    private:
        double dt_;
};