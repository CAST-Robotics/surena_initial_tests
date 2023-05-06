#pragma once

#include"eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include <eigen3/Eigen/Geometry>

#include "iostream"
#include "fstream"
#include <vector>
#include "math.h"
#include "cmath"

#include "MinJerk.h"

using namespace Eigen;
using namespace std;


const double K_G = 9.81;

class DCMPlanner: private MinJerk{
    /*
        Three dimentional trajectory Generation Based on Divergant Component of Motion (DCM)
        Reference Paper:              https://ieeexplore.ieee.org/abstract/document/7063218/
    */
    friend class Surena;
    public:
        DCMPlanner(double deltaZ, double stepTime, double doubleSupportTime, double dt, int stepCount = 6, double alpha = 0.5, double theta = 0.0);
        ~DCMPlanner();
        void setFoot(Vector3d rF[], int sign);
        Vector3d* getXiTrajectory();
        Vector3d* getXiDot();
        Vector3d* getCoM();
        Vector3d* getZMP();
        Vector3d* get_CoMDot();
        Matrix3d* yawRotGen();
    private:
        // Design Parameters
        double deltaZ_;
        double tStep_;
        double tDS_;
        double alpha_;
        double theta_;

        double dt_;         // sampling time
        int stepCount_;     // trajectories will be generated over how many steps

        // Trajectory Arrays
        Vector3d* xi_;
        Vector3d* xiDot_;
        Vector3d* COM_;
        Vector3d* CoMDot_;
        Vector3d* ZMP_;

        // Other Points required for Generating Trajectories
        Vector3d* rF_;
        Vector3d* rVRP_;
        Vector3d* xiEOS_;
        Vector3d* xiDSI_;
        Vector3d* xiDSE_;
        Matrix3d* yawRotation_;
        int yawSign_;
        // Functions for generating trajectories
        void updateVRP();
        void updateSS();
        void updateDS();
        void updateXiEoS();
        void updateXiDSPositions();
        Vector3d* minJerkInterpolate(Vector3d theta_ini, Vector3d theta_f, Vector3d theta_dot_ini, Vector3d theta_dot_f, double tf);
};