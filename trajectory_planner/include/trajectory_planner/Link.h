#pragma once

#include"Eigen/Dense"
#include "Eigen/Core"
#include <Eigen/Geometry>

#include "iostream"
#include "fstream"
#include <vector>
#include "math.h"
#include "cmath"

using namespace Eigen;
using namespace std;


class _Link{
    friend class Robot;
    public:
        _Link(short int ID, Vector3d a, Vector3d b, double m, Matrix3d inertia, _Link* parent=NULL);
        _Link(){}
        _Link(const _Link& source);
        ~_Link();

        double q();
        double dq();
        void update(double q, double dq, double ddq);
        void initPose(Vector3d p, Matrix3d r);
        short int getID();
        _Link* getParent();
        Vector3d getPose();
        Matrix3d getRot();

        MatrixXd FK();
        MatrixXd updateJacobian();
        MatrixXd getVel();
        void setParams(short int ID, Vector3d a, Vector3d b, double m, Matrix3d inertia, _Link* parent=NULL);
        
    private:
        ////////////////// Link Properties ///////////////////////
        short int ID_;
        _Link* parent_;
        Vector3d p_;            // Link position (WRT world frame)
        Matrix3d R_;            // Link rotation (WRT world frame)
        Vector3d v_;            // Linear Velocity in world fram
        Vector3d w_;            // Angular Velocity in world frame
        double q_;              // joint angle
        double dq_;             // joint angular velocity
        double ddq_;            // joint angular acceleration

        Vector3d a_;            // joint Axis WRT parent frame
        Vector3d b_;            // joint position WRT parent frame
        
        double m_;              // mass
        Matrix3d I_;            // Inertia matrix


        //////////////////// private methods /////////////////////
        MatrixXd transformation();
        Matrix3d rodrigues(Vector3d w, double dt);
};