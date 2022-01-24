#include <iostream>
#include <eigen3/Eigen/Eigen>
#include <math.h>


using namespace std;
using namespace Eigen;

class Controller {
    public:
        Controller(Matrix3d K_p_ = Matrix3d::Zero(3,3), Matrix3d K_i_ = Matrix3d::Zero(3,3), Matrix3d K_zmp_ = Matrix3d::Zero(3,3), Matrix3d K_com_ = Matrix3d::Zero(3,3));
        Vector3d dcmController(Vector3d xiRef, Vector3d xiDotRef, Vector3d xiReal, double deltaZVRP);
        Vector3d comController(Vector3d xCOMRef, Vector3d xDotCOMRef, Vector3d xCOMReal, Vector3d rZMPRef, Vector3d rZMPReal);
        void setK_p_(Matrix3d K_p);
        void setK_i_(Matrix3d K_i);
        void setK_zmp_(Matrix3d K_zmp);
        void setK_com_(Matrix3d K_com);
        void setDt(double dt);
        void setInitCoM(Vector3d init_com);
        double footLenController(double fz_d, double fz, double kp, double kr);
        Vector3d footOrientController(Vector3d delta_zmp, Vector3d tau, double f_d, double k_p, double k_d, double k_r, bool is_right);
        Vector3d footDampingController(Vector3d zmp, Vector3d f_measured, Vector3d tau_measured, Matrix3d cop_gain, bool is_right);

    private:
        Matrix3d K_p_;
        Matrix3d K_i_;
        Matrix3d K_zmp_;
        Matrix3d K_com_;
        Vector3d xiErrorInt;
        Vector3d CoM_;
        double dt_;
        double deltaZ_;

        Vector3d thetaAnkleR_;
        Vector3d thetaAnkleL_;
        
        Vector3d uOrientR_;
        Vector3d uOrientL_;
        Vector3d prevTau_[2];
        Vector3d prevTau_d_[2];
};