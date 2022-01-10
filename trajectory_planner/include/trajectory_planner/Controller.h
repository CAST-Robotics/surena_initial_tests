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

    private:
        Matrix3d K_p_;
        Matrix3d K_i_;
        Matrix3d K_zmp_;
        Matrix3d K_com_;
        Vector3d xiErrorInt;
        Vector3d CoM_;
        double dt_;
        double deltaZ_;
};