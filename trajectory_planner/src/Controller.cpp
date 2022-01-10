#include "Controller.h"


Controller::Controller(Matrix3d K_p, Matrix3d K_i, Matrix3d K_zmp, Matrix3d K_com){
    setK_p_(K_p);
    setK_i_(K_i);
    setK_zmp_(K_zmp);
    setK_com_(K_com);
    //this->K_p_ = K_p;
    //this->K_i_ = K_i;
    //this->K_zmp_ = K_zmp;
    //this->K_com_ = K_com;
    xiErrorInt << 0.0, 0.0, 0.0;
    double deltaZ_ = 0.0;
}

Vector3d Controller::dcmController(Vector3d xiRef, Vector3d xiDotRef, Vector3d xiReal, double deltaZVRP){
    Vector3d xiError = xiRef - xiReal;
    xiErrorInt += xiError * dt_;
    
    Vector3d rRefZMP;
    rRefZMP = xiRef - xiDotRef/sqrt(9.81/deltaZVRP) - (K_p_) * xiError + (K_i_) * xiErrorInt;
    return rRefZMP;
}

Vector3d Controller::comController(Vector3d xCOMRef, Vector3d xDotCOMRef, Vector3d xCOMReal, Vector3d rZMPRef, Vector3d rZMPReal){
    Vector3d xDotStar;
    xDotStar = xDotCOMRef - K_zmp_*(rZMPRef - rZMPReal) + K_com_*(xCOMRef - xCOMReal);
    CoM_ += xDotStar * dt_;
    return CoM_;
}

double Controller::footLenController(double delta_fz_d, double delta_fz, double kp, double kr){
    double deltaZ_dot = kp * (delta_fz_d - delta_fz);
    deltaZ_ += deltaZ_dot * dt_;
    return deltaZ_dot;
}

void Controller::setDt(double dt){
    this->dt_ = dt;
}

void Controller::setInitCoM(Vector3d init_com){
    this->CoM_ = init_com;
}

void Controller::setK_p_(Matrix3d K_p){
    this->K_p_ = K_p;
}
void Controller::setK_i_(Matrix3d K_i){
    this->K_i_ = K_i;
}
void Controller::setK_zmp_(Matrix3d K_zmp){
    this->K_zmp_ = K_zmp;
}
void Controller::setK_com_(Matrix3d K_com){
    this->K_com_ = K_com;
}

/*int main(){
    Matrix3d kp_;
    kp_<<1.0, 1.0, 1.0,
        1.0, 1.0, 1.0,
        1.0, 1.0, 1.0;
    Matrix3d ki_;
    ki_<<1.0, 1.0, 1.0,
        1.0, 1.0, 1.0,
        1.0, 1.0, 1.0;
    Matrix3d kzmp_;
    kzmp_<<1.0, 1.0, 1.0,
           1.0, 1.0, 1.0,
           1.0, 1.0, 1.0;
    Matrix3d kcom_;
    kcom_<<1.0, 1.0, 1.0,
           1.0, 1.0, 1.0,
           1.0, 1.0, 1.0;
    Vector3d xiRef;
    xiRef<<1.0, 1.0, 1.0;
    Vector3d xiDotRef;
    xiDotRef<<1.0, 1.0, 1.0;
    Vector3d xiReal;
    xiReal<<1.0, 1.0, 1.0;
    double deltaZVRP = 1.0;
    Vector3d result ;






    Controller kosammat(kp_, ki_ , kzmp_, kcom_);
    result = kosammat.dcmController(xiRef, xiDotRef, xiReal, deltaZVRP);
    cout << result <<endl;


}*/



