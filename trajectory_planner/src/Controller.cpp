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
    uOrientR_ << 0, 0, 0;
    uOrientL_ << 0, 0, 0;
    // Index 0 is for right foot
    prevTau_[0] << 0, 0, 0;
    prevTau_d_[1] << 0, 0, 0;
    prevTau_[0] << 0, 0, 0;
    prevTau_d_[1] << 0, 0, 0;
    thetaAnkleR_ << 0, 0, 0;
    thetaAnkleL_ << 0, 0, 0;
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
    double deltaZ_dot = kp * (delta_fz_d - delta_fz) - kr * deltaZ_;
    deltaZ_ += deltaZ_dot * dt_;
    return deltaZ_;
}

Vector3d Controller::footOrientController(Vector3d delta_zmp, Vector3d tau, double f_d, double k_p, double k_d, double k_r, bool is_right){
    Vector3d tau_d, u_dot, diff, diff_d;
    tau_d(1) = f_d * delta_zmp(0);
    tau_d(0) = f_d * delta_zmp(1);
    
    int index = 0;
    if(!is_right)
        index = 1;

    diff_d = 1/dt_ *(tau_d - prevTau_d_[index]);
    diff = 1/dt_ *(tau - prevTau_[index]);
    
    prevTau_[index] = tau;
    prevTau_d_[index] = tau_d;

    if(is_right){
        u_dot = k_p * (tau_d - tau) + k_d * (diff_d - diff) - k_r * (uOrientR_);
        uOrientR_ += u_dot * dt_;
        return uOrientR_;
    }else{
        u_dot = k_p * (tau_d - tau) + k_d * (diff_d - diff) - k_r * (uOrientL_);
        uOrientL_ += u_dot * dt_;
        return uOrientL_;
    }

}

Vector3d Controller::footDampingController(Vector3d zmp, Vector3d f_measured, Vector3d tau_measured, Matrix3d cop_gain, bool is_right){
    Vector3d theta_dot = cop_gain * (zmp.cross(f_measured) - tau_measured);
    if(is_right){
        thetaAnkleR_ += theta_dot * dt_;
        return thetaAnkleR_;
    }else{
        thetaAnkleL_ += theta_dot * dt_;
        return thetaAnkleL_;
    }
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

