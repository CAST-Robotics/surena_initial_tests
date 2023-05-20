#include "Robot.h" 

using json = nlohmann::json;

void write2File(Vector3d* input, int size, string file_name="data"){
    ofstream output_file(file_name + ".csv");
    for(int i=0; i<size; i++){
         output_file << input[i](0) << " ,";
         output_file << input[i](1) << " ,";
         output_file << input[i](2) << " ,";
         output_file << "\n";
    }
    output_file.close();
}


Robot::Robot(ros::NodeHandle* nh, std::string config_path): nh_(nh),robotConfigPath_(config_path)
{   
    initROSCommunication();
    initializeRobotParams();

    bumpSensorCalibrated_ = false; // doc
    isTrajAvailable_ = false; // doc
    dataSize_ = 0;
    bumpBiasR_ = -56.0;
    bumpBiasL_ = -57.75;

    Matrix3d kp, ki, kcom, kzmp;
    kp << 1,0,0,0,1,0,0,0,0;
    ki = MatrixXd::Zero(3, 3);
    kcom = MatrixXd::Zero(3, 3);
    kzmp = MatrixXd::Zero(3, 3);
    onlineWalk_ = new Controller(kp, ki, kzmp, kcom);

    ankleColide_ = new Collision(soleXFront_, soleY_, soleXBack_, soleMinDist_);
    
    estimator_ = new Estimator();
}

void Robot::initROSCommunication() {
    zmpDataPub_ = nh_->advertise<geometry_msgs::Point>("/zmp_position", 100);
    comDataPub_ = nh_->advertise<geometry_msgs::Twist>("/com_data", 100);
    xiDataPub_ = nh_->advertise<geometry_msgs::Twist>("/xi_data", 100);
}

void Robot::initializeRobotParams() {
    std::ifstream f(robotConfigPath_);
    json config = json::parse(f);
    thigh_ = config["thigh"];      // SR1: 0.3535, Surena4: 0.37, Surena5: 0.36
    shank_ = config["shank"];      // SR1: 0.3, Surena4: 0.36, Surena5: 0.35
    torso_ = config["torso"];      // SR1: 0.09, Surena4: 0.115, Surena5: 0.0975
    soleXFront_ = config["sole_x_front"]; 
    soleXBack_ = config["sole_x_back"];    
    soleY_ = config["sole_y"];               
    soleMinDist_ = config["soles_min_distance"];
    rSole_ << config["sole_r"][0], config["sole_r"][1], config["sole_r"][2];
    lSole_ << config["sole_l"][0], config["sole_l"][1], config["sole_l"][2];

    totalMass_ = config["total_mass"];   // SR1: ?, Surena4: 48.3, Surena5: 66.5(Solid: 43.813) 62

    Vector3d a[12];
    Vector3d b[12];
    Vector3d com_pos[13];
    double links_mass[13];
    links_mass[0] = config["links_mass"][0];
    com_pos[0] << config["links_com_position"][0][0], config["links_com_position"][0][1], config["links_com_position"][0][2];

    for(int i=0; i<12; i++){
        links_mass[i+1] = config["links_mass"][i+1];
        com_pos[i+1] << config["links_com_position"][i+1][0], config["links_com_position"][i+1][1], config["links_com_position"][i+1][2];
        a[i] << config["joint_axis"][i][0], config["joint_axis"][i][1], config["joint_axis"][i][2];
        b[i] << config["joint_position"][i][0], config["joint_position"][i][1], config["joint_position"][i][2];
    }
    initializeLinkObjects(a, b, com_pos, links_mass);
}

void Robot::initializeLinkObjects(Vector3d a[], Vector3d b[], Vector3d com_pos[], double links_mass[]) {
    
    links_[0] = new _Link(0, Vector3d::Ones(3), Vector3d(0, 0, thigh_ + shank_), links_mass[0],
                              Matrix3d::Identity(3,3), com_pos[0]);
    links_[0]->initPose(Vector3d(0.0, 0.0, 0.0), Matrix3d::Identity(3, 3));

    for (int i = 0; i < 12; i++) {
        if (i == 6)
            links_[i+1] = new _Link(i+1, a[i], b[i], links_mass[i+1], Matrix3d::Identity(3,3),
                                    com_pos[i+1], links_[0]);
        else
            links_[i+1] = new _Link(i+1, a[i], b[i], links_mass[i+1], Matrix3d::Identity(3, 3),
                                    com_pos[i+1], links_[i]);
    }
}

void Robot::spinOnline(int iter, double config[], double jnt_vel[], Vector3d torque_r, Vector3d torque_l, 
                       double f_r, double f_l, Vector3d gyro, Vector3d accelerometer, int bump_r[], int bump_l[],
                       double* joint_angles, int& status)
{
    
    // updateRobotState(config, jnt_vel, torque_r, torque_l, f_r, f_l, gyro, accelerometer);

    int traj_index = findTrajIndex(trajSizes_, trajSizes_.size(), iter);

    // if(!bumpSensorCalibrated_ && robotPhase_[iter] == 0){
    //     bumpBiasR_ = 0.25 * (bump_r[0] + bump_r[1] + bump_r[2] + bump_r[3]);
    //     bumpBiasL_ = 0.25 * (bump_l[0] + bump_l[1] + bump_l[2] + bump_l[3]);
    // }

    if(robotControlState_[traj_index] == Robot::WALK){
        bumpSensorCalibrated_ = true;
        runFootLenController(iter, f_l, f_r, traj_index);
        
        // runBumpFootOrientController(iter, bump_r, bump_l);
        
        runEarlyContactController(iter, bump_r, bump_l);

        // runFootOrientController();

        // runZMPAdmitanceController();
    }
    else if(robotControlState_[traj_index] == Robot::IDLE){
        runFootLenController(iter, f_l, f_r, traj_index);
    }

    if(ankleColide_->checkColission(_lAnklePos_[iter], _rAnklePos_[iter], _lAnkleRot_[iter], _rAnkleRot_[iter])){
        status = 1;
        cout << "Collision Detected in Ankles!" << endl;
    }

    cout << _CoMPos_[iter](0) << ", " << _CoMPos_[iter](1) << ", " << _CoMPos_[iter](2) << ", ";
    cout << _lAnklePos_[iter](0) << ", " << _lAnklePos_[iter](1) << ", " << _lAnklePos_[iter](2) << ", ";
    cout << _rAnklePos_[iter](0) << ", " << _rAnklePos_[iter](1) << ", " << _rAnklePos_[iter](2) << endl;

    doIK(_CoMPos_[iter], _CoMRot_[iter], _lAnklePos_[iter], _lAnkleRot_[iter], _rAnklePos_[iter], _rAnkleRot_[iter]);

    for(int i = 0; i < 12; i++)
        joint_angles[i] = joints_[i];     // right leg(0-5) & left leg(6-11)
}

void Robot::runFootLenController(int iter, double f_l, double f_r, int traj_index){
    Vector3d l_wrench;
    Vector3d r_wrench;
    double deltaFd = 0;
    if(robotControlState_[traj_index] == Robot::WALK){
        distributeFT(_ZMPD_[iter - trajSizes_[0]], _rAnklePos_[iter], _lAnklePos_[iter], r_wrench, l_wrench);
        deltaFd = floor((l_wrench(0) - r_wrench(0)) * 10) / 10;
    }
    double delta_z = onlineWalk_->footLenController(deltaFd, floor((f_l - f_r) * 10) / 10, 0.00006, 0.0, 1.0);
    _lAnklePos_[iter](2) -= 0.5 * delta_z;
    _rAnklePos_[iter](2) += 0.5 * delta_z;
    // cout << delta_z << ',' << f_r << ',' << f_l << ',' << floor((f_l - f_r) * 10) / 10 << endl;
}

void Robot::runBumpFootOrientController(int iter, int bump_r[], int bump_l[])
{
    Vector3d delta_theta_r(0, 0, 0);
    Vector3d delta_theta_l(0, 0, 0);

    if (robotPhase_[iter] == 2){
        delta_theta_l = onlineWalk_->bumpFootOrientController(bump_l, Vector3d::Zero(), 2.5 / 300.0, 0.0, 6.0, false);
        delta_theta_r = onlineWalk_->bumpFootOrientController(bump_r, Vector3d::Zero(), 2.5 / 300.0, 0.0, 6.0, true);
    }else if(robotPhase_[iter] == 3){
        delta_theta_l = onlineWalk_->bumpFootOrientController(bump_l, Vector3d::Zero(), 2.5 / 300.0, 0.0, 6.0, false);
        delta_theta_r = onlineWalk_->bumpFootOrientController(bump_r, Vector3d::Zero(), 2.5 / 300.0, 0.0, 6.0, true);
    }else if(robotPhase_[iter] == 1){
        int temp_bump[] = {0, 0, 0, 0};
        delta_theta_l = onlineWalk_->bumpFootOrientController(temp_bump, Vector3d::Zero(), 2.5 / 300.0, 0.0, 6.0, false);
        delta_theta_r = onlineWalk_->bumpFootOrientController(temp_bump, Vector3d::Zero(), 2.5 / 300.0, 0.0, 6.0, true);
    }else{
        delta_theta_l = onlineWalk_->bumpFootOrientController(bump_l, Vector3d::Zero(), 0.0 / 300.0, 0.0, 0.0, false);
        delta_theta_r = onlineWalk_->bumpFootOrientController(bump_r, Vector3d::Zero(), 0.0 / 300.0, 0.0, 0.0, true);
    }

    Matrix3d delta_rot_r;
    Matrix3d delta_rot_l;
    delta_rot_r = AngleAxisd(delta_theta_r(2), Vector3d::UnitZ()) * AngleAxisd(delta_theta_r(1), Vector3d::UnitY()) * AngleAxisd(delta_theta_r(0), Vector3d::UnitX());
    delta_rot_l = AngleAxisd(delta_theta_l(2), Vector3d::UnitZ()) * AngleAxisd(delta_theta_l(1), Vector3d::UnitY()) * AngleAxisd(delta_theta_l(0), Vector3d::UnitX());
    
    _rAnkleRot_[iter] = _rAnkleRot_[iter] * delta_rot_r;
    _lAnkleRot_[iter] = _lAnkleRot_[iter] * delta_rot_l;
}

void Robot::runEarlyContactController(int iter, int bump_r[], int bump_l[]){
    double r_mean_bump = 0.25 * (bump_r[0] + bump_r[1] + bump_r[2] + bump_r[3]);
    double l_mean_bump = 0.25 * (bump_l[0] + bump_l[1] + bump_l[2] + bump_l[3]);

    double r_bump_d, l_bump_d;
    distributeBump(_rAnklePos_[iter](2), _rAnklePos_[iter](2), r_bump_d, l_bump_d);
    
    Vector3d delta_r_foot(0, 0, 0);
    Vector3d delta_l_foot(0, 0, 0);
    
    if(r_mean_bump < -20 && _rAnklePos_[iter](2) < _rAnklePos_[iter - 1](2))
        delta_r_foot = onlineWalk_->earlyContactController(bump_r, r_bump_d, 0.004, 3, true);
    else
        delta_r_foot = onlineWalk_->earlyContactController(bump_r, r_bump_d, 0.0, 3, true);

    if(l_mean_bump < -20 && _lAnklePos_[iter](2) < _lAnklePos_[iter - 1](2))
        delta_l_foot = onlineWalk_->earlyContactController(bump_l, l_bump_d, 0.004, 3, false);
    else
        delta_l_foot = onlineWalk_->earlyContactController(bump_l, l_bump_d, 0.0, 3, false);

    //Vector3d delta_r_foot = onlineWalk_->earlyContactController(bump_r, r_bump_d, 0.007, 5, true);
    //Vector3d delta_l_foot = onlineWalk_->earlyContactController(bump_l, l_bump_d, 0.007, 5, false);

    _rAnklePos_[iter] = _rAnklePos_[iter] + delta_r_foot;
    _lAnklePos_[iter] = _lAnklePos_[iter] + delta_l_foot;
    // cout << _rAnklePos_[iter](2) << _lAnklePos_[iter](2) << endl;
    // rAnklePos_[iter] = rAnklePos_[iter] + (0.5 * delta_r_foot - 0.5 * delta_l_foot);
    // lAnklePos_[iter] = lAnklePos_[iter] + (-0.5 * delta_r_foot + 0.5 * delta_l_foot);
}

void Robot::runFootOrientController(){
    //Vector3d delta_theta = onlineWalk_->footDampingController(Vector3d::Zero(), Vector3d(0, 0, f_r), torque_r, gain, true);
    // Vector3d delta_theta_r(0, 0, 0);
    // Vector3d delta_theta_l(0, 0, 0);
    // // delta_theta_r = onlineWalk_->footOrientController(Vector3d(0, 0, 0), torque_r, -0.05, 0.000, 2, true);
    // // delta_theta_l = onlineWalk_->footOrientController(Vector3d(0, 0, 0), torque_l, -0.05, 0.000, 2, false);
    // if(robotState_[iter] == 2){
    // //    delta_theta_r = onlineWalk_->footOrientController(Vector3d(r_wrench(1), r_wrench(2), 0), torque_r, 0.001, 0, 1, true);
    //    delta_theta_r = onlineWalk_->footOrientController(Vector3d(0, 0, 0), torque_r, 0, 0.000, 0.5, true);
    //    delta_theta_l = onlineWalk_->footOrientController(Vector3d(0, 0, 0), torque_l, -0.03, 0.000, 4, false);
    // }else if(robotState_[iter] == 3){
    //    delta_theta_r = onlineWalk_->footOrientController(Vector3d(0, 0, 0), torque_r, -0.03, 0.000, 4, true);
    //    delta_theta_l = onlineWalk_->footOrientController(Vector3d(0, 0, 0), torque_l, 0, 0.000, 0.5, false);
    // }else if(robotState_[iter] == 1){
    //    delta_theta_r = onlineWalk_->footOrientController(Vector3d(0, 0, 0), Vector3d(0, 0, 0), 0, 0.000, 0.5, true);
    // //    delta_theta_l = onlineWalk_->footOrientController(Vector3d(l_wrench(1), l_wrench(2), 0), torque_l, 0.001, 0, 1, false);
    //    delta_theta_l = onlineWalk_->footOrientController(Vector3d(0, 0, 0), Vector3d(0, 0, 0), 0, 0.000, 0.5, false);
    // }
    // Matrix3d delta_rot_r;
    // Matrix3d delta_rot_l;
    // delta_rot_r = AngleAxisd(delta_theta_r(2), Vector3d::UnitZ()) * AngleAxisd(delta_theta_r(1), Vector3d::UnitY()) * AngleAxisd(delta_theta_r(0), Vector3d::UnitX());
    // delta_rot_l = AngleAxisd(delta_theta_l(2), Vector3d::UnitZ()) * AngleAxisd(delta_theta_l(1), Vector3d::UnitY()) * AngleAxisd(delta_theta_l(0), Vector3d::UnitX());
    // rAnkleRot_[iter] = rAnkleRot_[iter] * delta_rot_r;
    // lAnkleRot_[iter] = lAnkleRot_[iter] * delta_rot_l;
}

void Robot::runZMPAdmitanceController()
{
    // Matrix3d kp = Vector3d(1.5, 1, 0).asDiagonal();
    // Matrix3d kc = Vector3d(4.5, 4, 0).asDiagonal();
    // if(robotPhase_[iter] == 2){
    //     Vector3d  temp = onlineWalk_->ZMPAdmitanceComtroller_(CoMPos_[iter], FKBase_[iter], rZMP_, Vector3d(0.02, 0, 0), kp, kc);
    //     CoMPos_[iter] += temp;
    //     // cout << temp(0) << "," << temp(1)  << "," << temp(2)  << "," << rZMP_(0) << "," << rZMP_(1)  << "," << rZMP_(2)  << "," << CoMPos_[iter](0) << "," << CoMPos_[iter](1)  << "," << FKBase_[iter](0) << "," << FKBase_[iter](1)  << ","; 
    // }
    // else if (robotPhase_[iter] == 3){
    //     Vector3d  temp = onlineWalk_->ZMPAdmitanceComtroller_(CoMPos_[iter], FKBase_[iter], lZMP_, Vector3d(0.02, 0, 0), kp, kc);
    //     CoMPos_[iter] += temp;
    //     // cout << temp(0) << "," << temp(1)  << "," << temp(2)  << "," << lZMP_(0) << "," << lZMP_(1)  << "," << lZMP_(2)  << "," << CoMPos_[iter](0) << "," << CoMPos_[iter](1)  << "," << FKBase_[iter](0) << "," << FKBase_[iter](1)  << ","; 
    // }
    // else if (robotPhase_[iter] == 1){
    //     Vector3d  temp = onlineWalk_->ZMPAdmitanceComtroller_(CoMPos_[iter], CoMPos_[iter], Vector3d(0, 0, 0), Vector3d(0, 0, 0), kp, kc);
    //     CoMPos_[iter] += temp;
    //     // cout << temp(0) << "," << temp(1)  << "," << temp(2)  << "," << 0 << "," << 0  << "," << 0  << "," << CoMPos_[iter](0) << "," << CoMPos_[iter](1)  << "," << FKBase_[iter](0) << "," << FKBase_[iter](1)  << ","; 
    // }
    // else {
    //     Vector3d  temp = onlineWalk_->ZMPAdmitanceComtroller_(CoMPos_[iter], FKBase_[iter], realZMP_[iter], Vector3d(0, 0, 0), kp, kc);
    //     CoMPos_[iter] += temp;
    //     // cout << temp(0) << "," << temp(1)  << "," << temp(2)  << "," << realZMP_[iter](0) << "," << realZMP_[iter](1) << "," << 0 << "," << CoMPos_[iter](0) << "," << CoMPos_[iter](1)  << "," << FKBase_[iter](0) << "," << FKBase_[iter](1)  << ","; 
    // }
}

void Robot::updateRobotState(double config[], double jnt_vel[], Vector3d torque_r, Vector3d torque_l, double f_r, double f_l, Vector3d gyro, Vector3d accelerometer){
    
    // update joint positions
    for (int i = 0; i < 13; i ++){
        links_[i]->update(config[i], jnt_vel[i], 0.0);
    }

    // Do the Forward Kinematics for Lower Limb
    links_[12]->FK();
    links_[6]->FK();    // update all raw values of sensors and link states
    
    // Interpret IMU data
    Vector3d change_attitude;
    //Vector3d base_attitude = links_[0]->getRot().eulerAngles(2, 1, 0); // rot(x) * rot(y) * rot(z) , eulerAngles() outputs are in the ranges [0:pi] * [-pi:pi] * [-pi:pi]
    Vector3d base_attitude = links_[0]->getEuler();
    Vector3d base_vel = links_[0]->getLinkVel();
    Vector3d base_pos = links_[0]->getPose();
    estimator_->atitudeEulerEstimator(base_attitude, gyro);
    Matrix3d rot = (AngleAxisd(base_attitude[2], Vector3d::UnitZ()) // roll, pitch, yaw
                  * AngleAxisd(base_attitude[1], Vector3d::UnitY())
                  * AngleAxisd(base_attitude[0], Vector3d::UnitX())).matrix();
    //links_[0]->setEuler(base_attitude);
    links_[0]->setRot(rot);
    links_[0]->setOmega(gyro);
    
    // Update swing/stance foot
    if ((links_[12]->getPose()(2) < links_[6]->getPose()(2)) && (robotPhase_[index_] == 3)){
        rightSwings_ = true;
        leftSwings_ = false;
    }
    else if ((links_[6]->getPose()(2) < links_[12]->getPose()(2)) && (robotPhase_[index_] == 2)){
        rightSwings_ = false;
        leftSwings_ = true;
    }
    else{
        rightSwings_ = false;
        leftSwings_ = false;
    }
    // Update CoM and Sole Positions
    updateSolePosition();

    // Calculate ZMP with FT data
    lZMP_ = getZMPLocal(torque_l, f_l);
    rZMP_ = getZMPLocal(torque_r, f_r);

    if (abs(f_l) < 20)
        f_l = 0;
    if (abs(f_r) < 20)
        f_r = 0;

    realZMP_[index_] = ZMPGlobal(rSole_ + lZMP_, lSole_ + rZMP_, f_r, f_l);
    zmpPosition_.x = realZMP_[index_](0);
    zmpPosition_.y = realZMP_[index_](1);
    zmpPosition_.z = realZMP_[index_](2);
    // zmpDataPub_.publish(zmpPosition_);
}

void Robot::updateSolePosition(){

    Matrix3d r_dot = this->rDot_(links_[0]->getRot());

    if(leftSwings_ && (!rightSwings_)){
        lSole_ = rSole_ - links_[6]->getPose() + links_[12]->getPose();
        //FKBase_[index_] = lSole_ - links_[0]->getRot() * links_[12]->getPose();
        FKBase_[index_] = lSole_ - links_[12]->getPose();
        FKBaseDot_[index_] = links_[6]->getVel().block(0, 0, 3, 1);
        //FKCoMDot_[index_] = -links_[0]->getRot() * links_[6]->getVel().block<3,1>(0, 0) - r_dot * links_[6]->getPose();
        rSoles_[index_] = rSole_;
        lSoles_[index_] = lSole_;
    }
    else if ((!leftSwings_) && rightSwings_){
        Matrix<double, 6, 1> q_dot;
        rSole_ = lSole_ - links_[12]->getPose() + links_[6]->getPose();
        //FKBase_[index_] = rSole_ - links_[0]->getRot() * links_[6]->getPose();
        FKBase_[index_] = rSole_ - links_[6]->getPose();
        FKBaseDot_[index_] = links_[12]->getVel().block(0, 0, 3, 1);
        //FKCoMDot_[index_] = -links_[0]->getRot() * links_[12]->getVel().block<3,1>(0, 0) - r_dot * links_[12]->getPose();
        rSoles_[index_] = rSole_;
        lSoles_[index_] = lSole_;
    }
    else{   // double support
        //FKBase_[index_] = rSole_ - links_[0]->getRot() * links_[6]->getPose();
        FKBase_[index_] = rSole_ - links_[6]->getPose();
        FKBaseDot_[index_] = links_[6]->getVel().block(0, 0, 3, 1);
        //FKCoMDot_[index_] = -links_[0]->getRot() * links_[6]->getVel().block<3,1>(0, 0) - r_dot * links_[6]->getPose();
        rSoles_[index_] = rSole_;
        lSoles_[index_] = lSole_;
    }
    FKCoM_[index_] = FKBase_ [index_] + CoM2Base();
    FKCoMDot_[index_] = FKBaseDot_[index_] + CoM2BaseVel();
    // 3-point backward formula for numeraical differentiation: 
    // https://www3.nd.edu/~zxu2/acms40390F15/Lec-4.1.pdf
    Vector3d f1, f0, f3, f2;
    if (index_ == 0) {
        f1 = FKCoM_[index_]; //com vel
        f0 = FKCoM_[index_]; //com vel
        f2 = Vector3d::Zero(3); //base vel
        f3 = Vector3d::Zero(3); //base vel
        
        }
    else if (index_ == 1) {
        f1 = FKCoM_[index_-1]; f0 = Vector3d::Zero(3);
        f3 = FKBase_[index_-1]; f2 = Vector3d::Zero(3);
        }
    else {
        f1 = FKCoM_[index_-1]; f0 = FKCoM_[index_-2];
        f3 = FKBase_[index_-1]; f2 = FKBase_[index_-2];
        } 
    FKBaseDot_[index_] = (f2 - 4 * f3 + 3 * FKBase_[index_])/(2 * this->dt_);
    FKCoMDot_[index_] = (f0 - 4 * f1 + 3 * FKCoM_[index_])/(2 * this->dt_);
    // if(index_ > 200){
    //     Vector3d posterior = CoMDot_[index_ - 201];
    //     double p = FKCoMDotP_[index_ - 201];
    //     estimator_->gaussianPredict(posterior, p, 0.2, CoMDot_[index_ - 200] - CoMDot_[index_ - 201]);
    //     estimator_->gaussianUpdate(posterior, p, FKCoMDot_[index_], 0.03);
    //     FKCoMDot_[index_] = posterior;
    //     FKCoMDotP_[index_ - 1] = p;
    // }
    // realXi_[index_] = FKBase_[index_] + FKBaseDot_[index_] / sqrt(K_G/CoM_height_);
    realXi_[index_] = FKCoM_[index_] + FKCoMDot_[index_] / sqrt(K_G/CoM_height_);
}

Vector3d Robot::getZMPLocal(Vector3d torque, double fz){
    // Calculate ZMP for each foot
    Vector3d zmp(0.0, 0.0, 0.0);
    if (fz == 0){
        //ROS_WARN("No Correct Force Value!");
        return zmp;
    }
    zmp(0) = -torque(1)/fz;
    zmp(1) = -torque(0)/fz;
    return zmp;
}

Vector3d Robot::ZMPGlobal(Vector3d zmp_r, Vector3d zmp_l, double f_r, double f_l){
    // Calculate ZMP during Double Support Phase
    Vector3d zmp(0.0, 0.0, 0.0);
    if (f_r + f_l == 0){
        //ROS_WARN("No Foot Contact, Check the Robot!");
        return zmp;
    }
    //assert(!(f_r + f_l == 0));
    return (zmp_r * f_r + zmp_l * f_l) / (f_r + f_l);
}

Vector3d Robot::CoM2Base(){
    Vector3d com;
    Vector3d mc = Vector3d::Zero();
    for(int i = 0; i < 13; i++){
        double m_i = links_[i]->getMass();
        Vector3d p_com_b = links_[i]->getPose() + links_[i]->getRot() * links_[i]->getLinkCoM();
        mc += m_i * p_com_b;
    }
    com = mc / totalMass_;
    return(com);
}

Vector3d Robot::CoM2BaseVel(){
    Vector3d m_p(0,0,0);
    Vector3d com_vel;
    for(int i = 0; i < 12; i ++){
        m_p += links_[i]->getMass() * (links_[0]->getOmega().cross(links_[0]->getRot() * links_[i]->getLinkCoM()) + 
                    links_[0]->getRot() * links_[i]->getLinkVel());
    }
    return m_p/ totalMass_;
}

void Robot::doIK(MatrixXd pelvisP, Matrix3d pelvisR, MatrixXd leftAnkleP, Matrix3d leftAnkleR, MatrixXd rightAnkleP, Matrix3d rightAnkleR){
    // Calculates and sets Robot Leg Configuration at each time step
    double* q_left = this->geometricIK(pelvisP, pelvisR, leftAnkleP, leftAnkleR, true);
    double* q_right = this->geometricIK(pelvisP, pelvisR, rightAnkleP, rightAnkleR, false);
    for(int i = 0; i < 6; i ++){
        joints_[i] = q_right[i];
        joints_[i+6] = q_left[i];
    }
    delete[] q_left;
    delete[] q_right;
}

Matrix3d Robot::Rroll(double phi){
    // helper Function for Geometric IK
    MatrixXd R(3,3);
    double c=cos(phi);
    double s=sin(phi);
    R<<1,0,0,0,c,-1*s,0,s,c;
    return R;
}

Matrix3d Robot::RPitch(double theta){
    // helper Function for Geometric IK
    MatrixXd Ry(3,3);
    double c=cos(theta);
    double s=sin(theta);
    Ry<<c,0,s,0,1,0,-1*s,0,c;
    return Ry;

}

Matrix3d Robot::rDot_(Matrix3d R){
    AngleAxisd angle_axis(R);
    Matrix3d r_dot, temp;
    temp << 0.0, -angle_axis.axis()(2), angle_axis.axis()(1),
             angle_axis.axis()(2), 0.0, -angle_axis.axis()(0),
             -angle_axis.axis()(1), angle_axis.axis()(0), 0.0;
    r_dot = temp * R;
    return r_dot;
}

double* Robot::geometricIK(MatrixXd p1, MatrixXd r1, MatrixXd p7, MatrixXd r7, bool isLeft){
    /*
        Geometric Inverse Kinematic for Robot Leg (Section 2.5  Page 53)
        Reference: Introduction to Humanoid Robotics by Kajita        https://www.springer.com/gp/book/9783642545351
        1 ----> Body        7-----> Foot
    */
   
    double* q = new double[6];  
    MatrixXd D(3,1);

    if (isLeft)
        D << 0.0,torso_,0.0;
    else
        D << 0.0,-torso_,0.0;
        
    MatrixXd r = r7.transpose() * (p1 + r1 * D - p7);
    double C = r.norm();
    double c3 = (pow(C,2) - pow(thigh_,2) - pow(shank_,2))/(2 * thigh_ * shank_);
    if (c3 >= 1){
        q[3] = 0.0;
        // Raise error
    }else if(c3 <= -1){
        q[3] = M_PI;
        // Raise error
    }else{
        q[3] = acos(c3);       // Knee Pitch
    }
    double q4a = asin((thigh_/C) * sin(M_PI - q[3]));
    q[5] = atan2(r(1,0),r(2,0));   //Ankle Roll
    if (q[5] > M_PI/2){
        q[5] = q[5] - M_PI;
        // Raise error
    }
    else if (q[5] < -M_PI / 2){
        q[5] = q[5] + M_PI;
        // Raise error
    }
    int sign_r2 = 1;
    if(r(2,0) < 0)
        sign_r2 = -1;
    q[4] = -atan2(r(0,0),sign_r2 * sqrt(pow(r(1,0),2) + pow(r(2,0),2))) - q4a;      // Ankle Pitch
    Matrix3d R = r1.transpose() * r7 * Rroll(-q[5]) * RPitch(-q[3] - q[4]);
    q[0] = atan2(-R(0,1),R(1,1));         // Hip Yaw
    if(abs(q[0]) > 3)
        q[0] = 0;
    q[1] = atan2(R(2,1), -R(0,1) * sin(q[0]) + R(1,1) * cos(q[0]));           // Hip Roll
    q[2] = atan2(-R(2,0), R(2,2));        // Hip Pitch
    return q;
}

int Robot::findTrajIndex(vector<int> arr, int n, int K)
{
    /*
        a binary search function to find the index of the running trajectory.
    */
    int start = 0;
    int end = n - 1;
    while (start <= end) {
        int mid = (start + end) / 2;
 
        if (arr[mid] == K)
            return mid + 1;
        else if (arr[mid] < K)
            start = mid + 1;
        else
            end = mid - 1;
    }
    return end + 1;
}

void Robot::distributeFT(Vector3d zmp, Vector3d r_foot,Vector3d l_foot, Vector3d &r_wrench, Vector3d &l_wrench){
    
    double k_f = abs((zmp(1) - r_foot(1))) / abs((r_foot(1) - l_foot(1)));
    l_wrench(0) = k_f * totalMass_ * K_G;
    r_wrench(0) = (1 - k_f) * totalMass_ * K_G;

    l_wrench(1) = l_wrench(0) * (zmp(1) - l_foot(1));
    r_wrench(1) = r_wrench(0) * (zmp(1) - r_foot(1));

    l_wrench(2) = l_wrench(0) * (zmp(0) - l_foot(0));
    r_wrench(2) = r_wrench(0) * (zmp(0) - r_foot(0));
}

void Robot::distributeBump(double r_foot_z, double l_foot_z, double &r_bump, double &l_bump){
    r_bump = max(bumpBiasR_, min(0.0, (bumpBiasR_ / 0.02) * (0.02 - r_foot_z)));
    l_bump = max(bumpBiasL_, min(0.0, (bumpBiasL_ / 0.02) * (0.02 - l_foot_z)));
}

// Generate footstep plan
void Robot::planFootSteps() {
    FootStepPlanner step_planner(torso_);
    ZMPPlanner zmp_planner;
    zmp_planner.setDt(dt_);
    if (use_file_)
    {
        string config_path = ros::package::getPath("trajectory_planner") + "/config/config.yaml";
        zmp_planner.setConfigPath(config_path);
    }
    else
    {
        step_planner.setParams(step_len_, step_width_, num_steps_, 0.0, theta_);
        step_planner.planSteps();
        zmp_planner.setFootStepsData(step_planner.getFootPrints(), step_planner.getFootYaws());
        zmp_planner.setParams(t_init_ds_, t_ds_, t_step_, t_final_ds_, swing_height_);
    }

    // Generate ZMP trajectory
    zmp_planner.planInitialDSPZMP();
    zmp_planner.planStepsZMP();
    zmp_planner.planFinalDSPZMP();
    vector<Vector3d> zmp = zmp_planner.getZMPTraj();
    _ZMPD_.insert(_ZMPD_.end(), zmp.begin(), zmp.end());
    ZMPPlanner_ = zmp_planner;
}

void Robot::getInitCondition(Vector3d& x0, Vector3d& y0)
{
    if(_CoMPos_.size() != 0)
    {
        x0 = Vector3d(0, 0, 0);
        y0 = Vector3d(_CoMPos_.back()(1), 0, 0);
    }
    else
    {
        x0 = Vector3d(0, 0, 0);
        y0 = Vector3d(0, 0, 0);
    }
}

// Generate CoM trajectory
void Robot::planCoMTraj() {
    PreviewTraj traj(&ZMPPlanner_, CoM_height_, 1.8 / dt_, dt_);

    Vector3d x0, y0;
    getInitCondition(x0, y0);
    traj.setInitCondition(x0, y0);

    traj.computeWeight();
    traj.computeTraj();

    vector<Vector3d> com = traj.getCoMPos();
    _CoMPos_.insert(_CoMPos_.end(), com.begin(), com.end());

    traj.planYawTraj();
    vector<Matrix3d> com_rot = traj.getCoMRot();
    _CoMRot_.insert(_CoMRot_.end(), com_rot.begin(), com_rot.end());
}

// Generate ankle trajectories
void Robot::planAnkleTraj() {
    AnkleTraj ank_traj;
    ank_traj.planInitialDSP();
    ank_traj.planSteps();
    ank_traj.planFinalDSP();
    vector<Vector3d> left_ankle_pos = ank_traj.getLAnklePos();
    _lAnklePos_.insert(_lAnklePos_.end(), left_ankle_pos.begin(), left_ankle_pos.end());
    vector<Matrix3d> left_ankle_rot = ank_traj.getLAnkleRot();
    _lAnkleRot_.insert(_lAnkleRot_.end(), left_ankle_rot.begin(), left_ankle_rot.end());
    vector<Vector3d> right_ankle_pos = ank_traj.getRAnklePos();
    _rAnklePos_.insert(_rAnklePos_.end(), right_ankle_pos.begin(), right_ankle_pos.end());
    vector<Matrix3d> right_ankle_rot = ank_traj.getRAnkleRot();
    _rAnkleRot_.insert(_rAnkleRot_.end(), right_ankle_rot.begin(), right_ankle_rot.end());
}


bool Robot::trajGen(int step_count, double t_step, double alpha, double t_double_support,
                    double COM_height, double step_length, double step_width, double dt,
                    double theta, double ankle_height, double step_height, bool use_file, 
                    double t_init_double_support, double t_final_double_support)
{
    int trajectory_size = int(((step_count + 2) * t_step) / dt);
    num_steps_ = step_count;
    CoM_height_ = COM_height;
    step_len_ = step_length;
    step_width_ = step_width;
    t_init_ds_ = t_init_double_support;
    t_ds_ = t_double_support;
    t_step_ = t_step;
    t_final_ds_ = t_final_double_support;
    swing_height_ = ankle_height;
    theta_ = theta;
    use_file_ = use_file;
    dt_ = dt;
    double init_COM_height = thigh_ + shank_;
    onlineWalk_->setDt(dt_);

    planFootSteps();

    planCoMTraj();

    planAnkleTraj();

    dataSize_ = _CoMPos_.size();
    trajSizes_.push_back(dataSize_);
    robotControlState_.push_back(Robot::WALK);
    isTrajAvailable_ = true;
    
    return true;
    // DCMPlanner* trajectoryPlanner = new DCMPlanner(CoM_height_, t_step, t_double_support, dt_, step_count + 2, alpha, theta);
    // Ankle* anklePlanner = new Ankle(t_step, t_double_support, ankle_height, alpha, step_count, dt_, theta);
    // Vector3d* dcm_rf = new Vector3d[step_count + 2];  // DCM rF
    // Vector3d* ankle_rf = new Vector3d[step_count + 2]; // Ankle rF
    // int sign;

    // if(theta == 0.0){   // Straight or Diagonal Walk
    //     int sign;
    //     if (step_width == 0){sign = 1;}
    //     else{sign = (step_width/abs(step_width));}

    //     ankle_rf[0] << 0.0, (torso_ + 0.0) * sign, 0.0;
    //     ankle_rf[1] << 0.0, (torso_ + 0.0) * -sign, 0.0;
    //     dcm_rf[0] << 0.0, 0.0, 0.0;
    //     dcm_rf[1] << 0.0, torso_ * -sign, 0.0;

    //     for(int i = 2; i <= step_count + 1; i ++){
    //         if (i == 2 || i == step_count + 1){
    //             ankle_rf[i] = ankle_rf[i-2] + Vector3d(step_length, step_width, step_height);
    //             dcm_rf[i] << ankle_rf[i-2] + Vector3d(step_length, step_width, step_height);
    //         }else{
    //             ankle_rf[i] = ankle_rf[i-2] + Vector3d(2 * step_length, step_width, step_height);
    //             dcm_rf[i] << ankle_rf[i-2] + Vector3d(2 * step_length, step_width, step_height);
    //         }
    //     }

    //     dcm_rf[step_count + 1] = 0.5 * (ankle_rf[step_count] + ankle_rf[step_count + 1]);
    //     ankle_rf[0] << 0.0, torso_ * sign, 0.0;
    //     ankle_rf[1] << 0.0, torso_ * -sign, 0.0;
    // }
    // else{   //Turning Walk
    //     double r = abs(step_length/theta);
    //     sign = abs(step_length) / step_length;
    //     ankle_rf[0] = Vector3d(0.0, -sign * torso_, 0.0);
    //     dcm_rf[0] = Vector3d::Zero(3);
    //     ankle_rf[step_count + 1] = (r + pow(-1, step_count) * torso_) * Vector3d(sin(theta * (step_count-1)), sign * cos(theta * (step_count-1)), 0.0) + 
    //                                 Vector3d(0.0, -sign * r, 0.0);
    //     for (int i = 1; i <= step_count; i ++){
    //         ankle_rf[i] = (r + pow(-1, i-1) * torso_) * Vector3d(sin(theta * (i-1)), sign * cos(theta * (i-1)), 0.0) + 
    //                                 Vector3d(0.0, -sign * r, 0.0);
    //         dcm_rf[i] = ankle_rf[i];
    //     }
    //     dcm_rf[step_count + 1] = 0.5 * (ankle_rf[step_count] + ankle_rf[step_count + 1]);
    // }

    // trajectoryPlanner->setFoot(dcm_rf, -sign);
    // xiDesired_ = trajectoryPlanner->getXiTrajectory();
    // zmpd_ = trajectoryPlanner->getZMP();
    // xiDot_ = trajectoryPlanner->getXiDot();
    // delete[] dcm_rf;
    // anklePlanner->updateFoot(ankle_rf, -sign);
    // anklePlanner->generateTrajectory();
    // delete[] ankle_rf;
    // onlineWalk_->setDt(dt);
    // estimator_->setDt(dt);
    // onlineWalk_->setBaseHeight(COM_height);
    // onlineWalk_->setBaseIdle(shank_ + thigh_);
    // onlineWalk_->setBaseLowHeight(0.65);
    // onlineWalk_->setInitCoM(Vector3d(0.0,0.0,CoM_height_));

    // if (dataSize_ != 0){
    //     dataSize_ += trajectory_size;

    //     CoMPos_ = appendTrajectory<Vector3d>(CoMPos_, trajectoryPlanner->getCoM(), dataSize_ - trajectory_size, trajectory_size);
    //     lAnklePos_ = appendTrajectory<Vector3d>(lAnklePos_, anklePlanner->getTrajectoryL(), dataSize_ - trajectory_size, trajectory_size);
    //     rAnklePos_ = appendTrajectory<Vector3d>(rAnklePos_, anklePlanner->getTrajectoryR(), dataSize_ - trajectory_size, trajectory_size);
    //     robotPhase_ = appendTrajectory<int>(robotPhase_, anklePlanner->getRobotState(), dataSize_ - trajectory_size, trajectory_size);

    //     CoMRot_ = appendTrajectory<Matrix3d>(CoMRot_, trajectoryPlanner->yawRotGen(), dataSize_ - trajectory_size, trajectory_size);
    //     lAnkleRot_ = appendTrajectory<Matrix3d>(lAnkleRot_, anklePlanner->getRotTrajectoryL(), dataSize_ - trajectory_size, trajectory_size);
    //     rAnkleRot_ = appendTrajectory<Matrix3d>(rAnkleRot_, anklePlanner->getRotTrajectoryR(), dataSize_ - trajectory_size, trajectory_size);      

    //     delete[] FKBase_;
    //     delete[] FKCoM_;
    //     delete[] FKCoMDot_;
    //     delete[] FKCoMDotP_;
    //     delete[] FKBaseDot_;
    //     delete[] realXi_;
    //     delete[] realZMP_;
    //     delete[] rSoles_;
    //     delete[] lSoles_;
        
    // }else{
    //     dataSize_ += trajectory_size;

    //     CoMPos_ = trajectoryPlanner->getCoM();
    //     lAnklePos_ = anklePlanner->getTrajectoryL();
    //     rAnklePos_ = anklePlanner->getTrajectoryR();
    //     robotPhase_ = anklePlanner->getRobotState();

    //     CoMRot_ = trajectoryPlanner->yawRotGen();
    //     lAnkleRot_ = anklePlanner->getRotTrajectoryR();
    //     rAnkleRot_ = anklePlanner->getRotTrajectoryL();
    // }
    // CoMDot_ = trajectoryPlanner->get_CoMDot();
    // trajSizes_.push_back(dataSize_);
    // robotControlState_.push_back(Robot::WALK);
    // isTrajAvailable_ = true;

    // FKBase_ = new Vector3d[dataSize_];
    // FKCoM_ = new Vector3d[dataSize_];
    // FKCoMDot_ = new Vector3d[dataSize_];
    // FKCoMDotP_ = new double[dataSize_];
    // FKCoMDotP_[0] = 2.0;
    // FKBaseDot_ = new Vector3d[dataSize_];
    // realXi_ = new Vector3d[dataSize_];
    // realZMP_ = new Vector3d[dataSize_];
    // rSoles_ = new Vector3d[dataSize_];
    // lSoles_ = new Vector3d[dataSize_];
}

bool Robot::generalTrajGen(double dt, double time, double init_com_pos[3], double final_com_pos[3], double init_com_orient[3], double final_com_orient[3],
                           double init_lankle_pos[3], double final_lankle_pos[3], double init_lankle_orient[3], double final_lankle_orient[3],
                           double init_rankle_pos[3], double final_rankle_pos[3], double init_rankle_orient[3], double final_rankle_orient[3])
{
    dt_ = dt;
    onlineWalk_->setDt(dt_);
    GeneralMotion* motion_planner = new GeneralMotion(dt_);
    motion_planner->_changeInPlace(Vector3d(init_com_pos[0], init_com_pos[1], init_com_pos[2]), 
                                  Vector3d(final_com_pos[0], final_com_pos[1], final_com_pos[2]), 
                                  Vector3d(init_com_orient[0], init_com_orient[1], init_com_orient[2]), 
                                  Vector3d(final_com_orient[0], final_com_orient[1], final_com_orient[2]),
                                  Vector3d(init_lankle_pos[0], init_lankle_pos[1], init_lankle_pos[2]), 
                                  Vector3d(final_lankle_pos[0], final_lankle_pos[1], final_lankle_pos[2]),
                                  Vector3d(init_lankle_orient[0], init_lankle_orient[1], init_lankle_orient[2]), 
                                  Vector3d(final_lankle_orient[0], final_lankle_orient[1], final_lankle_orient[2]),
                                  Vector3d(init_rankle_pos[0], init_rankle_pos[1], init_rankle_pos[2]), 
                                  Vector3d(final_rankle_pos[0], final_rankle_pos[1], final_rankle_pos[2]),
                                  Vector3d(init_rankle_orient[0], init_rankle_orient[1], init_rankle_orient[2]), 
                                  Vector3d(final_rankle_orient[0], final_rankle_orient[1], final_rankle_orient[2]),
                                  time);
    int trajectory_size = motion_planner->getLength();

    vector<Vector3d> com_pos = motion_planner->_getCoMPos();
    _CoMPos_.insert(_CoMPos_.end(), com_pos.begin(), com_pos.end());
    vector<Matrix3d> com_rot = motion_planner->_getCoMOrient();
    _CoMRot_.insert(_CoMRot_.end(), com_rot.begin(), com_rot.end());

    vector<Vector3d> lank = motion_planner->_getLAnklePos();
    _lAnklePos_.insert(_lAnklePos_.end(), lank.begin(), lank.end());
    vector<Matrix3d> lank_rot = motion_planner->_getLAnkleOrient();
    _lAnkleRot_.insert(_lAnkleRot_.end(), lank_rot.begin(), lank_rot.end());

    vector<Vector3d> rank = motion_planner->_getRAnklePos();
    _rAnklePos_.insert(_rAnklePos_.end(), rank.begin(), rank.end());
    vector<Matrix3d> rank_rot = motion_planner->_getRAnkleOrient();
    _rAnkleRot_.insert(_rAnkleRot_.end(), rank_rot.begin(), rank_rot.end());

    vector<int> robot_state = motion_planner->_getRobotState();
    robotState_.insert(robotState_.end(), robot_state.begin(), robot_state.end());  

    dataSize_ += trajectory_size;
    robotControlState_.push_back(Robot::IDLE);
    trajSizes_.push_back(dataSize_);
    isTrajAvailable_ = true;

    return true;
    // onlineWalk_->setDt(dt);
    // estimator_->setDt(dt);
    // onlineWalk_->setBaseIdle(shank_ + thigh_);
    // onlineWalk_->setBaseLowHeight(0.65);
    // onlineWalk_->setInitCoM(Vector3d(0.0,0.0,CoM_height_));

    // if (dataSize_ != 0){
    //     dataSize_ += trajectory_size;

    //     CoMPos_ = appendTrajectory<Vector3d>(CoMPos_, motion_planner->getCOMPos(), dataSize_ - trajectory_size, trajectory_size);
    //     lAnklePos_ = appendTrajectory<Vector3d>(lAnklePos_, motion_planner->getLAnklePos(), dataSize_ - trajectory_size, trajectory_size);
    //     rAnklePos_ = appendTrajectory<Vector3d>(rAnklePos_, motion_planner->getRAnklePos(), dataSize_ - trajectory_size, trajectory_size);
    //     robotPhase_ = appendTrajectory<int>(robotPhase_, motion_planner->getRobotState(), dataSize_ - trajectory_size, trajectory_size);

    //     CoMRot_ = appendTrajectory<Matrix3d>(CoMRot_, motion_planner->getCOMOrient(), dataSize_ - trajectory_size, trajectory_size);
    //     lAnkleRot_ = appendTrajectory<Matrix3d>(lAnkleRot_, motion_planner->getLAnkleOrient(), dataSize_ - trajectory_size, trajectory_size);
    //     rAnkleRot_ = appendTrajectory<Matrix3d>(rAnkleRot_, motion_planner->getRAnkleOrient(), dataSize_ - trajectory_size, trajectory_size);      

    //     delete[] FKBase_;
    //     delete[] FKCoM_;
    //     delete[] FKCoMDot_;
    //     delete[] FKCoMDotP_;
    //     delete[] FKBaseDot_;
    //     delete[] realXi_;
    //     delete[] realZMP_;
    //     delete[] rSoles_;
    //     delete[] lSoles_;
        
    // }else{
    //     dataSize_ += trajectory_size;

    //     CoMPos_ = motion_planner->getCOMPos();
    //     lAnklePos_ = motion_planner->getLAnklePos();
    //     rAnklePos_ = motion_planner->getRAnklePos();
    //     robotPhase_ = motion_planner->getRobotState();

    //     CoMRot_ = motion_planner->getCOMOrient();
    //     lAnkleRot_ = motion_planner->getLAnkleOrient();
    //     rAnkleRot_ = motion_planner->getRAnkleOrient();
    // }

    // trajSizes_.push_back(dataSize_);
    // robotControlState_.push_back(Robot::IDLE);
    // isTrajAvailable_ = true;
    // FKBase_ = new Vector3d[dataSize_];
    // FKCoM_ = new Vector3d[dataSize_];
    // FKCoMDot_ = new Vector3d[dataSize_];
    // FKCoMDotP_ = new double[dataSize_];
    // FKCoMDotP_[0] = 2.0;
    // FKBaseDot_ = new Vector3d[dataSize_];
    // realXi_ = new Vector3d[dataSize_];
    // realZMP_ = new Vector3d[dataSize_];
    // rSoles_ = new Vector3d[dataSize_];
    // lSoles_ = new Vector3d[dataSize_];
    // return true;
}

bool Robot::getJointAngs(int iter, double config[12], double jnt_vel[12], double right_ft[3],
                         double left_ft[3], int right_bump[4], int left_bump[4], double gyro[3],
                         double accelerometer[3], double jnt_command[12],int &status)
{
    /*
        function for returning joint angles. before calling this function, 
        you must first call trajectory planning functions. 
    */
    if (isTrajAvailable_)
    {
        index_ = iter;
        Vector3d right_torque(right_ft[1], right_ft[2], 0.0);
        Vector3d left_torque(left_ft[1], left_ft[2], 0.0);
        double robot_config[13];
        double robot_jnt_vel[13];
        robot_config[0] = 0;     //Pelvis joint angle
        robot_jnt_vel[0] = 0;  //Pelvis joint velocity
        for(int i = 1; i < 13; i++){
            robot_config[i] = config[i-1];
            robot_jnt_vel[i] = jnt_vel[i-1];  
        }
        status = 0;     // 0: Okay, 1: Ankle Collision
        this->spinOnline(iter, robot_config, robot_jnt_vel, right_torque, left_torque, right_ft[0], left_ft[0],
                         Vector3d(gyro[0], gyro[1], gyro[2]), Vector3d(accelerometer[0], accelerometer[1], accelerometer[2]),
                         right_bump, left_bump, jnt_command, status);
    }else{
        ROS_INFO("First call traj_gen service");
        return false;
    }
    return true;
}

bool Robot::resetTraj()
{
    // delete[] CoMPos_;
    // delete[] robotPhase_;
    // delete[] lAnklePos_;
    // delete[] rAnklePos_;
    // delete[] CoMRot_;
    // delete[] lAnkleRot_;
    // delete[] rAnkleRot_;

    // delete[] FKBase_;
    // delete[] FKCoM_;
    // delete[] FKCoMDot_;
    // delete[] FKCoMDotP_;
    // delete[] FKBaseDot_;
    // delete[] realXi_;
    // delete[] realZMP_;
    // delete[] rSoles_;
    // delete[] lSoles_;

    trajSizes_.clear();
    robotControlState_.clear();
    _CoMPos_.clear();
    _CoMRot_.clear();
    _lAnklePos_.clear();
    _lAnkleRot_.clear();
    _rAnklePos_.clear();
    _rAnkleRot_.clear();
    _ZMPD_.clear();
    dataSize_ = 0;
    rSole_ << 0.0, -torso_, 0.0;
    lSole_ << 0.0, torso_, 0.0; 
    isTrajAvailable_ = false;
    Vector3d position(0.0, 0.0, 0.0);
    links_[0]->initPose(position, Matrix3d::Identity(3, 3));
    return true;
}

Robot::~Robot(){
    delete ankleColide_;
    //delete[] links_;
    //delete[] FKBase_;
}