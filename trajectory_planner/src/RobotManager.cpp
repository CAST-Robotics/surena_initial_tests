#include <ros/ros.h>
#include <ros/package.h>
#include <Robot.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32MultiArray.h>
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "sensor_msgs/Imu.h"
#include <vector>
#include "trajectory_planner/command.h"
#include "trajectory_planner/getdata.h"
#include <math.h>
#include <std_srvs/Empty.h>
#include <iostream>
#include "trajectory_planner/JntAngs.h"
#include "trajectory_planner/Trajectory.h"
#include "trajectory_planner/GeneralTraj.h"
#include <ctime>
#include <chrono>
using namespace std::chrono;
using namespace std;



class RobotManager{
  public:
    RobotManager(ros::NodeHandle *n){
        string config_path = ros::package::getPath("trajectory_planner") + "/config/surenav_config.json";
        robot = new Robot(n, config_path);

        motorDataPub_ = n->advertise<std_msgs::Int32MultiArray>("jointdata/qc", 100);
        absSub_ = n->subscribe("/surena/abs_joint_state",100, &RobotManager::absReader, this);
        offsetSub_ = n->subscribe("/surena/inc_joint_state",100, &RobotManager::qcInitial, this);
        incSub_ = n->subscribe("/surena/inc_joint_state",100, &RobotManager::incReader, this);
        jointCommand_ = n->advertiseService("joint_command", &RobotManager::sendCommand, this);
        // trajectoryGenerator_ = n->serviceClient<trajectory_planner::Trajectory>("/traj_gen");
        // generalTrajectory_  = n->serviceClient<trajectory_planner::GeneralTraj>("/general_traj");
        // resetTrajectory_  = n->serviceClient<std_srvs::Empty>("/reset_traj");
        // jointAngles_ = n->serviceClient<trajectory_planner::JntAngs>("/jnt_angs");
        absPrinter_ = n->advertiseService("print_absolute", &RobotManager::absPrinter, this);
        walkService_ = n->advertiseService("walk_service", &RobotManager::walk, this);
        homeService_ = n->advertiseService("home_service", &RobotManager::home, this);
        dummyCommand_ = n->advertiseService("get_data", &RobotManager::dummyCallback, this);
        lFT_ = n->subscribe("/surena/ft_l_state",100, &RobotManager::ftCallbackLeft, this);
        rFT_ = n->subscribe("/surena/ft_r_state",100, &RobotManager::ftCallbackRight, this);
        IMUSub_ = n->subscribe("/surena/imu_state",100, &RobotManager::IMUCallback, this);
        AccSub_ = n->subscribe("/imu/acceleration",100, &RobotManager::AccCallback, this);
        GyroSub_ = n->subscribe("/imu/angular_velocity",100, &RobotManager::GyroCallback, this);
        bumpSub_ = n->subscribe("/surena/bump_sensor_state",100, &RobotManager::bumpCallback, this);
        //bumpSub_ = n->subscribe("/SerialBump",100, &RobotManager::bumpCallback, this);

        b = 0.049;
        c = 0.35;
        r1 = 0.36;
        r0 = 0.047;
        r30_inner << 0.035, 0.034, -0.002;
        r30_outer << 0.035, -0.034, -0.002;

        qcInitialBool_ = true;
        int temp_ratio[12] = {100, 100, 50, 80, 100, 100, 50, 80, 120, 120, 120, 120};
        int temp_home_abs[12] = {122570, 139874, 137321, 8735, 131448, 129963, 140545, 145183, 122054, 18816, 131690, 140432};
        int temp_abs_high[12] = {108426, 119010, 89733, 136440, 71608, 102443, 119697, 82527, 168562, 131334, 191978, 111376};
        int temp_abs_low[12] = {145354, 183778, 194153, 7000, 203256, 160491, 150225, 180000, 61510, 16500, 61482, 172752};
        int temp_abs2inc_dir[12] = {1, 1, -1, -1, -1, 1, 1, 1, -1, 1, 1, 1};
        int temp_abs_dir[12] = {-1, -1, -1, 1, -1, -1, -1, -1, 1, 1, 1, -1};
        int temp_motor_dir[12] = {1, 1, 1, -1, -1, 1, 1, 1, -1, 1, 1, -1};
        int temp_bump_order[8] = {3, 0, 1, 2, 6, 5, 4, 7};
        int temp_initial_bump[4] = {0, 0, 0, 0};

        collision_ = false;

        for (int i=0; i<12; i++){
            harmonicRatio_[i] = temp_ratio[i];
            homeAbs_[i] = temp_home_abs[i];
            absHigh_[i] = temp_abs_high[i];
            absLow_[i] = temp_abs_low[i];
            abs2incDir_[i] = temp_abs2inc_dir[i];
            absDir_[i] = temp_abs_dir[i];
            motorDir_[i] = temp_motor_dir[i];
            commandConfig_[0][i] = 0.0;
            commandConfig_[1][i] = 0.0;
            commandConfig_[2][i] = 0.0;
            if (i<8){
                bumpOrder_[i] = temp_bump_order[i];
            }
            if (i<4){
                rBumpOffset_[i] = temp_initial_bump[i];
                lBumpOffset_[i] = temp_initial_bump[i];
            }
        }
        
        leftFTFile_.open("/home/surena/SurenaV/log/left_ft.csv");
        rightFTFile_.open("/home/surena/SurenaV/log/right_ft.csv");
        if(!leftFTFile_.is_open())
            ROS_DEBUG("left FT log file not open!");
        if(!rightFTFile_.is_open())
            ROS_DEBUG("right FT log file not open!");

        FTOffsetPeriod_ = 60;
        lFTOffset_ = Vector3d::Zero();
        rFTOffset_ = Vector3d::Zero();

        double temp[3] = {0.0, 0.0, 0.0};
        for (int i = 0; i<3; i++){
            currentLFT_[i] = temp[i];
            currentRFT_[i] = temp[i];
        }

    }   

    bool setPos(int jointID, int dest){
        // this function is used for changing the position of all joint except ankle joints.
        // the value of dest is on the basis of absolute incs.
        ros::Rate rate_(200);
        int temp_roll = jointID;
        // if(jointID == 1){
        //     temp_roll = 0;   // For Fixing Right Hip Roll Problem
        // }
        while(abs(abs(absData_[temp_roll]) - dest) > 100){
            if (abs(absData_[temp_roll]) < 262144){
                    if (abs(absData_[temp_roll]) > dest){
                        motorCommand_.data[temp_roll] -= abs2incDir_[jointID]*(4096*4*0.01);
                        //cout << "!!!!!!!" << abs(absData_[temp_roll]) - dest << endl;
                    }
                    else{
                        motorCommand_.data[temp_roll] += abs2incDir_[jointID]*(4096*4*0.01);
                        //cout << "@@@@@@" << abs(absData_[temp_roll]) - dest << endl;

                }
            }
            motorDataPub_.publish(motorCommand_);
            ros::spinOnce();
            rate_.sleep();
        } 
        return true;
    }

    bool home(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
        
        //ankleHome(false);
        //setPos(6, homeAbs_[6]);
        //setPos(0, homeAbs_[0]);
        setPos(1, homeAbs_[1] - 20000);
        setPos(7, homeAbs_[7] + 20000);
        ankleHome(false, homeAbs_[5], homeAbs_[4]);
        setPos(3, homeAbs_[3]);
        setPos(2, homeAbs_[2]);
        setPos(1, homeAbs_[1]);
        ankleHome(true, homeAbs_[11], homeAbs_[10]);
        setPos(9, homeAbs_[9]);
        setPos(8, homeAbs_[8]);
        setPos(7, homeAbs_[7]);
        qcInitialBool_ = true;
        setFTZero();
        setBumpZero();
        ros::spinOnce();
        for (int i=0; i<12; i++){
            commandConfig_[0][i] = 0.0;
            commandConfig_[1][i] = 0.0;
            commandConfig_[2][i] = 0.0;
        }
        return true;
    }

    void qcInitial(const sensor_msgs::JointState & msg){

        if (qcInitialBool_){

            for (int i = 0; i <= 31; ++i) {
                homeOffset_[i]=int(msg.position[i+1]);
                motorCommand_.data.push_back(homeOffset_[i]);
            }
            qcInitialBool_=false;
            //ROS_INFO("Offset=%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t\n",
            //        homeOffset_[0],homeOffset_[1],homeOffset_[2],homeOffset_[3],homeOffset_[4],
            //        homeOffset_[5],homeOffset_[6],homeOffset_[7],homeOffset_[8],homeOffset_[9],
            //        homeOffset_[10],homeOffset_[11],homeOffset_[12],homeOffset_[13],homeOffset_[14],homeOffset_[15],homeOffset_[20],homeOffset_[21],homeOffset_[22],homeOffset_[23]);
        
        }
        
    }


    void ftCallbackLeft(const geometry_msgs::Wrench &msg){
        currentLFT_[0] = msg.force.x - lFTOffset_(0);
        currentLFT_[1] = msg.force.y - lFTOffset_(1);
        currentLFT_[2] = msg.force.z - lFTOffset_(2);
        leftFTFile_ << currentLFT_[0] << "," << currentLFT_[1] << "," << currentLFT_[2] << endl;
        if(recentLFT_.size() < FTOffsetPeriod_){
            recentLFT_.push_back(Vector3d(msg.force.x, msg.force.y, msg.force.z));
        }
        else{
            recentLFT_.erase (recentLFT_.begin());
            recentLFT_.push_back(Vector3d(msg.force.x, msg.force.y, msg.force.z));
        }
    }

    void bumpCallback(const std_msgs::Int32MultiArray &msg){
        for (int i = 0; i < 8; ++i){
            if (i<4){
                currentRBump_[bumpOrder_[i]] = msg.data[i] - rBumpOffset_[bumpOrder_[i]];
                realRBump_[bumpOrder_[i]] = msg.data[i];
            }
            else{
                currentLBump_[bumpOrder_[i] - 4] = msg.data[i] - lBumpOffset_[bumpOrder_[i] - 4]; 
                realLBump_[bumpOrder_[i] - 4] = msg.data[i];
            }
        }
        /*
        cout<<currentLBump_[0]<< "," <<currentLBump_[1]<< "," <<currentLBump_[2]<< "," <<currentLBump_[3]<<endl;
        cout<<currentRBump_[0]<< "," <<currentRBump_[1]<< "," <<currentRBump_[2]<< "," <<currentRBump_[3]<<endl; 
        */  
    }

    void ftCallbackRight(const geometry_msgs::Wrench &msg){
        currentRFT_[0] = msg.force.x - rFTOffset_(0);
        currentRFT_[1] = msg.force.y - rFTOffset_(1);
        currentRFT_[2] = msg.force.z - rFTOffset_(2);
        rightFTFile_ << currentRFT_[0] << "," << currentRFT_[1] << "," << currentRFT_[2] << endl;
        if(recentRFT_.size() < FTOffsetPeriod_){
            recentRFT_.push_back(Vector3d(msg.force.x, msg.force.y, msg.force.z));
        }
        else{
            recentRFT_.erase (recentRFT_.begin());
            recentRFT_.push_back(Vector3d(msg.force.x, msg.force.y, msg.force.z));
        }
    }
    void IMUCallback(const sensor_msgs::Imu &msg){
        // baseAcc_[0] = msg.linear_acceleration.x;
        // baseAcc_[1] = msg.linear_acceleration.y;
        // baseAcc_[2] = msg.linear_acceleration.z;

        baseOrient_[0] = msg.orientation.x;
        baseOrient_[1] = msg.orientation.y;
        baseOrient_[2] = msg.orientation.z;
        
        // baseAngVel_[0] = msg.angular_velocity.x;
        // baseAngVel_[1] = msg.angular_velocity.y;
        // baseAngVel_[2] = msg.angular_velocity.z;
    }

    void AccCallback(const geometry_msgs::Vector3Stamped &msg){
        baseAcc_[0] = msg.vector.x;
        baseAcc_[1] = msg.vector.y;
        baseAcc_[2] = msg.vector.z;
    }

    void GyroCallback(const geometry_msgs::Vector3Stamped &msg){
        baseAngVel_[0] = msg.vector.x;
        baseAngVel_[1] = msg.vector.y;
        baseAngVel_[2] = msg.vector.z;
    }

    bool setFTZero(){

        for (int i = 0; i < recentRFT_.size(); i++){
            lFTOffset_ += recentLFT_[i];
            rFTOffset_ += recentRFT_[i];
        }
        lFTOffset_ = lFTOffset_ / recentLFT_.size();
        rFTOffset_ = rFTOffset_ / recentRFT_.size();
        //cout << "left ft offset: " << lFTOffset_ << "   " << rFTOffset_ << endl;
        return true;

    }

    bool setBumpZero(){
        for (int i = 0; i<4; i++){
            rBumpOffset_[i] = realRBump_[i];
            lBumpOffset_[i] = realLBump_[i];
        }
    }

    bool dummyCallback(trajectory_planner::getdata::Request &req, trajectory_planner::getdata::Response &res){
        for(int i = 0; i < req.time * 100; i ++){
            emptyCommand();
        }
        return true;
    }

    bool emptyCommand(){
        ros::spinOnce();
        ros::Rate rate_(200);
        for(int i=0; i<32; i++)
            motorCommand_.data[i] = incData_[i];

        motorCommand_.data[0] += 1;
        motorDataPub_.publish(motorCommand_);
        rate_.sleep();
        motorCommand_.data[0] -= 1;
        motorDataPub_.publish(motorCommand_);
        rate_.sleep();
        return true;
    }

    bool sendCommand(trajectory_planner::command::Request &req, trajectory_planner::command::Response &res){

        ros::Rate rate_(200);
        this->emptyCommand();
        if(req.motor_id == 4){
            
            double theta_inner;
            double theta_outer;
            double cur_pitch = absDir_[4] * (absData_[4] - homeAbs_[4]) * 2 * M_PI / pow(2, 19);
            double cur_roll = absDir_[5] * (absData_[5] - homeAbs_[5]) * 2 * M_PI / pow(2, 19);

            double desired_pitch = cur_pitch + req.angle;
            this->ankleMechanism(theta_inner, theta_outer, desired_pitch, cur_roll, false);
            double inner_inc = motorDir_[5] * theta_inner * 4096 * 4 * 100 * 1.5 / 2 / M_PI + homeOffset_[5];
            double outer_inc = motorDir_[4] * theta_outer * 4096 * 4 * 100 * 1.5 / 2 / M_PI + homeOffset_[4];

            double inner_delta_inc = inner_inc - incData_[5];
            double outer_delta_inc = outer_inc - incData_[4];
            
            motorCommand_.data[4] = incData_[4];
            motorCommand_.data[5] = incData_[5];

            for (int i = 0; i < 100; i++){
                motorCommand_.data[4] += outer_delta_inc / 100;
                motorCommand_.data[5] +=  inner_delta_inc / 100;
                motorDataPub_.publish(motorCommand_);
                ros::spinOnce();
                rate_.sleep();
            }
        } else if(req.motor_id == 5){

            double theta_inner;
            double theta_outer;
            double cur_pitch = absDir_[4] * (absData_[4] - homeAbs_[4]) * 2 * M_PI / pow(2, 19);
            double cur_roll = absDir_[5] * (absData_[5] - homeAbs_[5]) * 2 * M_PI / pow(2, 19);

            double desired_roll = cur_roll + req.angle;
            this->ankleMechanism(theta_inner, theta_outer, cur_pitch, desired_roll, false);
            double inner_inc = motorDir_[5] * theta_inner * 4096 * 4 * 100 * 1.5 / 2 / M_PI + homeOffset_[5];
            double outer_inc = motorDir_[4] * theta_outer * 4096 * 4 * 100 * 1.5 / 2 / M_PI + homeOffset_[4];

            double inner_delta_inc = inner_inc - incData_[5];
            double outer_delta_inc = outer_inc - incData_[4];
            
            motorCommand_.data[4] = incData_[4];
            motorCommand_.data[5] = incData_[5];

            for (int i = 0; i < 100; i++){
                motorCommand_.data[4] += outer_delta_inc / 100;
                motorCommand_.data[5] +=  inner_delta_inc / 100;
                motorDataPub_.publish(motorCommand_);
                ros::spinOnce();
                rate_.sleep();
            }
        } else if(req.motor_id == 10){

            double theta_inner;
            double theta_outer;
            double cur_pitch = absDir_[10] * (absData_[10] - homeAbs_[10]) * 2 * M_PI / pow(2, 19);
            double cur_roll = absDir_[11] * (absData_[11] - homeAbs_[11]) * 2 * M_PI / pow(2, 19);

            double desired_pitch = cur_pitch + req.angle;
            this->ankleMechanism(theta_inner, theta_outer, desired_pitch, cur_roll, true);
            double inner_inc = motorDir_[11] * theta_inner * 4096 * 4 * 100 * 1.5 / 2 / M_PI + homeOffset_[11];
            double outer_inc = motorDir_[10] * theta_outer * 4096 * 4 * 100 * 1.5 / 2 / M_PI + homeOffset_[10];

            double inner_delta_inc = inner_inc - incData_[11];
            double outer_delta_inc = outer_inc - incData_[10];
            
            motorCommand_.data[10] = incData_[10];
            motorCommand_.data[11] = incData_[11];

            for (int i = 0; i < 100; i++){
                motorCommand_.data[10] += outer_delta_inc / 100;
                motorCommand_.data[11] +=  inner_delta_inc / 100;
                motorDataPub_.publish(motorCommand_);
                ros::spinOnce();
                rate_.sleep();
            }
        } else if(req.motor_id == 11){

            double theta_inner;
            double theta_outer;
            double cur_pitch = absDir_[10] * (absData_[10] - homeAbs_[10]) * 2 * M_PI / pow(2, 19);
            double cur_roll = absDir_[11] * (absData_[11] - homeAbs_[11]) * 2 * M_PI / pow(2, 19);

            double desired_roll = cur_roll + req.angle;
            this->ankleMechanism(theta_inner, theta_outer, cur_pitch, desired_roll, true);
            double inner_inc = motorDir_[11] * theta_inner * 4096 * 4 * 100 * 1.5 / 2 / M_PI + homeOffset_[11];
            double outer_inc = motorDir_[10] * theta_outer * 4096 * 4 * 100 * 1.5 / 2 / M_PI + homeOffset_[10];

            double inner_delta_inc = inner_inc - incData_[11];
            double outer_delta_inc = outer_inc - incData_[10];
            
            motorCommand_.data[10] = incData_[10];
            motorCommand_.data[11] = incData_[11];

            for (int i = 0; i < 100; i++){
                motorCommand_.data[10] += outer_delta_inc / 100;
                motorCommand_.data[11] +=  inner_delta_inc / 100;
                motorDataPub_.publish(motorCommand_);
                ros::spinOnce();
                rate_.sleep();
            }
        } else if(req.motor_id == 0){
            double theta;
            this->yawMechanism(theta, req.angle, 0.03435, 0.088, false);
            double yaw_inc =  motorDir_[0] * theta * 4096 * 4 * 100 / 2 / M_PI;
            cout << theta << ": theta" << endl;
            for (int i = 0; i < 100; i++){
                motorCommand_.data[0] += yaw_inc / 100;
                motorDataPub_.publish(motorCommand_);
                ros::spinOnce();
                rate_.sleep();
            }
        } else if(req.motor_id == 6){
            double theta;
            this->yawMechanism(theta, req.angle, 0.03435, 0.088, true);
            double yaw_inc = motorDir_[6] * theta * 4096 * 4 * 100 / 2 / M_PI;
            for (int i = 0; i < 100; i++){
                motorCommand_.data[6] += yaw_inc / 100;
                motorDataPub_.publish(motorCommand_);
                ros::spinOnce();
                rate_.sleep();
            }           
        } else{
            double inc = motorDir_[req.motor_id] * req.angle * 4096 * 4 * 160 / 2 / M_PI;
            for (int i = 0; i < int(abs(inc)) / 100; i++){
                motorCommand_.data[req.motor_id] += sgn(inc) * 100;
                motorDataPub_.publish(motorCommand_);
                ros::spinOnce();
                rate_.sleep();
            }           
        }
    }

    void absReader(const sensor_msgs::JointState &msg){
        for(int i=0; i<32; i++){
            absData_[i] = msg.position[i+1];

            if(i == 10)
                absData_[10] = msg.position[12];
            else if(i == 11)
                absData_[11] = msg.position[11];

            //Lower body collision detection
            if(i < 12 && abs(absData_[i]) > 262144){
                //cout << "Invalid Abs Data, id: " << i << endl;
                continue;
            }
            if(i < 12 && (absData_[i] >= ((absHigh_[i] - homeAbs_[i])*absDir_[i] - 100) || absData_[i] <= ((absLow_[i] - homeAbs_[i])*absDir_[i] + 100))){
                collision_ = true;
                //cout << "Error: Collision detected!!, id: " << i << endl;
            }
        }
    }

    void incReader(const sensor_msgs::JointState &msg){
        for(int i=0; i<32; i++){
            incData_[i] = msg.position[i+1];
        }
    }

    bool absPrinter(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
        for(int i=0; i<32; i++){
            cout<<" Sensor ID: "<< i;
            cout<<" value: "<< absData_[i]<<endl;
        }
    }
    

    bool ankleHome(bool is_left, int roll_dest, int pitch_dest){

        int inner = 5;//roll
        int outer = 4;//pitch
        int ankle_dir = 1;
        if (is_left){
            ankle_dir = -1;
            inner = 11;
            outer = 10;
        }

        ros::Rate rate_(200);
        while(abs(abs(absData_[inner]) - roll_dest) > 100){

            if (abs(absData_[inner]) < 262144){

                if (abs(absData_[inner]) > roll_dest){
                    motorCommand_.data[outer] -= (4096*4*0.01); //
                    motorCommand_.data[inner] -= (4096*4*0.01);//
                    //cout << "roll-if:" << abs(absData_[inner]) - roll_dest << endl;
                }
                else{
                    motorCommand_.data[outer] += (4096*4*0.01);//
                    motorCommand_.data[inner] += (4096*4*0.01);//
                    //cout << "roll-else:" << abs(absData_[inner]) - roll_dest << endl;

                }
            }
            motorDataPub_.publish(motorCommand_);
            ros::spinOnce();
            rate_.sleep();
        }
        while(abs(abs(absData_[outer]) - pitch_dest) > 100){

            if (abs(absData_[outer]) < 262144){

                if (abs(absData_[outer]) > pitch_dest){
                    motorCommand_.data[outer] -= (4096*4*0.01);//+
                    motorCommand_.data[inner] += (4096*4*0.01);//-
                    //cout << "pitch-if:" << abs(absData_[outer]) - pitch_dest << endl;
                }
                else{
                    motorCommand_.data[outer] += (4096*4*0.01);
                    motorCommand_.data[inner] -= (4096*4*0.01);
                    //cout << "pitch-else:" << abs(absData_[outer]) - pitch_dest << endl;
                } 
            }
            motorDataPub_.publish(motorCommand_);
            ros::spinOnce();
            rate_.sleep();
            
        }
    }

    bool walk(trajectory_planner::Trajectory::Request  &req,
            trajectory_planner::Trajectory::Response &res){
        this->emptyCommand();
        int rate = 200;
        ros::Rate rate_(rate);

        
        double init_com_pos[3] = {0, 0, 0.71};
        double init_com_orient[3] = {0, 0, 0};
        double final_com_pos[3] = {0, 0, req.COM_height};
        double final_com_orient[3] = {0, 0, 0};

        double init_lankle_pos[3] = {0, 0.0975, 0};
        double init_lankle_orient[3] = {0, 0, 0};
        double final_lankle_pos[3] = {0, 0.0975, 0};
        double final_lankle_orient[3] = {0, 0, 0};

        double init_rankle_pos[3] = {0, -0.0975, 0};
        double init_rankle_orient[3] = {0, 0, 0};
        double final_rankle_pos[3] = {0, -0.0975, 0};
        double final_rankle_orient[3] = {0, 0, 0};

        robot->generalTrajGen(req.dt, 2, init_com_pos, final_com_pos, init_com_orient, final_com_orient,
                              init_lankle_pos, final_lankle_pos, init_lankle_orient, final_lankle_orient,
                              init_rankle_pos, final_rankle_pos, init_rankle_orient, final_rankle_orient);

        // general_traj.request.time = req.t_step;
        // general_traj.request.init_com_pos = {0, 0, req.COM_height};
        // general_traj.request.final_com_pos = {0, 0, req.COM_height};
        // generalTrajectory_.call(general_traj);

        // general_traj.request.init_rankle_pos = {0, -0.0975, 0.05};
        // general_traj.request.final_rankle_pos = {0, -0.0975, 0.0};
        // generalTrajectory_.call(general_traj);

        // general_traj.request.init_rankle_pos = {0, -0.0975, 0.0};
        // general_traj.request.final_rankle_pos = {0, -0.0975, 0.05};
        // generalTrajectory_.call(general_traj);

        robot->trajGen(req.step_count, req.t_step, req.alpha, req.t_double_support, req.COM_height,
                       req.step_length, req.step_width, req.dt, req.theta, req.ankle_height, 
                       req.step_height, false, 1, 1);
        //if(traj_srv.response.result){
           
        init_com_pos[2] = req.COM_height;
        final_com_pos[2] = 0.71;
        robot->generalTrajGen(req.dt, 2, init_com_pos, final_com_pos, init_com_orient, final_com_orient,
                              init_lankle_pos, final_lankle_pos, init_lankle_orient, final_lankle_orient,
                              init_rankle_pos, final_rankle_pos, init_rankle_orient, final_rankle_orient);
        if(true){
            auto start = high_resolution_clock::now();
            auto stop = high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(stop - start);
            
            // int final_iter = rate * (req.t_step * (req.step_count + 2) + 4);
            // int final_iter = rate * (req.t_step + 4);
            int final_iter = robot->getTrajSize();
            
            double jnt_command[12];
            int status;
            int i = 0;
            
            while(i < final_iter){
                double config[12];
                double jnt_vel[12];
                double left_ft[3] = {-currentLFT_[0], -currentLFT_[2], -currentLFT_[1]};
                double right_ft[3] = {-currentRFT_[0], -currentRFT_[2], -currentRFT_[1]};
                int right_bump[4] = {currentRBump_[0], currentRBump_[1], currentRBump_[2], currentRBump_[3]};
                int left_bump[4] = {currentLBump_[0], currentLBump_[1], currentLBump_[2], currentLBump_[3]};
                double accelerometer[3] = {baseAcc_[0], baseAcc_[1], baseAcc_[2]};
                double gyro[3] = {baseAngVel_[0], baseAngVel_[1], baseAngVel_[2]};
                
                for (int i=0; i<12; i++){
                    config[i] = commandConfig_[2][i];
                    jnt_vel[i] = (commandConfig_[0][i] - 4 * commandConfig_[1][i] + 3 * commandConfig_[2][i])/(2 * req.dt);
                }

                robot->getJointAngs(i, config, jnt_vel, right_ft, left_ft, right_bump,
                                    left_bump, gyro, accelerometer, jnt_command, status);
                if(status != 0){
                    cout << "Node was shut down due to Ankle Collision!" << endl;
                    return false;
                }
                // cout << jnt_command[0] << ',' << jnt_command[1] << ','<< jnt_command[2] << ','
                // << jnt_command[3] << ','<< jnt_command[4] << ','<< jnt_command[5] << ','
                // << jnt_command[6] << ','<< jnt_command[7] << ','<< jnt_command[8] << ','
                // << jnt_command[9] << ','<< jnt_command[10] << ','<< jnt_command[11] << endl;
                for(int j=0; j < 12; j++){
                    double dif = 0;
                    if(this->checkAngle(j, jnt_command[j], dif)){
                        switch (j)
                        {
                            double theta;
                            double alpha;
                            double theta_inner;
                            double theta_outer;
                            double desired_pitch;
                            double desired_roll;
                            double inner_inc;
                            double outer_inc;
                            
                        case 0:
                            this->yawMechanism(theta, jnt_command[j], 0.03435, 0.088, false);
                            motorCommand_.data[j] =  motorDir_[j] * theta * 4096 * 4 * 100 / 2 / M_PI + homeOffset_[j];
                            //motorCommand_.data[j] =  motorDir_[1] * jnt_command[1] * 4096 * 4 * 160 / 2 / M_PI + homeOffset_[j];
                            
                            yawMechanismFK(alpha,inc2rad(incData_[0] - homeOffset_[0]) / 100, 0.03435, 0.088, false);
                            commandConfig_[2][j] = alpha;
                            break;
                        case 6:
                            this->yawMechanism(theta, jnt_command[j], 0.03435, 0.088, true);
                            motorCommand_.data[j] = motorDir_[j] * theta * 4096 * 4 * 100 / 2 / M_PI + homeOffset_[j];
                            yawMechanismFK(alpha,inc2rad(motorDir_[j] * (incData_[j] - homeOffset_[j])) / 100, 0.03435, 0.088, true);
                            commandConfig_[2][j] =  alpha;
                            break;
                        case 4:
                            desired_roll = jnt_command[5];
                            desired_pitch = jnt_command[4];
                            this->ankleMechanism(theta_inner, theta_outer, desired_pitch, desired_roll, false);
                            inner_inc = motorDir_[5] * theta_inner * 4096 * 4 * 100 * 1.5 / 2 / M_PI + homeOffset_[5];
                            outer_inc = motorDir_[4] * theta_outer * 4096 * 4 * 100 * 1.5 / 2 / M_PI + homeOffset_[4];                            
                            motorCommand_.data[4] = outer_inc;
                            motorCommand_.data[5] = inner_inc;
                            //commandConfig_[2][j] = jnt_command[4];
                            commandConfig_[2][j] = absDir_[j] * abs2rad(absData_[j] - homeAbs_[j]);
                            break;
                        case 5:
                            desired_roll = jnt_command[5];
                            desired_pitch = jnt_command[4];
                            this->ankleMechanism(theta_inner, theta_outer, desired_pitch, desired_roll, false);
                            inner_inc = motorDir_[5] * theta_inner * 4096 * 4 * 100 * 1.5 / 2 / M_PI + homeOffset_[5];
                            outer_inc = motorDir_[4] * theta_outer * 4096 * 4 * 100 * 1.5 / 2 / M_PI + homeOffset_[4];                            
                            motorCommand_.data[4] = outer_inc;
                            motorCommand_.data[5] = inner_inc;
                            //commandConfig_[2][j] = jnt_command[j];
                            commandConfig_[2][j] = absDir_[j] * abs2rad(absData_[j] - homeAbs_[j]);
                            break;
                        case 10:
                            desired_roll = jnt_command[11];
                            desired_pitch = jnt_command[10];
                            this->ankleMechanism(theta_inner, theta_outer, desired_pitch, desired_roll, true);
                            inner_inc = motorDir_[11] * theta_inner * 4096 * 4 * 100 * 1.5 / 2 / M_PI + homeOffset_[11];
                            outer_inc = motorDir_[10] * theta_outer * 4096 * 4 * 100 * 1.5 / 2 / M_PI + homeOffset_[10];                            
                            motorCommand_.data[10] = outer_inc;
                            motorCommand_.data[11] = inner_inc;
                            //commandConfig_[2][j] = jnt_command[j];
                            commandConfig_[2][j] = absDir_[j] * abs2rad(absData_[j] - homeAbs_[j]);
                            break;
                        case 11:
                            desired_roll = jnt_command[11];
                            desired_pitch = jnt_command[10];
                            this->ankleMechanism(theta_inner, theta_outer, desired_pitch, desired_roll, true);
                            inner_inc = motorDir_[11] * theta_inner * 4096 * 4 * 100 * 1.5 / 2 / M_PI + homeOffset_[11];
                            outer_inc = motorDir_[10] * theta_outer * 4096 * 4 * 100 * 1.5 / 2 / M_PI + homeOffset_[10];                            
                            motorCommand_.data[10] = outer_inc;
                            motorCommand_.data[11] = inner_inc;
                            //commandConfig_[2][j] = jnt_command[j];
                            commandConfig_[2][j] = absDir_[j] * abs2rad(absData_[j] - homeAbs_[j]);
                            break;
                        default:
                            motorCommand_.data[j] = motorDir_[j] * jnt_command[j] * 4096 * 4 * 160 / 2 / M_PI + homeOffset_[j];
                            commandConfig_[2][j] = motorDir_[j] * inc2rad(incData_[j] - homeOffset_[j]) / 160;
                            break;
                        }
                        
                        // if(j == 0)
                        //     commandConfig_[j] = 0;
                        // else if(j == 1)
                        //     commandConfig_[j] = absDir_[j] * abs2rad(absData_[0] - homeAbs_[1]);
                        // else if (j == 10)
                        //     continue;
                        // else
                        if(i == 0){
                            commandConfig_[0][j] = 0;
                            commandConfig_[1][j] = 0;
                        }else if(i == 1){
                            commandConfig_[0][j] = 0;
                            commandConfig_[1][j] = commandConfig_[2][j];
                        }else{
                            commandConfig_[0][j] = commandConfig_[1][j];
                            commandConfig_[1][j] = commandConfig_[2][j];
                        }

                        //commandConfig_[2][j] = absDir_[j] * abs2rad(absData_[j] - homeAbs_[j]);
                        


                        
                    }else{
                        cout<< "joint " << j << " out of workspace in iteration "<< i << ", angle difference: " << dif << endl ;
                        return false;
                    }
                }    
                stop = high_resolution_clock::now();
                duration = duration_cast<microseconds>(stop - start);
                //cout << duration.count()/1000000.0 << endl;
                motorDataPub_.publish(motorCommand_);
                ros::spinOnce();
                rate_.sleep();
                start = high_resolution_clock::now();
                i++;
            }
            // std_srvs::Empty reset_srv;
            robot->resetTraj();
            // resetTrajectory_.call(reset_srv);
            res.result = true;
            return true;
        }
        else{
            ROS_INFO("Joint angles were not sent\n");
            res.result = false;
            return false;
        }
    }

    int sgn(double v){
        return ((v < 0) ? -1 : (v > 0));
    }

    double abs2rad(int abs){
        double angle = abs/pow(2, 19)*2*M_PI;
        return angle;
    }

    double inc2rad(int inc){
        double angle = inc/pow(2, 14)*2*M_PI;
        return angle;
    }    

    bool checkAngle(int joint_ID, double angle, double &ang_dif){
        if (angle >= abs2rad(absHigh_[joint_ID] - homeAbs_[joint_ID])*absDir_[joint_ID]){
            ang_dif = angle;

            return false;
        }else if(angle <= abs2rad(absLow_[joint_ID] - homeAbs_[joint_ID])*absDir_[joint_ID]){
            ang_dif = angle;
            return false;
        }else
            return true; 
    }

    void ankleMechanism(double &theta_inner, double &theta_outer, double teta_p, double teta_r, bool is_left){
        
        Matrix3d pitch_rot;
        pitch_rot<<cos(teta_p), 0, sin(teta_p),
                    0, 1, 0,
                    -sin(teta_p), 0, cos(teta_p);
        Matrix3d roll_rot;
        roll_rot<<1, 0, 0,
                    0, cos(teta_r), -sin(teta_r),
                    0, sin(teta_r), cos(teta_r); 
        Matrix3d rot_mat = pitch_rot * roll_rot;
        Vector3d r3_inner = rot_mat * r30_inner;
        Vector3d r3_outer = rot_mat * r30_outer;
        Vector3d r4_inner(0, -b, -c);
        Vector3d r4_outer(0, b, -c);
        Vector3d r2_inner = r3_inner + r4_inner;
        Vector3d r2_outer = r3_outer + r4_outer;
        Vector3d norm1(r2_inner(0),0,r2_inner(2));
        Vector3d norm2(r2_outer(0),0,r2_outer(2));


        if(is_left){
            theta_inner = acos((pow(r2_outer.norm(),2) + pow(r0,2) -pow(r1,2)) / ((norm2.norm()*2*r0))) + atan2(r2_outer(0) , -r2_outer(2));
            theta_outer = acos((pow(r2_inner.norm(),2) + pow(r0,2) -pow(r1,2)) / ((norm1.norm()*2*r0))) + atan2(r2_inner(0) , -r2_inner(2));
        }else{
            theta_inner = acos((pow(r2_inner.norm(),2) + pow(r0,2) -pow(r1,2)) / ((norm1.norm()*2*r0))) + atan2(r2_inner(0) , -r2_inner(2));
            theta_outer = acos((pow(r2_outer.norm(),2) + pow(r0,2) -pow(r1,2)) / ((norm2.norm()*2*r0))) + atan2(r2_outer(0) , -r2_outer(2));
        }
        theta_inner = -(theta_inner - M_PI / 2 - 0.160405462422601);
        theta_outer = -(theta_outer - M_PI / 2 - 0.160405462422601);
    }

    void yawMechanism(double &theta, double alpha1, double R, double B, bool is_left){
        double theta_home, alpha_home;
        if(is_left){
            alpha_home = 0.0529;
            theta_home = 0.1868;  
        }else{
            alpha_home = 0.0227;
            theta_home = 0.0809;
        }

        double alpha = alpha_home + alpha1;
        theta = atan2(tan(alpha) * (-(B * pow(tan(alpha), 0.2e1) - sqrt(-B * B * pow(tan(alpha), 0.2e1) + pow(tan(alpha), 0.2e1) * R * R + R * R)) / (pow(tan(alpha), 0.2e1) + 0.1e1) + B) / R, -(B * pow(tan(alpha), 0.2e1) - sqrt(-B * B * pow(tan(alpha), 0.2e1) + pow(tan(alpha), 0.2e1) * R * R + R * R)) / (pow(tan(alpha), 0.2e1) + 0.1e1) / R);
        theta = theta_home - theta;
    }

    void yawMechanismFK(double &alpha, double theta, double R, double B, bool is_left){
        double theta_home, alpha_home;
        if(is_left){
            alpha_home = 0.0529;
            theta_home = 0.1868;  
        }else{
            alpha_home = 0.0227;
            theta_home = 0.0809;
        }
        theta -= theta_home;
        alpha = atan(R * sin(theta) / (B + R * cos(theta)));
        alpha += alpha_home;
    }

private:
    Robot* robot;

    ros::Publisher motorDataPub_;
    ros::Subscriber incSub_;
    ros::Subscriber offsetSub_;
    ros::Subscriber absSub_;
    ros::Subscriber lFT_;
    ros::Subscriber rFT_;
    ros::Subscriber IMUSub_;
    ros::Subscriber AccSub_;
    ros::Subscriber GyroSub_;
    ros::Subscriber bumpSub_;
    ros::ServiceServer jointCommand_;
    ros::ServiceServer absPrinter_;
    // ros::ServiceClient trajectoryGenerator_;
    // ros::ServiceClient generalTrajectory_ ;
    // ros::ServiceClient resetTrajectory_;
    // ros::ServiceClient jointAngles_;
    ros::ServiceServer walkService_;
    ros::ServiceServer homeService_;
    ros::ServiceServer dummyCommand_;
    bool qcInitialBool_;
    int homeOffset_[32];
    std_msgs::Int32MultiArray motorCommand_;
    int harmonicRatio_[12];
    float absData_[32];
    int incData_[32];
    int homeAbs_[12];
    int absHigh_[12];
    int absLow_[12];
    int abs2incDir_[12];
    int absDir_[12];
    int motorDir_[12];
    bool collision_;
    int bumpOrder_[8];

    // Ankle Mechanism Parameters

    float b;
    float c;
    float r1;
    float r0;
    Vector3d r30_inner;
    Vector3d r30_outer;
    MatrixXd r4_inner;
    MatrixXd r4_outer;
    
    ofstream leftFTFile_;
    ofstream rightFTFile_;

    Vector3d rFTOffset_;
    Vector3d lFTOffset_;
    int rBumpOffset_[4];
    int lBumpOffset_[4];
    vector<Vector3d> recentLFT_;
    vector<Vector3d> recentRFT_;

    double currentLFT_[3];
    double currentRFT_[3];
    int currentLBump_[4];
    int currentRBump_[4];
    int realRBump_[4];
    int realLBump_[4];
    double baseAcc_[3];
    double baseOrient_[3];
    double baseAngVel_[3];
    double commandConfig_[3][12];

    int FTOffsetPeriod_;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "eth_subscriber");
    ros::NodeHandle n;
    RobotManager wt(&n);
    ros::spin();
	return 0;
}