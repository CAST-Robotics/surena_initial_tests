#pragma once

#include <iostream>
#include <math.h>
#include <vector>

#include <ros/ros.h>
#include <ros/package.h>

#include <std_srvs/Empty.h>
#include <sensor_msgs/JointState.h>
#include "sensor_msgs/Imu.h"
#include <std_msgs/Int32MultiArray.h>
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Vector3Stamped.h"

#include "trajectory_planner/command.h"
#include "trajectory_planner/getdata.h"

#include "trajectory_planner/JntAngs.h"
#include "trajectory_planner/Trajectory.h"
#include "trajectory_planner/GeneralTraj.h"

#include <Robot.h>

using namespace std;

class RobotManager{
    public:
        RobotManager(ros::NodeHandle *n);
        bool sendCommand();
        bool setPos(int jointID, int dest);
        bool home(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
        void qcInitial(const sensor_msgs::JointState & msg);
        void ftCallbackLeft(const geometry_msgs::Wrench &msg);
        void bumpCallback(const std_msgs::Int32MultiArray &msg);
        void ftCallbackRight(const geometry_msgs::Wrench &msg);
        void IMUCallback(const sensor_msgs::Imu &msg);
        void AccCallback(const geometry_msgs::Vector3Stamped &msg);
        void GyroCallback(const geometry_msgs::Vector3Stamped &msg);
        bool setFTZero();
        bool setBumpZero();
        bool dummyCallback(trajectory_planner::getdata::Request &req,
                           trajectory_planner::getdata::Response &res);
        bool emptyCommand();
        bool sendCommand(trajectory_planner::command::Request &req,
                         trajectory_planner::command::Response &res);
        void absReader(const sensor_msgs::JointState &msg);
        void incReader(const sensor_msgs::JointState &msg);
        bool absPrinter(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
        bool ankleHome(bool is_left, int roll_dest, int pitch_dest);
        bool walk(trajectory_planner::Trajectory::Request  &req,
                  trajectory_planner::Trajectory::Response &res);
        int sgn(double v);
        double abs2rad(int abs);
        double inc2rad(int inc);
        bool checkAngle(int joint_ID, double angle, double &ang_dif);
        void ankleMechanism(double &theta_inner, double &theta_outer,
                            double teta_p, double teta_r, bool is_left);
        void yawMechanism(double &theta, double alpha1, double R, double B, bool is_left);
        void yawMechanismFK(double &alpha, double theta, double R, double B, bool is_left);

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
        double motorCommandArray_[20];
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