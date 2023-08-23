#include "../Eigen/Dense"
#include "../Eigen/Geometry"
#include "../Eigen/QuadProg.h"
#include "../Eigen/testQPsolvers.hpp"
#include "../Eigen/eiquadprog.hpp"
#include "../Eigen/Core"
#include "../Eigen/Cholesky"
#include "../Eigen/LU"
#include <iostream>
#include <math.h>
#include <vector>
#include "fstream"
#include <string>
#include <geometry_msgs/PoseArray.h>
#include<std_msgs/Int32MultiArray.h>
#include "ros/ros.h"
using namespace std;
using namespace Eigen;


double PtoR = 0.0825;
double YtoP = 0.06025;
double h_pitch = 0;
double h_roll = 0;
double h_yaw = 0;
VectorXd camera(3);
double Kp = 0.8;
double Ky = 0.8;
double theta_pitch; 
double sai_roll;
double phi_yaw;
int n = 0;
int N = 4000;
double x = 1;
double y = 0.0005;
double z = 0.0005;
MatrixXd target_to_camera(1,3);
MatrixXd TEMP(N,3);

void object_detect(const geometry_msgs::PoseArray &msg){
            x = msg.poses[0].position.x;
            y = msg.poses[0].position.y;
            z = msg.poses[0].position.z;
            }

MatrixXd ObjToNeck(VectorXd camera, double h_pitch, double h_roll, double h_yaw, double PtoR, double YtoP) {
    MatrixXd T0(4, 4);
    T0 << cos(M_PI / 9), 0, sin(M_PI / 9), 0,
        0, 1, 0, 0,
        -sin(M_PI / 9), 0, cos(M_PI / 9), 0,
        0, 0, 0, 1;

    MatrixXd T1(4, 4);
    T1 << 1, 0, 0, camera(0),
        0, 1, 0, camera(1),
        0, 0, 1, camera(2),
        0, 0, 0, 1;

    MatrixXd T2(4, 4);
    T2 << 1, 0, 0, 0,
        0, cos(h_roll), -sin(h_roll), 0,
        0, sin(h_roll), cos(h_roll), PtoR,
        0, 0, 0, 1;

    MatrixXd T3(4, 4);
    T3 << cos(h_pitch), 0, sin(h_pitch), 0,
        0, 1, 0, 0,
        -sin(h_pitch), 0, cos(h_pitch), 0,
        0, 0, 0, 1;

    MatrixXd T4(4, 4);
    T4 << cos(h_yaw), -sin(h_yaw), 0, 0,
        sin(h_yaw), cos(h_yaw), 0, 0,
        0, 0, 1, YtoP,
        0, 0, 0, 1;

    MatrixXd T_EEtobase(4, 4);
    T_EEtobase = T4 * T3 * T2 * T1 * T0;
    return T_EEtobase;
}

MatrixXd returnAngles(MatrixXd T_EEtobase) {
    double theta_pitch;
    double sai_roll;
    double phi_yaw;
    MatrixXd output(3,1);

    if (T_EEtobase(2, 0) != 1 && T_EEtobase(2, 0) != -1)
    {
        theta_pitch = -asin(T_EEtobase(2, 0));
        sai_roll = atan2(T_EEtobase(2, 1) / cos(theta_pitch), T_EEtobase(2, 2) / cos(theta_pitch));
        phi_yaw = atan2(T_EEtobase(1, 0) / cos(theta_pitch), T_EEtobase(0, 0) / cos(theta_pitch));
    }
    else{
            phi_yaw = 0;
            if (T_EEtobase(2, 0) != -1)
            {
                theta_pitch = M_PI / 2;
                sai_roll = atan2(T_EEtobase(0, 1), T_EEtobase(0, 2));
            }
            else{
                theta_pitch = -M_PI / 2;
                sai_roll = atan2(-T_EEtobase(0, 1), -T_EEtobase(0, 2));
                }
    }
    output<< phi_yaw,sai_roll,theta_pitch;
    return output;
}

int main(int argc, char **argv) 
{
    target_to_camera << x, y, z;
    ros::init(argc, argv, "head_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("Camera_Ai", 100, object_detect);
    ros::Publisher  trajectory_data_pub  = nh.advertise<std_msgs::Int32MultiArray>("jointdata/qc",100);
    std_msgs::Int32MultiArray trajectory_data;
    ros::Rate rate_(200);

    MatrixXd T_EEtobase(4, 4);
    MatrixXd init_angle(3,1);
    MatrixXd curr_angle(3,1);
    camera << 0.1248, 0, 0.06746;
    T_EEtobase << ObjToNeck(camera, h_pitch, h_roll, h_yaw, PtoR, YtoP);
    init_angle <<returnAngles(T_EEtobase);
    double PHI = h_yaw;
    double THETA = h_pitch; 
    double time = 0;
    double sx = 0;
    double sy = 0;
    double sz = 0;
    double tempY = 0;
    double tempP = 0;

    vector<int> pitch_range = {-30, 30};
    vector<int> roll_range = {-50, 50};
    vector<int> yaw_range = {-90, 90};
    vector<int> pitch_command_range = {180, 110};
    vector<int> roll_command_range = {100, 190};
    vector<int> yaw_command_range = {90, 210};
    vector<double> head_command(23,0);

    ofstream test;
    test.open("/home/cast/catkin_ws/src/hand_planner_test/src/test_surena4_ros/test.txt", std::ofstream::out);
    ofstream testP;
    testP.open("/home/cast/catkin_ws/src/hand_planner_test/src/test_surena4_ros/testP.txt", std::ofstream::out);
    ofstream testY;
    testY.open("/home/cast/catkin_ws/src/hand_planner_test/src/test_surena4_ros/testY.txt", std::ofstream::out);

    while (time < 20) {

        TEMP.block(n,0,1,3) << target_to_camera;
        target_to_camera << x, y, z;

        if (n>1) {
            if (target_to_camera(0) == TEMP(n,0)){
                sx += 1;
            }       
            else {
                sx = 0;
            }
            if (target_to_camera(1) == TEMP(n,1)){
                sy += 1;
            }
            else {
                sy = 0;
            }
            if (target_to_camera(2) == TEMP(n,2)){
                sz += 1;
            }
            else {
                sz = 0;
            }
        }

        if (abs((PHI-atan2(target_to_camera(1),target_to_camera(0)))*180/M_PI) > 0.015) {
            if (n == 0){
                h_yaw = h_yaw + Ky*(atan2(target_to_camera(1),target_to_camera(0)));
            }
            else{
                h_yaw = h_yaw + Ky*(atan2(target_to_camera(1),target_to_camera(0))-atan2(TEMP(n-sy,1),TEMP(n-sx,0)));
                tempY = h_yaw;
                testY << "n=" << n << ",   "<<"target(1)=" <<target_to_camera(1)<<",   "<<"target(0)="<<target_to_camera(0)<<",   "<<"sy="<<sy<<",   "<<"sx="<<sx<<",   "<<"TEMP(1)="<<TEMP(n-sy,1)<<",   "<<"TEMP(0)="<<TEMP(n-sx,0)<<",  "<<"h_yaw="<<h_yaw*180/M_PI<<",  "<<"error="<<abs((PHI-atan2(target_to_camera(1),target_to_camera(0)))*180/M_PI)<<endl;
                // test << "h_yaw = "<<h_yaw*180/M_PI <<endl;
            }
            }
        else{
            h_yaw = tempY;
            // test << "h_yaw = "<<h_yaw*180/M_PI <<endl;
            testY << "n=" << n << ",   "<<"target(1)=" <<target_to_camera(1)<<",   "<<"target(0)="<<target_to_camera(0)<<",   "<<"sy="<<sy<<",   "<<"sx="<<sx<<",   "<<"TEMP(1)="<<TEMP(n-sy,1)<<",   "<<"TEMP(0)="<<TEMP(n-sx,0)<<",  "<<"h_yaw="<<h_yaw*180/M_PI<<",  "<<"error="<<abs((PHI-atan2(target_to_camera(1),target_to_camera(0)))*180/M_PI)<<endl;
             }
        

        if (abs(180/M_PI*(THETA-atan2(target_to_camera(2),target_to_camera(0)))) > 0.015){
            if (n == 0){
                h_pitch = h_pitch + Kp*(atan2(target_to_camera(2),target_to_camera(0)));
            }
            else{
                h_pitch = h_pitch + Kp*(atan2(target_to_camera(2),target_to_camera(0))-atan2(TEMP(n-sz,2),TEMP(n-sx,0))); 
                tempP = h_pitch;  
                testP << "n=" << n << ",   "<<"target(2)=" <<target_to_camera(2)<<",   "<<"target(0)="<<target_to_camera(0)<<",   "<<"sz="<<sz<<",   "<<"sx="<<sx<<",   "<<"TEMP(2)="<<TEMP(n-sz,2)<<",   "<<"TEMP(0)="<<TEMP(n-sx,0)<<",  "<<"h_pitch="<<h_pitch*180/M_PI<<",  "<<"error="<<abs((THETA-atan2(target_to_camera(2),target_to_camera(0)))*180/M_PI)<<endl;
            }
            }
        else {
            h_pitch = tempP;
            testP << "n=" << n << ",   "<<"target(2)=" <<target_to_camera(2)<<",   "<<"target(0)="<<target_to_camera(0)<<",   "<<"sz="<<sz<<",   "<<"sx="<<sx<<",   "<<"TEMP(2)="<<TEMP(n-sz,2)<<",   "<<"TEMP(0)="<<TEMP(n-sx,0)<<",  "<<"h_pitch="<<h_pitch*180/M_PI<<",  "<<"error="<<abs((THETA-atan2(target_to_camera(2),target_to_camera(0)))*180/M_PI)<<endl;
            }

        T_EEtobase << ObjToNeck(camera, h_pitch, h_roll, h_yaw, PtoR, YtoP);
        curr_angle << returnAngles(T_EEtobase);
        PHI = curr_angle(0)-init_angle(0);
        THETA = curr_angle(2)-init_angle(2);
    
        head_command[21] = int(roll_command_range[0] + (roll_command_range[1] - roll_command_range[0]) * ((-(h_roll*180/M_PI) - (roll_range[0])) / (roll_range[1] - (roll_range[0]))));
        head_command[20] = int(pitch_command_range[0] + (pitch_command_range[1] - pitch_command_range[0]) * ((-(h_pitch*180/M_PI) - pitch_range[0]) / (pitch_range[1] - pitch_range[0])));
        head_command[22] = int(yaw_command_range[0] + (yaw_command_range[1] - yaw_command_range[0]) * ((-(h_yaw*180/M_PI) - yaw_range[0]) / (yaw_range[1] - yaw_range[0])));

        trajectory_data.data.clear();
        for(int  i = 0; i < 23; i++)
        {
            trajectory_data.data.push_back(head_command[i]);
        }
        trajectory_data_pub.publish(trajectory_data);

        n = n + 1;
        time += 0.005;
        ROS_INFO("x=%f, y=%f, z=%f", x, y, z);
        ros::spinOnce();
        rate_.sleep();

    }
    
    testP.close();
    testY.close();
    test.close();
    return 0;
}
