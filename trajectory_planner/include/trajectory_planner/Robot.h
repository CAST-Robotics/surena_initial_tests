#pragma once

#include <ros/ros.h>
#include <ros/console.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include "trajectory_planner/JntAngs.h"
#include "trajectory_planner/Trajectory.h"
#include "trajectory_planner/GeneralTraj.h"

#include "json.hpp"
#include "DCM.h"
#include "Link.h"
#include "PID.h"
#include "Controller.h"
#include "Ankle.h"
#include "MinJerk.h"
#include "GeneralMotion.h"
#include "QuatEKF.h"
#include "LieEKF.h"
#include "Collision.h"
#include "Estimator.h"
#include "PreviewTraj.h"
#include "ZMPPlanner.h"
#include "AnkleTraj.h"
#include "FootStepPlanner.h"

#include "fstream"

using namespace std;

class Robot{
    friend class Surena;
    public:
        Robot(ros::NodeHandle* nh, std::string config_path);
        ~Robot();

        void initROSCommunication();
        void initializeRobotParams();
        void initializeLinkObjects(Vector3d a[], Vector3d b[], Vector3d com_pos[], double links_mass[]);

        void spinOnline(int iter, double config[], double jnt_vel[], Vector3d torque_r, Vector3d torque_l, double f_r, double f_l, Vector3d gyro, Vector3d accelerometer, int bump_r[], int bump_l[], double* joint_angles, int& status);
        
        void runFootLenController(int iter, double f_l, double f_r, int traj_index);

        void runBumpFootOrientController(int iter, int bump_r[], int bump_l[]);

        void runEarlyContactController(int iter, int bump_r[], int bump_l[]);

        void runFootOrientController();

        void runZMPAdmitanceController();
        
        bool getJointAngs(int iter, double config[12], double jnt_vel[12], double right_ft[3],
                          double left_ft[3], int right_bump[4], int left_bump[4], double gyro[3],
                          double accelerometer[3], double jnt_command[12],int &status);
        bool trajGen(int step_count, double t_step, double alpha, double t_double_support,
                     double COM_height, double step_length, double step_width, double dt,
                     double theta, double ankle_height, double step_height, bool use_file, 
                     double t_init_double_support, double t_final_double_support);
        bool generalTrajGen(double dt, double time, double init_com_pos[3], double final_com_pos[3], double init_com_orient[3], double final_com_orient[3],
                            double init_lankle_pos[3], double final_lankle_pos[3], double init_lankle_orient[3], double final_lankle_orient[3],
                            double init_rankle_pos[3], double final_rankle_pos[3], double init_rankle_orient[3], double final_rankle_orient[3]);
        bool resetTraj();

        int findTrajIndex(vector<int> arr, int n, int K);

        void distributeFT(Vector3d zmp_y, Vector3d r_foot_y,Vector3d l_foot_y, Vector3d &r_wrench, Vector3d &l_wrench);
        void distributeBump(double r_foot_z, double l_foot_z, double &r_bump, double &l_bump);

        void planFootSteps();
        void planCoMTraj();
        void getInitCondition(Vector3d& x0, Vector3d& y0);
        void planAnkleTraj();

        inline int getTrajSize(){return dataSize_;}

    private:
        enum ControlState {
            IDLE,
            WALK
        };

        vector<ControlState> robotControlState_;

        ros::NodeHandle* nh_;
        std::string robotConfigPath_;

        double thigh_;
        double shank_;
        double torso_;
        double soleXFront_; 
        double soleXBack_;    
        double soleY_;               
        double soleMinDist_;
        double totalMass_;

        int num_steps_ = 2;
        double CoM_height_ = 0.68;
        double step_len_ = 0.15;
        double step_width_ = 0;
        double t_init_ds_ = 1;
        double t_ds_ = 0.1;
        double t_step_ = 1;
        double t_final_ds_ = 1;
        double swing_height_ = 0.025;
        double theta_ = 0;
        double dt_ = 0.005;
        bool use_file_ = true;

        double joints_[12];

        PID* DCMController_;
        PID* CoMController_;
        Controller* onlineWalk_;

        template <typename T>
        T* appendTrajectory(T* old_traj, T* new_traj, int old_size, int new_size){
            /*
                This function appends a new trajectory to an old one.
                for example:
                when you want to call general_traj service twice.
            */
            int total_size = old_size + new_size;
            T* temp_traj = new T[total_size];
            copy(old_traj, old_traj + old_size, temp_traj);
            delete[] old_traj;
            old_traj = temp_traj;
            temp_traj = new_traj;
            copy(temp_traj, temp_traj + new_size, old_traj + old_size);
            return old_traj;
        }

        void doIK(MatrixXd pelvisP, Matrix3d pelvisR, MatrixXd leftAnkleP, Matrix3d leftAnkleR, MatrixXd rightAnkleP, Matrix3d rightAnkleR);
        double* geometricIK(MatrixXd p1, MatrixXd r1, MatrixXd p7, MatrixXd r7, bool isLeft);
        Matrix3d Rroll(double phi);
        Matrix3d RPitch(double theta);

        vector<Vector3d> _CoMPos_;
        vector<Matrix3d> _CoMRot_;
        vector<Vector3d> _rAnklePos_;
        vector<Vector3d> _lAnklePos_;
        vector<Matrix3d> _rAnkleRot_;
        vector<Matrix3d> _lAnkleRot_;
        vector<int> robotState_;
        
        Vector3d* CoMPos_;
        Matrix3d* CoMRot_;
        Vector3d* zmpd_;
        Vector3d* CoMDot_;
        Vector3d* xiDesired_;
        Vector3d* xiDot_;
        Vector3d* rAnklePos_;
        Vector3d* lAnklePos_;
        Matrix3d* rAnkleRot_;
        Matrix3d* lAnkleRot_;
        int* robotPhase_;
        double bumpBiasR_;
        double bumpBiasL_;
        bool bumpSensorCalibrated_;

        Vector3d rSole_;    // current position of right sole
        Vector3d lSole_;    // current position of left sole
        Vector3d* FKBase_;      // current CoM of robot
        Vector3d* FKBaseDot_;
        Vector3d* FKCoM_;
        Vector3d* FKCoMDot_;
        double* FKCoMDotP_;
        Vector3d* realXi_;
        Vector3d* realZMP_;      // current ZMP of robot
        Vector3d* rSoles_;
        Vector3d* lSoles_;
        Vector3d* cntOut_;
        bool leftSwings_;
        bool rightSwings_;

        _Link* links_[13];

        Vector3d CoMEstimatorFK(double config[]);
        void updateRobotState(double config[], double jnt_vel[], Vector3d torque_r, Vector3d torque_l, double f_r, double f_l, Vector3d gyro, Vector3d accelerometer);
        Matrix3d rDot_(Matrix3d R);
        void updateSolePosition();
        Vector3d getZMPLocal(Vector3d torque, double fz);
        Vector3d ZMPGlobal(Vector3d zmp_r, Vector3d zmp_l, double f_r, double f_l);
        Vector3d CoM2Base();
        Vector3d CoM2BaseVel();

        ros::ServiceServer jntAngsServer_;
        ros::ServiceServer trajGenServer_;
        ros::ServiceServer generalTrajServer_;
        ros::ServiceServer resetTrajServer_;
        ros::Publisher zmpDataPub_;
        geometry_msgs::Point zmpPosition_;
        ros::Publisher comDataPub_;
        ros::Publisher xiDataPub_;
        bool isTrajAvailable_;
        bool useController_;

        int index_;
        int size_;
        int dataSize_;
        vector<int> trajSizes_;

        Collision* ankleColide_;
        Estimator* estimator_;
        ZMPPlanner ZMPPlanner_;

        Vector3d lZMP_;
        Vector3d rZMP_;
};
