#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32MultiArray.h>
#include <vector>
#include "walk_test/command.h"
#include <math.h>
#include <std_srvs/Empty.h>
#include <iostream>
#include "trajectory_planner/JntAngs.h"
#include "trajectory_planner/Trajectory.h"
#include <ctime>

using namespace std;



class WalkTest{
  public:
    WalkTest(ros::NodeHandle *n){

        motorDataPub_ = n->advertise<std_msgs::Int32MultiArray>("jointdata/qc", 100);
        absSub_ = n->subscribe("/surena/abs_joint_state",100, &WalkTest::absReader, this);
        offsetSub_ = n->subscribe("/surena/inc_joint_state",100, &WalkTest::qcInitial, this);
        incSub_ = n->subscribe("/surena/inc_joint_state",100, &WalkTest::incReader, this);
        jointCommand_ = n->advertiseService("joint_command", &WalkTest::sendCommand, this);
        trajectoryGenerator_ = n->serviceClient<trajectory_planner::Trajectory>("/traj_gen");
        jointAngles_ = n->serviceClient<trajectory_planner::JntAngs>("/jnt_angs");
        absPrinter_ = n->advertiseService("print_absolute", &WalkTest::absPrinter, this);
        walkService_ = n->advertiseService("walk_service", &WalkTest::walk, this);
        homeService_ = n->advertiseService("home_service", &WalkTest::home, this);

        qcInitialBool_ = true;
        int temp_ratio[12] = {100, 100, 50, 80, 100, 100, 50, 80, 120, 120, 120, 120};
        int temp_home_abs[12] = {122018, 135266, 135529, 15199, 125880, 129579, 140562, 131167, 126854, 16320, 138922, 141328};
        int temp_abs_high[12] = {145354, 183778, 180841, 15199, 203256, 160491, 150225, 146143, 168562, 131334, 191978, 172752};
        int temp_abs_low[12] = {108426, 119010, 89833, 131615, 71608, 102443, 119697, 82527, 71238, 16320, 61482, 111376};
        int temp_motor_dir[12] = {1, -1, -1, -1, 1, 1, 1, -1, -1, 1, 1, 1};
        collision_ = false;

        for (int i=0; i<12; i++){
            harmonicRatio_[i] = temp_ratio[i];
            homeAbs_[i] = temp_home_abs[i];
            absHigh_[i] = temp_abs_high[i];
            absLow_[i] = temp_abs_low[i];
            motorDir_[i] = temp_motor_dir[i]; 
        }
        
    }   

    bool setPos(int jointID, int dest){
        // this function is used for changing the position of all joint except ankle joints.
        // the value of dest is on the basis of absolute incs.
        ros::Rate rate_(200);

        while(abs(abs(absData_[jointID]) - dest) > 100){
            if (abs(absData_[jointID]) < 262144){
                if (abs(absData_[jointID]) > dest){
                    motorCommand_.data[jointID] -= motorDir_[jointID]*(4096*4*0.01);
                    cout << "!!!!!!!" << abs(absData_[jointID]) - dest << endl;
                }
                else{
                    motorCommand_.data[jointID] += motorDir_[jointID]*(4096*4*0.01);
                    cout << "@@@@@@" << abs(absData_[jointID]) - dest << endl;

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
        setPos(6, homeAbs_[6]);
        setPos(1, homeAbs_[1] + 20000);
        setPos(0, homeAbs_[0]);
        setPos(7, homeAbs_[7] - 20000);
        ankleHome(false, homeAbs_[5], homeAbs_[4]);
        setPos(3, homeAbs_[3]);
        setPos(2, homeAbs_[2]);
        setPos(1, homeAbs_[1]);
        ankleHome(true, homeAbs_[11], homeAbs_[10]);
        setPos(9, homeAbs_[9]);
        setPos(8, homeAbs_[8]);
        setPos(7, homeAbs_[7]);

       
     
        return true;
    }

    void qcInitial(const sensor_msgs::JointState & msg){

        if (qcInitialBool_){

            for (int i = 0; i <= 31; ++i) {
                qcOffset_[i]=int(msg.position[i+1]);
                motorCommand_.data.push_back(qcOffset_[i]);
                //qcOffset_[i]=int(0);


            }
            qcInitialBool_=false;
            ROS_INFO("Offset=%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t\n",
                     qcOffset_[0],qcOffset_[1],qcOffset_[2],qcOffset_[3],qcOffset_[4],
                    qcOffset_[5],qcOffset_[6],qcOffset_[7],qcOffset_[8],qcOffset_[9],
                    qcOffset_[10],qcOffset_[11],qcOffset_[12],qcOffset_[13],qcOffset_[14],qcOffset_[15],qcOffset_[20],qcOffset_[21],qcOffset_[22],qcOffset_[23]);
        
        }
        
    }

    bool sendCommand(walk_test::command::Request &req, walk_test::command::Response &res){
        
        ros::Rate rate_(200);
        for (int i = 0; i < 100; i++){
        motorCommand_.data[req.motor_id] += req.angle*4*4096;
        motorDataPub_.publish(motorCommand_);
        rate_.sleep();
        }
        for (int i = 0; i < 32; i++){
            cout << motorCommand_.data[i] << "\t";
        }
        //motorDataPub_.publish(motorCommand_);
        cout << "----------------------"<<endl;
        
    }

    void absReader(const sensor_msgs::JointState &msg){
        for(int i=0; i<32; i++){
            absData_[i] = msg.position[i+1];
            //Lower body collision detection
            if(abs(absData_[i]) > 262144){
                cout << "Invalid Abs Data, id: " << i << endl;
                continue;
            }
            if(i < 12 && (absData_[i] >= (absHigh_[i] - 100) || absData_[i] <= (absLow_[i] + 100))){
                collision_ = true;
                cout << "Error: Collision detected!!" << endl;
            }
        }
        //cout << "pitch" << absData_[1] << endl;
        cout << "roll" << absData_[11] << endl;
        cout << "pitch" << absData_[10] << endl;
    }

    void incReader(const sensor_msgs::JointState &msg){
        for(int i=0; i<32; i++){
            incData_[i] = msg.position[i+1];
        }
        //cout << incData_[0] << endl;
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
                    cout << "roll-if:" << abs(absData_[inner]) - roll_dest << endl;
                }
                else{
                    motorCommand_.data[outer] += (4096*4*0.01);//
                    motorCommand_.data[inner] += (4096*4*0.01);//
                    cout << "roll-else:" << abs(absData_[inner]) - roll_dest << endl;

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
                    cout << "pitch-if:" << abs(absData_[outer]) - pitch_dest << endl;
                }
                else{
                    motorCommand_.data[outer] += (4096*4*0.01);
                    motorCommand_.data[inner] -= (4096*4*0.01);
                    cout << "pitch-else:" << abs(absData_[outer]) - pitch_dest << endl;
                } 
            }
            motorDataPub_.publish(motorCommand_);
            ros::spinOnce();
            rate_.sleep();
            
        }
    }

    bool walk(trajectory_planner::Trajectory::Request  &req,
            trajectory_planner::Trajectory::Response &res){
        ros::Rate rate_(200);
        trajectory_planner::Trajectory traj_srv;
        traj_srv.request.alpha = req.alpha;
        traj_srv.request.t_double_support = req.t_double_support;
        traj_srv.request.t_step = req.t_step;
        traj_srv.request.COM_height = req.COM_height;
        traj_srv.request.step_length = req.step_length;
        traj_srv.request.step_count = req.step_count;
        traj_srv.request.ankle_height = req.ankle_height;
        trajectoryGenerator_.call(traj_srv);

        if(traj_srv.response.result){

            for (int i = 0; i < 32; i++){
                motorCommand_.data.push_back(qcOffset_[i]);
            }
            int i = 0;
            while(i < 2000){
                
                trajectory_planner::JntAngs jnt_srv;
                jnt_srv.request.iter = i;
                jointAngles_.call(jnt_srv);
                int amani[] = {5, 4, 3, 2, 10, 11, 9, 8, 0, 1, 7, 6};
                //cout << i << ", ";
                for(int j=0; j < 12; j++){
                    int modification = +1;
                    if (j == 1 || j == 6 || j == 3 || j == 8 || j == 11)
                        modification = -1;
                    
                    motorCommand_.data[j] = jnt_srv.response.jnt_angs[amani[j]]/2/M_PI*2304*harmonicRatio_[j] * modification + qcOffset_[j];
                    //cout << jnt_srv.response.jnt_angs[j] << ", ";
                    
                    //cout << jnt_srv.response.jnt_angs[j] << endl;
                    /*if (i==0 && j == 0){
                        motorCommand_.data[j] += -0.15/2/M_PI*2304*harmonicRatio_[j] * modification;
                        time_t now = time(0);
                        char* dt = ctime(&now);
                        cout << "iteration = " << i << ", motor id = " << j << ", time = " << dt << endl;
                    }else if (i==1 && j == 1)
                    {
                        motorCommand_.data[j] += -0.15/2/M_PI*2304*harmonicRatio_[j] * modification;
                        time_t now = time(0);
                        char* dt = ctime(&now);
                        cout << "iteration = " << i << ", motor id = " << j << ", time = " << dt << endl;
                    }else if (i==2 && j == 2)
                    {
                        motorCommand_.data[j] += 0.15/2/M_PI*2304*harmonicRatio_[j] * modification;
                        time_t now = time(0);
                        char* dt = ctime(&now);
                        cout << "iteration = " << i << ", motor id = " << j << ", time = " << dt << endl;
                    }*/
                        
                }
                //cout << endl;    
                motorDataPub_.publish(motorCommand_);
                rate_.sleep();
                i++;
            }
            res.result = true;
            return true;
        }
        else{
            ROS_INFO("Joint angles were not sent\n");
            res.result = false;
            return false;
        }
    }

private:
    ros::Publisher motorDataPub_;
    ros::Subscriber incSub_;
    ros::Subscriber offsetSub_;
    ros::Subscriber absSub_;
    ros::ServiceServer jointCommand_;
    ros::ServiceServer absPrinter_;
    ros::ServiceClient trajectoryGenerator_;
    ros::ServiceClient jointAngles_;
    ros::ServiceServer walkService_;
    ros::ServiceServer homeService_;
    bool qcInitialBool_;
    int qcOffset_[32];
    std_msgs::Int32MultiArray motorCommand_;
    int harmonicRatio_[12];
    float absData_[32];
    int incData_[32];
    int homeAbs_[12];
    int absHigh_[12];
    int absLow_[12];
    int motorDir_[12];
    int absDir_[12];
    bool collision_;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "eth_subscriber");
    ros::NodeHandle n;
    WalkTest wt(&n);

    /*motorDataPub_ = n.advertise<std_msgs::Int32MultiArray>("jointdata/qc", 100);
    ros::Subscriber incSub_ = n.subscribe("/surena/inc_joint_state",100, qcInitial);
    
    trajectoryGenerator_ = n.serviceClient<trajectory_planner::Trajectory>("/traj_gen");
    jointAngles_ = n.serviceClient<trajectory_planner::JntAngs>("/jnt_angs");
    ros::ServiceServer walkService_ = n.advertiseService("walk_service", walk);
    
    //WalkTest dcm_walk(&n);*/
    ros::spin();
	return 0;
}
