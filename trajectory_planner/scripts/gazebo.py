#!/usr/bin/env python
import numpy as np
import rospy 
from trajectory_planner.srv import JntAngs, Trajectory, GeneralTraj
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu

def imu_callback(data):
    global imu_data
    imu_data = data

def gazebo_sender():
    joint_topics = ["/surenav_ll/r_hip_yaw_joint_position_controller/command", "/surenav_ll/r_hip_roll_joint_position_controller/command", "/surenav_ll/r_hip_pitch_joint_position_controller/command",
                    "/surenav_ll/r_knee_joint_position_controller/command", "/surenav_ll/r_ankle_pitch_joint_position_controller/command", "/surenav_ll/r_ankle_roll_joint_position_controller/command",
                    "/surenav_ll/l_hip_yaw_joint_position_controller/command", "/surenav_ll/l_hip_roll_joint_position_controller/command", "/surenav_ll/l_hip_pitch_joint_position_controller/command",
                    "/surenav_ll/l_knee_joint_position_controller/command", "/surenav_ll/l_ankle_pitch_joint_position_controller/command", "/surenav_ll/l_ankle_roll_joint_position_controller/command"]
    publishers = []
    for topic in joint_topics:
        publishers.append(rospy.Publisher(topic, Float64, queue_size=10))

    rospy.init_node("gazebo_sender", anonymous=True)
    rate = rospy.Rate(500)

    rospy.Subscriber("/imu", Imu, imu_callback)

    rospy.wait_for_service("/general_traj")
    general_motion_handle = rospy.ServiceProxy("/general_traj", GeneralTraj)
    init_com_pos = [0, 0, 0.71]
    init_com_orient = [0, 0, 0]  
    final_com_pos = [0, 0, 0.68]
    final_com_orient = [0, 0, 0]
    init_lankle_pos = [0, 0.1, 0]
    init_lankle_orient = [0, 0, 0]
    final_lankle_pos = [0, 0.1, 0]
    final_lankle_orient = [0, 0, 0]
    init_rankle_pos = [0, -0.1, 0]
    init_rankle_orient = [0, 0, 0]
    final_rankle_pos = [0, -0.1, 0]
    final_rankle_orient = [0, 0, 0]
    init_motion_time = 2.0
    freq = 500
    done = general_motion_handle(init_com_pos,init_com_orient,final_com_pos,final_com_orient,
                init_lankle_pos,init_rankle_orient,final_lankle_pos,final_lankle_orient,
                init_rankle_pos,init_rankle_orient,final_rankle_pos,final_rankle_orient, init_motion_time, 1 / freq)
    
    while (not done):
        print("General Motion Failed, calling again...")
        done = general_motion_handle(init_com_pos,init_lankle_orient,final_com_pos,final_com_orient,
                init_lankle_pos,init_rankle_orient,final_lankle_pos,final_lankle_orient,
                init_rankle_pos,init_rankle_orient,final_rankle_pos,final_rankle_orient, init_motion_time, 1 / freq)
    
    alpha = 0.44
    t_ds = 0.1
    t_step = 1.0
    step_length = 0.2
    step_width = 0.0
    CoM_height = 0.68
    step_count = 5
    ankle_height = 0.025
    theta = 0.0
    rospy.wait_for_service("/traj_gen")
    trajectory_handle = rospy.ServiceProxy("/traj_gen", Trajectory)

    done = trajectory_handle(alpha,t_ds,t_step,step_length,step_width,CoM_height,
                                        step_count, ankle_height, 1 / freq, theta)
    
    while not done:
        print("Trajectory generation failed, calling again...")
        done = trajectory_handle(alpha,t_ds,t_step,step_length,step_width,CoM_height,
                                        step_count, ankle_height, 1 / freq, theta)
    
    iter = 0
    size = freq * (init_motion_time + (step_count + 2) * t_step)
    while (iter < size):
        rospy.wait_for_service("/jnt_angs")
        try:
            joint_state_handle = rospy.ServiceProxy("/jnt_angs", JntAngs)
            left_ft = [1, 1, 1]
            right_ft = [1, 1, 1]
            config = [1 for i in range(12)]
            jnt_vel = [1 for i in range(12)]
            #acc = np.array([imu_data.linear_acceleration.x, imu_data.linear_acceleration.y, imu_data.linear_acceleration.z])
            acc = np.array([0, 0, 0])
            #gyro = np.array([imu_data.angular_velocity.x, imu_data.angular_velocity.y, imu_data.angular_velocity.z])
            gyro = np.array([0, 0, 0])
            bump = np.array([0, 0, 0, 0, 0, 0, 0, 0])
            #left_bump = np.array([0, 0, 0, 0])
            All = joint_state_handle(iter, left_ft, right_ft, config, jnt_vel, acc, gyro, bump).jnt_angs
            
            for i in range(len(All)):
                publishers[i].publish(All[i])
            
            print(All)
        except rospy.ServiceException as e:
            print("Jntangls Service call failed: %s"%e)
        rate.sleep()
        iter += 1
            

if __name__ == "__main__":
    try:
        gazebo_sender()
    except rospy.ROSInterruptException:
        pass
