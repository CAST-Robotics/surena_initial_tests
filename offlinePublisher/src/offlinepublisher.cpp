#include "mainwindow.h"
#include <QApplication>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include<std_msgs/Int32MultiArray.h>
#include <stdlib.h>
#include "datareader.h"
#include "offlinepublisher/srv1.h"
#include "offlinepublisher/num.h"

int _dataIndex=0;
QList<int> _motorPosition;
bool add(offlinepublisher::srv1::Request  &req,
         offlinepublisher::srv1::Response &res)
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}
//echo "source  /home/amin/Desktop/catkin_ws/devel/setup.sh" >> ~/.bashrc
//source  /home/amin/Desktop/catkin_ws/devel/setup.sh
//check service with this command rosservice call /add "a: 4 b: 2"
//
int main(int argc, char *argv[])
{
  QApplication a(argc, argv);
  DataReader _dataReader;

  _dataReader.Load("/home/surena/Documents/data.txt");
  //Initializes ROS, and sets up a node
  ros::init(argc, argv, "offline_publisher");
  ros::NodeHandle nh,nh2;
  ros::ServiceServer service = nh.advertiseService("add", add);

  ros::Publisher pub=nh.advertise<std_msgs::Int32MultiArray>("jointdata/qc",100);
  ros::Publisher pub2=nh.advertise<offlinepublisher::num>("custom_msg",100);

  //Sets up the random number generator
  srand(time(0));

  //Sets the loop to publish at a rate of 10Hz
  ros::Rate  rate(200);

  std_msgs::Int32MultiArray msg;
  std_msgs::MultiArrayDimension msg_dim;
  offlinepublisher::num myCustomMsg;
myCustomMsg.age=40;
myCustomMsg.first_name="amin";

myCustomMsg.last_name="amani";
myCustomMsg.score=20;




  msg_dim.label = "jointdata/qc";
  msg_dim.size = 1;
  msg.layout.dim.clear();
  msg.layout.dim.push_back(msg_dim);

int pos=0;
bool dir=false;
  while(ros::ok()) 
  {

    msg.data.clear();
    //_dataReader.GetData(_motorPosition,_dataIndex++);
    for(size_t i = 0; i < 16; i++)
    {
	if(i == 0 || i > 5)
		msg.data.push_back(0);
	else{
		if(dir){
		  //_motorPosition[i]=pos;
		  pos+=10;
		  }
		else{
		   //_motorPosition[i]=pos;
		   pos-=10;
		   }
		msg.data.push_back(pos);
	}
    }

    pub.publish(msg);
    if(pos>(16384*3) || pos<(-16384*3)  )
    dir=!dir;
   // pub2.publish(myCustomMsg);
    ROS_INFO("this is %d" ,_motorPosition[0]);
    //Delays untill it is time to send another message
    ros::spinOnce();// let system to receive callback from another nodes
    rate.sleep();
  }

  return 0;
  return a.exec();
}
