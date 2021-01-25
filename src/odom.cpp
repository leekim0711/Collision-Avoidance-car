#include<stdio.h>
#include"ros/ros.h"
#include"std_msgs/Float32.h"
#include"geometry_msgs/Pose.h"
#include <tf2/LinearMath/Quaternion.h>
#include<math.h>

#define R2D 180/M_PI

class odom
{
  public:
    odom();
    void kinematics();
    void localize();
    
  private:
    ros::NodeHandle nh;
    ros::Subscriber speedR_sub, speedL_sub;
    ros::Publisher pose_pub;
    geometry_msgs::Pose pose_msg;
    
    double lastms = 0;
    double R_speed = 0, L_speed = 0;
    double v,w;
    double length = 0.1355;
    double x = 0, y = 0, theta = 0;
    
    void speedRCallback(const std_msgs::Float32::ConstPtr& msg);
    void speedLCallback(const std_msgs::Float32::ConstPtr& msg);
};

odom::odom()
{
  speedR_sub = nh.subscribe("/right_speed", 10, &odom::speedRCallback, this);
  speedL_sub = nh.subscribe("/left_speed", 10, &odom::speedLCallback, this);
  pose_pub = nh.advertise<geometry_msgs::Pose>("/pose",10);
}

void odom::speedRCallback(const std_msgs::Float32::ConstPtr& msg)
{
  R_speed = msg->data;
  R_speed/=100.;
}

void odom::speedLCallback(const std_msgs::Float32::ConstPtr& msg)
{
  L_speed = msg->data;
  L_speed/=100.;
}

void odom::kinematics()
{
  
  v = (R_speed + L_speed)/2.;
  w = (R_speed - L_speed)/length;
}

void odom::localize()
{
  ros::Rate loop_rate(10);
  double deltaT;
  while (ros::ok())
  {
    kinematics();
    deltaT = ros::Time::now().toSec() - lastms;
    if(deltaT < 0.001 || deltaT > 1)
      deltaT = 0.1;
    theta += w*deltaT;
    x += v*deltaT*cos(theta);
    y += v*deltaT*sin(theta);
    //ROS_INFO("Delta T: %lf",deltaT);
    ROS_INFO("x : %lf, y : %lf, theta : %lf",x,y,theta*R2D);
    
    pose_msg.position.x = x;
    pose_msg.position.y = y;
    pose_msg.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    pose_msg.orientation.x = q.x();
    pose_msg.orientation.y = q.y();
    pose_msg.orientation.z = q.z();
    pose_msg.orientation.w = q.w();
    
    pose_pub.publish(pose_msg);

    lastms = ros::Time::now().toSec();
    usleep(100*1000);
    //ros::spinOnce();
    //loop_rate.sleep();
  }
}

int main(int argc, char **argv){
	ros::init(argc,argv,"odom");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  odom gogo;
  gogo.localize();
  
  ROS_INFO("Bye Bye~~");
  ros::shutdown();

	return 0;
}