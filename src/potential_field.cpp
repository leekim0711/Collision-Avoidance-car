#include "ros/ros.h"
#include <cool/polar.h>
#include"std_msgs/Int32.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include<math.h>
#define RAD2DEG(x) ((x)*180./M_PI)

const double length = 13.55;
int PWM_LIMIT = 230;
double ATT_MAX_LINEAR_SP = 7.5;
double ATT_MAX_ROTATION_SP = 8.5;
double REP_MAX_LINEAR_SP = 5.0;
double REP_MAX_ROTATION_SP = 5.0;

struct obstacle
{
  double distance;
  double rad;
};

struct vel_vector
{
  double linear;  //  cm/sec
  double rotation;  //  rad/sec
};

class PF
{
  public:
    PF();
    void run();
    void Attractive(double des_x, double des_y);
    void get_pose(double &x, double &y, double &theta,double &tar_x, double &tar_y, double des_x, double des_y);
    void Repulsive();
    double cal_distance(double des_x, double des_y);
    double cal_angular(double des_x, double des_y);
    void cal_total_vel();
    void inverse_kinematics(double &Vr, double &Vl);
    void publish_pwm(double Vr, double Vl);
    
  private:
    ros::NodeHandle nh;
    ros::Subscriber obs_sub;
    ros::Publisher motorR_pub, motorL_pub;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener* tfListener;
    
    std_msgs::Int32 pwmR_msg,pwmL_msg;
    
    obstacle obs;
    
    vel_vector attr_vel, rep_vel, total_vel;
  
    void obsCallback(const cool::polar::ConstPtr& msg);
  
};

PF::PF()
{
  motorR_pub = nh.advertise<std_msgs::Int32>("/MotorR",10);
  motorL_pub = nh.advertise<std_msgs::Int32>("/MotorL",10);
  obs_sub = nh.subscribe("/obs_polar", 1000, &PF::obsCallback, this);
  tfListener = new tf2_ros::TransformListener(tfBuffer);
}

void PF::get_pose(double &x, double &y, double &theta,double &tar_x, double &tar_y, double des_x, double des_y)
{
  geometry_msgs::TransformStamped transformStamped,transformStamped2;
  geometry_msgs::PointStamped target_pt, transformed_pt;
  target_pt.header.frame_id = "world";
  target_pt.point.x = des_x;
  target_pt.point.y = des_y;
  
  try{
    transformStamped = tfBuffer.lookupTransform("world", "laser", ros::Time(0));
    transformStamped2 = tfBuffer.lookupTransform("laser", "world", ros::Time(0));
    tf2::doTransform(target_pt, transformed_pt, transformStamped2);
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
  x = transformStamped.transform.translation.x;
  y = transformStamped.transform.translation.y;
  tf2::Quaternion q(transformStamped.transform.rotation.x,
                    transformStamped.transform.rotation.y,
                    transformStamped.transform.rotation.z,
                    transformStamped.transform.rotation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  theta = yaw;
  
  tar_x = transformed_pt.point.x;
  tar_y = transformed_pt.point.y;
  ROS_INFO("x : %lf, y : %lf",tar_x, tar_y);
}

double PF::cal_distance(double des_x, double des_y)
{
  return sqrt(pow(des_x, 2) + pow(des_y, 2));
}

double PF::cal_angular(double des_x, double des_y)
{
  return atan2(des_y, des_x);
}

void PF::Attractive(double des_x, double des_y)
{
  double cur_x, cur_y, cur_theta, tar_x, tar_y;
  double v, w;
  //double attr_trans_k = 3.0;
  double attr_trans_k = 5.0;
  double attr_rotat_k = 0.1;

  get_pose(cur_x, cur_y, cur_theta, tar_x, tar_y, des_x, des_y);
  //ROS_INFO("cur x : %lf, cur y : %lf, cur theta : %lf, des_x : %lf, des_y : %lf",cur_x,cur_y,RAD2DEG(cur_theta),des_x,des_y);
  //v = attr_trans_k*cal_distance(des_x, des_y);
  v = ATT_MAX_LINEAR_SP*(1 - exp(-1*attr_trans_k*cal_distance(tar_x, tar_y)));
  //w = attr_rotat_k*cal_angular(des_x, des_y);
  //w = MAX_ROTATION_SP*(1 - exp(-1*attr_trans_k*cal_angular(tar_x, tar_y)));
  w = ATT_MAX_ROTATION_SP*sin(cal_angular(tar_x, tar_y) * attr_rotat_k);
  //ROS_INFO("v : %lf, w : %lf",v,w);
  attr_vel.linear = v;
  attr_vel.rotation = w;
  
  if(cal_distance(tar_x, tar_y) < 0.05){
    while(ros::ok()){
      pwmR_msg.data = 0;
      pwmL_msg.data = 0;
      
      motorR_pub.publish(pwmR_msg);
      motorL_pub.publish(pwmL_msg);
    }
  }
}

void PF::Repulsive()
{
  double v, w;
  double safe_dis = 0.3;
  double rep_trans_k = 10.0;
  double rep_rotat_k = 0.2;
  //double rep_rotat_k = 0.1;
  double obs_dis, obs_ang;
  
  obs_dis = obs.distance;
  obs_ang = obs.rad;
  
  v = REP_MAX_LINEAR_SP/(1-exp((obs_dis-safe_dis)*rep_trans_k));
  
  if(obs_ang >= 0 && obs_ang <= 1.57)
    w = -1 * REP_MAX_ROTATION_SP * cos(obs_ang * rep_rotat_k);
  else if(obs_ang < 0 && obs_ang >= -1.57)
    w = REP_MAX_ROTATION_SP * cos(obs_ang * rep_rotat_k);
  /*if(obs_ang >= 0 && obs_ang <= 1.57)
    w = -1*REP_MAX_ROTATION_SP * cos((obs_ang+M_PI) * rep_rotat_k);
  else if(obs_ang < 0 && obs_ang >= -1.57)
    w = REP_MAX_ROTATION_SP * cos((obs_ang+M_PI) * rep_rotat_k);*/
  else{
    v = 0;
    w = 0;
  }
  
  
  rep_vel.linear = v;
  rep_vel.rotation = w;
}

void PF::cal_total_vel()
{
  double attr_x, attr_y, rep_x, rep_y, tot_x, tot_y;
  attr_x = attr_vel.linear * cos(attr_vel.rotation);
  attr_y = attr_vel.linear * sin(attr_vel.rotation);
  rep_x = rep_vel.linear * cos(rep_vel.rotation);
  rep_y = rep_vel.linear * sin(rep_vel.rotation);
  
  tot_x = attr_x + rep_x;
  tot_y = attr_y + rep_y;
  
  total_vel.linear = cal_distance(tot_x, tot_y);
  total_vel.rotation = cal_angular(tot_x, tot_y);
  ROS_INFO("linear : %lf, rorar : %lf",total_vel.linear, total_vel.rotation);
}

void PF::inverse_kinematics(double &Vr, double &Vl)
{
  Vr = total_vel.linear + length * 0.5 * total_vel.rotation;
  Vl = total_vel.linear - length * 0.5 * total_vel.rotation;
}

void PF::publish_pwm(double Vr, double Vl)
{
  double cmdR, cmdL;
  int pwmR, pwmL;
  cmdR = (Vr/(6.5*M_PI))*1920*0.09;
  cmdL = (Vl/(6.5*M_PI))*1920*0.09;
  
  pwmR = int(round(cmdR));
  pwmL = int(round(cmdL));
  
  if(pwmR >= PWM_LIMIT)
    pwmR = PWM_LIMIT;
  else if(pwmR <= -PWM_LIMIT)
    pwmR = -PWM_LIMIT;
    
  if(pwmL >= PWM_LIMIT)
    pwmL = PWM_LIMIT;
  else if(pwmL <= -PWM_LIMIT)
    pwmL = -PWM_LIMIT;
    
  pwmR_msg.data = pwmR;
  pwmL_msg.data = pwmL;
  
  motorR_pub.publish(pwmR_msg);
  motorL_pub.publish(pwmL_msg);
  
  //ROS_INFO("pwmR : %d, pwmL : %d",pwmR,pwmL);
}

void PF::run()
{
  double Vr,Vl;
  ros::Rate loop_rate(10);
  
  printf("Press Enter to Start!!");
  getchar();
  
  while (ros::ok()){
    
    Attractive(3.5, 0.0);
    Repulsive();
    cal_total_vel();
    inverse_kinematics(Vr, Vl);
    publish_pwm(Vr, Vl);
    //ROS_INFO("Vr : %lf, Vl : %lf",Vr,Vl);
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void PF::obsCallback(const cool::polar::ConstPtr& msg)
{
  obs.distance = msg->distance;
  obs.rad = msg->rad;
  //ROS_INFO("dis : %lf, rad : %lf", obs.distance, RAD2DEG(obs.rad));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "potential_field");
    ros::AsyncSpinner spinner(2);
    spinner.start();
  
    PF ohoh;
    
    ohoh.run();

    ros::shutdown();
    return 0;
}