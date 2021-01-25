#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <cool/polar.h>
#include<math.h>
#define RAD2DEG(x) ((x)*180./M_PI)
const double react_dis = 1.0;
const double distance_threshold = 0.03;
const double degree_threshold = 1.5;



struct laser_data
{
  double degree[360];
  double distance[360];
};

struct obstacle
{
  int points;
  double degree[360];
  double distance[360];
};

struct point
{
  double x;
  double y;
};

class Object_detect
{
  public:
    Object_detect();
    void segment(int count);
    int cluster(int count);
    int assemble(int index, int cluster_count);
    int find_closest_obs(int cluster_number);
    int find_closest_pt(int shortest_cluster);
    double avg_distance(struct obstacle obs);
    void polar_to_cartesian(int shortest_cluster, int rep_point);
    
  private:
    ros::NodeHandle nh;
    ros::Subscriber lidar_sub;
    ros::Publisher polar_pub;
    
    laser_data raw_data, segment_data;
    obstacle obstacle_data[50];
    obstacle closest_obs;
    point pt, rep_pt;
    
    cool::polar polar_msg;
    
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
};

Object_detect::Object_detect()
{
  lidar_sub = nh.subscribe("/scan", 1000, &Object_detect::scanCallback, this);
  polar_pub = nh.advertise<cool::polar>("/obs_polar",10);
}

void Object_detect::polar_to_cartesian(int shortest_cluster, int rep_point)
{
  double r, theta;
  r = obstacle_data[shortest_cluster].distance[rep_point];
  theta = obstacle_data[shortest_cluster].degree[rep_point];
  
  pt.x = r*cos(theta);
  pt.y = r*sin(theta);
  //ROS_INFO("x : %lf, y : %lf",pt.x,pt.y);
}

int Object_detect::find_closest_pt(int shortest_cluster)
{
  bool first = true;
  int shortest_index;
  double shortest_dis;
  for(int i=0;i<obstacle_data[shortest_cluster].points;i++){
    double buffer = obstacle_data[shortest_cluster].distance[i];
    if(first){
      shortest_index = i;
      shortest_dis = buffer;
      first = false;
    }
    if(buffer < shortest_dis){
      shortest_dis = buffer;
      shortest_index = i;
    }
  }
  
  if(!first)
    //ROS_INFO("The shortest cluster : %d , the shortest point : %d, dis : %lf, anf : %lf", shortest_cluster, shortest_index, shortest_dis, RAD2DEG(obstacle_data[shortest_cluster].degree[shortest_index]));
  
  return shortest_index;
}

double Object_detect::avg_distance(struct obstacle obs)
{
  double count = 0;
  for(int i = 0 ; i < obs.points ; i++){
    count += obs.distance[i];
  }
  double avg = count/obs.points;
  
  return  avg;
}

int Object_detect::find_closest_obs(int cluster_number)
{
  bool first = true;
  int shortest_index;
  double shortest_dis;
  for(int i=0;i<cluster_number;i++){
    if(obstacle_data[i].points != 1){
      double buffer = avg_distance(obstacle_data[i]);
      if(first){
        shortest_index = i;
        shortest_dis = buffer;
        first = false;
      }
      if(buffer < shortest_dis){
        shortest_dis = buffer;
        shortest_index = i;
      }
      //ROS_INFO("Cluster number : %d, avg distance : %lf",i,buffer);
    }
    
  }
  
  if(!first)
    //ROS_INFO("The shortest cluster : %d , dis : %lf", shortest_index, shortest_dis);
  
  return shortest_index;
}

int Object_detect::assemble(int index, int cluster_count)
{

  bool end = false;
  double dis = 0;
  double ang = 0;
  int point_index = 0;
  int its_ok = 0;
  
  obstacle_data[cluster_count].distance[point_index] = segment_data.distance[index];
  obstacle_data[cluster_count].degree[point_index] = segment_data.degree[index];
  point_index++;
  
  while(!end){
    index++;
    if(index >358)
      break;
    
    if(its_ok > 2)
      break;
    if(isinf(segment_data.distance[index]) == 0 ){
      if(index < 359){
        dis = segment_data.distance[index] - segment_data.distance[index-1];
        ang = segment_data.degree[index] - segment_data.degree[index-1];
      }
      else
        dis = segment_data.distance[358] - segment_data.distance[0];
        
      if(abs(dis) < distance_threshold && RAD2DEG(abs(ang)) < degree_threshold){
        obstacle_data[cluster_count].distance[point_index] = segment_data.distance[index];
        obstacle_data[cluster_count].degree[point_index] = segment_data.degree[index];
        point_index++;
      }
      else if(dis > distance_threshold)
        break;
      else if(RAD2DEG(abs(ang)) > degree_threshold)
        break;
    
    }
    else if(isinf(segment_data.distance[index]) != 0){
      its_ok++;
    }
    
  }
  obstacle_data[cluster_count].points = point_index;
  return index;
}

int Object_detect::cluster(int count)
{
  int cluster_count = 0;
  for(int i=0; i<count ;i++){
    if(isinf(segment_data.distance[i]) == 0){
      int j = assemble(i, cluster_count);
      cluster_count++;
      i = j;
    }
    //ROS_INFO("%d",cluster_count);
  }
  /*ROS_INFO("Number of cluster : %d",cluster_count);
  for(int i=0;i<cluster_count;i++){
    ROS_INFO("Cluster number : %d, points : %d",i,obstacle_data[i].points);
    for(int j=0;j<obstacle_data[i].points;j++){
       ROS_INFO(": [%lf, %lf]", RAD2DEG(obstacle_data[i].degree[j]), obstacle_data[i].distance[j]);
    }
  }*/
  
  return cluster_count;
}

void Object_detect::segment(int count)
{
  for(int i=0; i<count ;i++){
    if(raw_data.distance[i] < react_dis){
      segment_data.distance[i] = raw_data.distance[i];
      segment_data.degree[i] = raw_data.degree[i];
    }
    else{
      segment_data.distance[i] = 1.0/0.0;
      segment_data.degree[i] = raw_data.degree[i];
    }
  }
  
}

void Object_detect::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int count = scan->scan_time / scan->time_increment;
    //ROS_INFO("I heard a laser scan %s[%d]:", scan->header.frame_id.c_str(), count);
    //ROS_INFO("angle_range, %f, %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));
  
    for(int i = 0; i < count; i++) {
        //float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        //ROS_INFO(": [%f, %f]", degree, scan->ranges[i]);
        raw_data.degree[i] = scan->angle_min + scan->angle_increment * i;
        raw_data.distance[i] = scan->ranges[i];
    }
    segment(count);
    /*for(int i = 0; i < count; i++) {
        //float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        ROS_INFO(": [%lf, %lf]", RAD2DEG(segment_data.degree[i]), segment_data.distance[i]);
    }*/
    int cluster_number = cluster(count);
    int shortest_cluster = find_closest_obs(cluster_number);
    int rep_point = find_closest_pt(shortest_cluster);
    polar_msg.distance = obstacle_data[shortest_cluster].distance[rep_point];
    polar_msg.rad = obstacle_data[shortest_cluster].degree[rep_point];
    polar_pub.publish(polar_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_detection");
    Object_detect yaya;

    ros::spin();
    return 0;
}