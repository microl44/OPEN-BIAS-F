#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "move_base_msgs/MoveBaseActionResult.h"
#include "nav_msgs/Odometry.h"
#include <move_base_msgs/MoveBaseAction.h>
#include "rosgraph_msgs/Log.h"
#include <actionlib/client/simple_action_client.h>
#include <vector>
#include <string>
#include <iostream>
#include <ctime>
#include <fstream>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
std::string status;
double current_x;
double current_y;
std::clock_t start;
double duration;
double goal_x = NULL;
double goal_y = NULL;
move_base_msgs::MoveBaseGoal goal;
double total_distance = 0.0;
double previous_x = 0.0;
double previous_y = 0.0;
bool first_run = true;
bool is_moving = false;
std::ofstream file;
std::string fileLocation = "C:\\ws\\result.csv";
std::string algorithm = "N/A";
std::string world = "N/A";
std::string env_type = "N/A";
std::string map_type = "N/A";
std::string pre_defined = "No";
std::string error = "";

void getGoal(const geometry_msgs::PoseStamped::ConstPtr& msg){
  ROS_INFO_STREAM("Destination: " << msg->pose.position.x << "," << msg->pose.position.y << std::endl);
  start = std::clock();
  is_moving = true;
}

void getStatus(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg){
  file.open(fileLocation, std::ios::out | std::ios::app);
  is_moving = false;
  first_run = true;
  status = msg->status.text;
  // ROS_INFO_STREAM(status << std::endl);
  duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
  ROS_INFO_STREAM("Operation took "<< duration << " seconds and travelled " << total_distance << " meters " << std::endl);
  file << duration << ";" << total_distance << ";" << algorithm << ";" << map_type << 
  ";" << env_type << ";" << status << ";" << error << "\n";
  file.close();
  ros::shutdown();
}

void getOdom(const nav_msgs::Odometry::ConstPtr& msg){
  current_x = msg->pose.pose.position.x;
  current_y = msg->pose.pose.position.y;
  if(is_moving){
    if(first_run){
      previous_x = current_x;
      previous_y = current_y;
    }
    double x = current_x;
    double y = current_y;
    double d_increment = sqrt(((x - previous_x) * (x - previous_x)) + ((y - previous_y) * (y - previous_y)));
    total_distance = total_distance + d_increment;
    previous_x = current_x;
    previous_y = current_y;
    first_run = false;
  }
}

void setGoal(double x, double y){
  ROS_INFO_STREAM("Destination: " << x << "," << y << std::endl);
  start = std::clock();
  is_moving = true;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.w = 1.0;
}

void getOutput(const rosgraph_msgs::Log::ConstPtr& msg){
  if(msg->msg == "Rotate recovery behavior started."){
    error = "Error";
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "experiment_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/move_base_simple/goal", 10, getGoal);
  ros::Subscriber subuwu = nh.subscribe("/move_base/result", 10, getStatus);
  ros::Subscriber odomsub = nh.subscribe("/odom", 10, getOdom);
  ros::Subscriber subout = nh.subscribe("/rosout",10,getOutput);
  nh.getParam("/experiment_node/algorithm", algorithm);
  nh.getParam("/experiment_node/goal_x", goal_x);
  nh.getParam("/experiment_node/goal_y", goal_y);
  nh.getParam("/experiment_node/world", world);
  nh.getParam("/experiment_node/env_type", env_type);
  nh.getParam("/experiment_node/map_type", map_type);

  MoveBaseClient ac("move_base", true);
  //wait for the action server to come up 
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  if(goal_x != NULL && goal_y != NULL){
    pre_defined = "Yes";
    setGoal(goal_x,goal_y);
    ac.sendGoal(goal);
  }
  ros::spin();
  return 0;
}