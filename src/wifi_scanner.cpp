#include "wifi_slam/wifi_scanner.h"
#include <fstream>
#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>
#include <stdio.h>

WifiMapGen::WifiMapGen(ros::NodeHandle &nh){
    nh_ = nh;

    float interval_;
    float almostequal_;
    std::string filePath_;
    std::string accessPath_;
    std::vector<float> currentGoal_;
    std::vector<std::vector<float>> waypoints_;
    std::vector<std::string> access_points_;
    const char* wifi_cmd_;
    const char* acc_pts_cmd_;
    int currentIdx_ = 0;

    nh_.param<float>("stop_interval", interval_, 0.5);
    nh_.getParam("file_path", filePath_);
    nh_.getParam("start_point", currentGoal_);
}

//WifiMapGen::~WifiMapGen(){
//    delete filePath_;
//    delete waypoints_;
//}

void WifiMapGen::initialize(){
    almostequal_ = 0.05;
    wifi_cmd_ = (const char*)"iwconfig wlp3s0";
    acc_pts_cmd_ = (const char*)"sudo iwlist wlp3s0 scanning";
    reach_local_goal_pub_ = nh_.advertise<std_msgs::Bool>("local_goal",1);
}

std::vector<std::vector<float>> WifiMapGen::get_skeleton(){
    std::ifstream inFile;

    std::vector<std::vector<float>> skeleton;
    float x;
    float y;

    inFile.open(filePath_);
    if (!inFile){
        ROS_INFO("Unable to open file");
        exit(1);
    }

    while (inFile >> x >> y){
        std::vector<float> point;
        point.push_back(x);
        point.push_back(y);

        skeleton.push_back(point);
    }
    
    inFile.close();

    return skeleton;
}

void WifiMapGen::get_access_points(){
    std::ifstream inFile;

    std::string ap;

    inFile.open(accessPath_);
    if(!inFile){
        ROS_INFO("Unable to open file");
        exit(1);
    }

    while (inFile >> ap){
        access_points_.push_back(ap);
    }

    inFile.close();
}

std::vector<std::vector<float>> WifiMapGen::generate_waypoints(const std::vector<std::vector<float>> &skeleton){
    
    int num = skeleton.size();
    std::vector<float> start;
    std::vector<float> end;

    float dist;
    int num_wpts;
    float unit_x;
    float unit_y;

    std::vector<std::vector<float>> vertices;

    for (int i=0; i<num-1; i=i+2){
        start = skeleton[i];
        end = skeleton[i+1];

        dist = sqrt(pow((end[1]-start[1]), 2) + pow((end[0]-start[0]),2));
        num_wpts = dist/interval_;

        unit_x = (end[0]-start[0])/interval_;
        unit_y = (end[1]-start[1])/interval_;

        vertices.push_back(start);

        for (int j = 0; j<num_wpts; j++){
            std::vector<float> new_point;
            new_point.push_back(start[0] + unit_x);
            new_point.push_back(end[0] + unit_y);

            vertices.push_back(new_point);
        }
    }
    //TODO : take care of last vertex of map
    
    return vertices;
}

int WifiMapGen::rearrange_wpts(const std::vector<std::vector<float>> &vertices){
    int start_idx;
    int len = vertices.size();

    for (int i=0; i<len; i++){
        if(currentGoal_==vertices[i]){
            start_idx = i;
        }
    }

    for (int i=0; i<len; i++){
        if(start_idx+i < len){
            waypoints_.push_back(vertices[start_idx+i]);
        }
    }

    for (int i=0; i<start_idx; i++){
        waypoints_.push_back(vertices[i]);
    }

    map_.vertices = waypoints_;

    return 0;
}

void WifiMapGen::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg){
    float current_x;
    float current_y;
    float current_oz;
    
    current_x = odom_msg->pose.pose.position.x;
    current_y = odom_msg->pose.pose.position.y;
    current_oz = odom_msg->pose.pose.orientation.z;

    float dist_goal = sqrt(pow((current_x-currentGoal_[0]),2) + pow((current_y-currentGoal_[1]),2));
    
    if (dist_goal<almostequal_){
        std_msgs::Bool msg;
        msg.data = true;
        reach_local_goal_pub_.publish(msg);

    }
    else{
        std_msgs::Bool msg;
        msg.data = false;
        reach_local_goal_pub_.publish(msg);

        //TODO Send commands to move robot
        
    }
}

void WifiMapGen::localGoalCallback(const std_msgs::Bool::ConstPtr& local_goal_msg){
    bool reachGoal = local_goal_msg->data;

    if(reachGoal){

        //TODO Collect wifi information and store in required format
        
        currentIdx_++;
        currentGoal_ = waypoints_[currentIdx_];
    }
}

std::string WifiMapGen::get_data(const char* cmd){
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe){
        throw std::runtime_error("popen failed!");
    }

    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr){
        result += buffer.data();
    }

    return result;
}
