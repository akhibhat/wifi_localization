#include "wifi_slam/wifi_scan.h"
#include <fstream>
#include <sstream>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>
#include <stdio.h>
#include <math.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

WifiMapGen::WifiMapGen(ros::NodeHandle &nh){
    nh_ = nh;

    float interval_;
    float almostequal_;
    std::vector<float> startPoint_;
//    geometry_msgs::Pose currentGoal_;
//    const char* wifi_cmd_;
//    const char* acc_pts_cmd_;
//    Eigen::MatrixXf mean_wifi_;
//    Eigen::MatrixXf std_wifi_;

    nh_.param<float>("stop_interval", interval_, 0.5);
    nh_.getParam("start_point", startPoint_);
}

//WifiMapGen::~WifiMapGen(){
//    delete filePath_;
//    delete waypoints_;
//}

void WifiMapGen::initialize(){
    almostequal_ = 0.05;
    drive_vel_ = 0.3;
    angular_max_ = 0.2;
    angular_min_ = 0.05;
    stopMsg_.linear.x = 0;
    stopMsg_.angular.z = 0;
    wifi_cmd_ = (const char*)"iwconfig wlp3s0";
    acc_pts_cmd_ = (const char*)"sudo iwlist wlp3s0 scanning";

    reach_local_goal_pub_ = nh_.advertise<std_msgs::Bool>("reached_goal", 1);
    twist_pub_ = nh_.advertise<nav_msgs::Odometry>("cmd_vel", 10);

    odom_sub_ = nh_.subscribe("odom", 1, &WifiMapGen::odomCallback, this);
    next_goal_sub_ = nh_.subscribe("waypoint", 1, &WifiMapGen::nextGoalCallback, this);
}

void WifiMapGen::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg){
    float current_x;
    float current_y;
    geometry_msgs::Quaternion current_quat;
    float goal_orient;
    geometry_msgs::Pose currentPos;
    std_msgs::Bool reachGoal;
    
    current_x = odom_msg->pose.pose.position.x;
    current_y = odom_msg->pose.pose.position.y;
    current_quat = odom_msg->pose.pose.orientation;
    
    //transform the goal wrt the map and then send twist messages
    currentPos.position.x = current_x + startPoint_[0];
    currentPos.position.y = current_y + startPoint_[1];

    //TODO Do some check to make orientation change efficient
    currentPos.orientation = current_quat;

    float distToGoal = sqrt(pow(currentPos.position.x - currentGoal_.position.x, 2) + pow(currentPos.position.y - currentGoal_.position.y, 2));

    if (distToGoal < almostequal_){
        //function to collect wifi information
        //Then publish high boolean to reached goal topic
        reachedGoalTasks();

        reachGoal.data = 1;
        reach_local_goal_pub_.publish(reachGoal);
    }
    else{
        reachGoal.data = 0;
        reach_local_goal_pub_.publish(reachGoal);
        
        //navigate to goal
        goal_orient = atan2(currentGoal_.position.y,currentGoal_.position.x);
        rotateTurtle(goal_orient, currentPos.orientation);
        moveTurtle(drive_vel_, 0);
    }
}

void WifiMapGen::nextGoalCallback(const geometry_msgs::Pose::ConstPtr& waypoint_msg){
    currentGoal_.position = waypoint_msg->position;
    currentGoal_.orientation = waypoint_msg->orientation;
}

void WifiMapGen::rotateTurtle(float goal_orient, geometry_msgs::Quaternion curr_quat){
    
    tf2::Quaternion quat_tf;
    tf2::fromMsg(curr_quat, quat_tf);
    tf2::Matrix3x3 m(quat_tf);
    
    double goal_z_degree;
    double total_turn;
    double goalAngle;

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    goal_z_degree = (goal_orient*180)/3.14;
    total_turn = goal_z_degree + yaw;

    if (total_turn > 180){
        goalAngle = total_turn - 360;
    }
    else if(total_turn < -180){
        goalAngle = total_turn + 360;
    }
    else{
        goalAngle = total_turn;
    }

    if ((yaw < goalAngle - 1.5) or (yaw > goalAngle + 1.5)){
        moveTurtle(0,(goal_orient/abs(goal_orient)*0.2));
    }
    else{
        twist_pub_.publish(stopMsg_);
    }
}

void WifiMapGen::moveTurtle(double linearVelocity, double angVelocity){
    geometry_msgs::Twist move_msg;

    move_msg.linear.x = linearVelocity;

    if (angVelocity > angular_max_){
        move_msg.angular.z = angular_max_;
    }
    else if (angVelocity < angular_min_ and angVelocity > 0){
        move_msg.angular.z = angular_min_;
    }
    else{
        move_msg.angular.z = angVelocity;
    }

    twist_pub_.publish(move_msg);
}

void WifiMapGen::reachedGoalTasks(){

    //TODO Collect wifi information and store in required format
    std::string access_pts = get_data(acc_pts_cmd_);

    std::istringstream f(access_pts);
    std::string line;

    std::vector<std::string> address_line;
    std::vector<std::string> strength_line;

    std::vector<std::string> temp_address;
    std::vector<std::string> temp_str;
    std::vector<std::string> temp_ssid;

    while (std::getline(f, line)){
        if (line.length()>35){
            std::string out_line = line.substr(20,35);
            if (out_line.substr(0,7) == "Address"){
                temp_address.push_back(out_line.substr(9,17));
            }
            if (out_line.substr(0,7) == "Quality"){
                temp_str.push_back(out_line.substr(28,3));
            }
            if (out_line.substr(0,5) == "ESSID"){
                temp_ssid.push_back(out_line.substr(6));
            }
        }
    }

    for (int i=0; i<temp_str.size(); i++){
        if (temp_ssid[i] == "AirPennNet-Device"){
            address_line.push_back(temp_address[i]);
            strength_line.push_back(temp_str[i]);
        }
    }
    
//    for (int i=0; i<address_line.size(); i++){
//        int j;
//        for (j=0; j<access_points_.size(); j++){
//            if (access_points_[j] == address_line[i]){
//                
//            }
//        }
//    }
    
//    currentIdx_++;
//    currentGoal_ = waypoints_[currentIdx_];
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
