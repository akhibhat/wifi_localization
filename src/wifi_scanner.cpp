#include <wifi_slam/wifi_scan.h>
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
    nh_.param<float>("stop_interval", interval_, 0.5);
    nh_.getParam("start_point", startPoint_);

    waypoint_gen_ = new WaypointGen(nh_);
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
    currentIdx_ = 0;
    wifi_cmd_ = (const char*)"iwconfig wlp3s0";
    acc_pts_cmd_ = (const char*)"sudo iwlist wlp3s0 scanning";

    map_.access_points = waypoint_gen_->get_access_points();
    map_.vertices = waypoint_gen_->rearrange_wpts();

    reach_local_goal_pub_ = nh_.advertise<std_msgs::Bool>("reached_goal", 1);
    collect_info_pub_ = nh_.advertise<std_msgs::Bool>("wifi_tasks", 1);
    twist_pub_ = nh_.advertise<nav_msgs::Odometry>("cmd_vel", 10);

    odom_sub_ = nh_.subscribe("odom", 1, &WifiMapGen::odomCallback, this);
    next_goal_sub_ = nh_.subscribe("waypoint", 1, &WifiMapGen::nextGoalCallback, this);
    wifi_info_sub_ = nh_.subscribe("wifi_info", 5, &WifiMapGen::wifiInfoCallback, this);
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

    if (distToGoal > almostequal_){
        //function to collect wifi information
        //Then publish high boolean to reached goal topic
        reachGoal.data = 0;
        reach_local_goal_pub_.publish(reachGoal);
        
        //navigate to goal
        goal_orient = atan2(currentGoal_.position.y,currentGoal_.position.x);
        rotateTurtle(goal_orient, currentPos.orientation);
        moveTurtle(drive_vel_, 0);
    }
    else if (distToGoal < almostequal_){
        currentIdx_++;
        reachGoal.data = 1;
        reach_local_goal_pub_.publish(reachGoal);
    }
}

void WifiMapGen::wifiInfoCallback(const wifi_slam::WifiInfo::ConstPtr& wifi_msg){
    std::vector<double> list_mean;
    std::vector<double> list_std;

    list_mean = wifi_msg->wifiMean;
    list_std = wifi_msg->wifiStd;

    for (int i=0; i<list_mean.size(); i++){
        map_.mean_wifi(currentIdx_,i) = list_mean[i];
        map_.std_wifi(currentIdx_,i) = list_std[i];
    }

    std_msgs::Bool taskDone;
    taskDone.data = 1;
    collect_info_pub_.publish(taskDone);
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
