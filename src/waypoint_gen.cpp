#include <wifi_slam/waypoint_gen.h>
#include <fstream>
#include <sstream>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>
#include <stdio.h>
#include <geometry_msgs/Pose.h>

WaypointGen::WaypointGen(ros::NodeHandle &nh){
    nh_ = nh;

    std::string filePath_;
    std::vector<float> startPoint_;
    std::vector<geometry_msgs::Pose> waypoints_;
    int currentIdx_ = 0;

//    nh_.getParam("file_path", filePath_);
}

WaypointGen::~WaypointGen(){
//    delete filePath_;
//    delete waypoints_;
}

void WaypointGen::initialize(){
    almostequal_ = 0.05;
    filePath_ = "/home/akhilesh/catkin_ws/src/wifi_localization/maps/Levine1Moore1.map";
    accessPath_ = "";
    nh_.advertise<geometry_msgs::Pose>("waypoint",1);
    nh_.subscribe("wifi_tasks", 1, &WaypointGen::reachGoalCallback, this);
}

std::vector<std::vector<float>> WaypointGen::get_skeleton(){
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

std::vector<std::vector<float>> WaypointGen::generate_waypoints(const std::vector<std::vector<float>> &skeleton){
    
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

std::vector<geometry_msgs::Pose> WaypointGen::rearrange_wpts(){
    int start_idx;
    
    std::vector<std::vector<float>> skeleton = get_skeleton();
    std::vector<std::vector<float>> vertices = generate_waypoints(skeleton);
    int len = vertices.size();

    for (int i=0; i<len; i++){
        if(startPoint_ == vertices[i]){
            start_idx = i;
        }
    }

    for (int i=0; i<len; i++){
        if(start_idx+i < len){
            geometry_msgs::Pose wpt;
            wpt.position.x = vertices[start_idx+i][0];
            wpt.position.y = vertices[start_idx+i][1];
            wpt.position.z = 0;
            waypoints_.push_back(wpt);
        }
    }

    for (int i=0; i<start_idx; i++){
        geometry_msgs::Pose wpt;
        wpt.position.x = vertices[i][0];
        wpt.position.y = vertices[i][1];
        wpt.position.z = 0;
        waypoints_.push_back(wpt);
    }
}

void WaypointGen::reachGoalCallback(const std_msgs::Bool::ConstPtr& reach_goal_msg){
    
    if (reach_goal_msg->data != 0){
        geometry_msgs::Pose waypoint_msg = waypoints_[currentIdx_];
        
        currentIdx_++;
        waypoint_pub_.publish(waypoint_msg);
    }
}

std::vector<std::string> WaypointGen::get_access_points(){
    std::ifstream inFile;
    std::string ap;

    inFile.open(accessPath_);
    if(!inFile){
        ROS_INFO("Unable to open file");
        exit(1);
    }

    while (inFile >> ap){
        accessPoints_.push_back(ap);
    }
    inFile.close();
    
    return accessPoints_;
}
