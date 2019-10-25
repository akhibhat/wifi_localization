#include <fstream>
#include <iostream>
#include <vector>
#include <cmath>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>

//std::vector<std::vector<float>> get_skeleton(const std::string file_path){
//    std::ifstream inFile;
//
//    std::vector<std::vector<float>> skeleton;
//    float x;
//    float y;
//
//    inFile.open(file_path);
//    if (!inFile){
//        std::cout << "Unable to open file" << "\n";
//        exit(1);
//    }
//
//    while (inFile >> x >> y){
//        std::vector<float> point;
//        point.push_back(x);
//        point.push_back(y);
//
//        skeleton.push_back(point);
//    }
//    
//    inFile.close();
//    return skeleton;
//}
//
//std::vector<std::vector<float>> generate_waypoints(const std::vector<std::vector<float>> skeleton){
//    
//    int num = skeleton.size();
//    std::cout << num << "\n";;
//    std::vector<float> start;
//    std::vector<float> end;
//    float dist = 0.0;
//    int interval = 5;
//    int num_wpts = 0;
//    float unit_x = 0.0;
//    float unit_y = 0.0;
//
//    std::vector<std::vector<float>> vertices;
//
//    for (int i = 0; i < num-1; i=i+2){
//        start = skeleton[i];
//        end = skeleton[i+1];
//
//        dist = sqrt(pow((end[1]-start[1]),2) + pow((end[0]-start[0]),2));
//        num_wpts = dist/interval;
//        
//        // TODO: Reconsider logic of generating waypoints 
//        unit_x = (end[0]-start[0])/interval;
//        unit_y = (end[1]-start[1])/interval;
//
//        vertices.push_back(start);
//
//        for (int j=0; j<num_wpts; j++){
//
//            std::vector<float> new_point;
//            new_point.push_back(start[0] + unit_x);
//            new_point.push_back(start[1] + unit_y);
//
//            vertices.push_back(new_point);
//        }
//    }
//
//    return vertices;
//}
//
//std::vector<std::vector<float>> rearrange_wpts(std::vector<std::vector<float>> waypts,
//                                               std::vector<float> start_pt){
//    
//    int start_idx;
//    int len = waypts.size();
//
//    std::cout << waypts.size() << "\n";
//
//    std::vector<std::vector<float>> waypoints;
//
//    for(int i=0; i<len; i++){
//        if(start_pt==waypts[i]){
//            start_idx = i;
//        }
//    }
//
//    std::cout << "Start index: " << start_idx << "\n";
//    
//    for(int j=0; j<len; j++){
//        if(start_idx+j < len){
//            waypoints.push_back(waypts[start_idx+j]);
//        }
//    }
//
//    for(int i=0; i<start_idx; i++){
//        waypoints.push_back(waypts[i]);
//    }
//
//    return waypoints;
//}
//
//void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg){
//
//}
//
//void local_goal_cb(const std_msgs::Bool::ConstPtr& local_goal_msg){
//
//}

int main(int argc, char **argv){

    ros::init(argc, argv, "wifi_scanner");

    ros::NodeHandle n;

    std::vector<std::vector<float>> skeleton;
    std::vector<std::vector<float>> vertices;
    std::vector<std::vector<float>> waypoints;

    std::vector<float> start_pt;

    start_pt.push_back(27.357632442768093);
    start_pt.push_back(26.234641167243048);

//    skeleton = get_skeleton("/home/akhilesh/catkin_ws/src/wifi_slam/maps/Levine1Moore1.map");
//    vertices = generate_waypoints(skeleton);
    
//    waypoints = rearrange_wpts(vertices,start_pt);

//    ros::Subscriber odom_sub_ = n.subscribe("/odom", 100, odomCallback);
//    ros::Subscriber local_goal_sub_ = n.subscribe("/local_goal", 100, local_goal_cb);

    return 0;
}
