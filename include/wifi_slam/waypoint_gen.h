#include <fstream>
#include <iostream>
#include <vector>
#include <cmath>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>

class WaypointGen
{
public:
    WaypointGen(ros::NodeHandle &nh);
    ~WaypointGen();
    void initialize();

private:
    ros::NodeHandle nh_;

    //Publishers
    ros::Publisher waypoint_pub;

    //Subscribers
    ros::Subscriber reach_goal_sub_;

    //Global constants
    float interval_;
    float almostequal_;

    //Global variables
    std::string filePath_;
    std::string accessPath_;
    std::vector<float> startPoint_;
    std::vector<geometry_msgs::Pose> waypoints_;
    std::vector<std::vector<float>> edges_;
    std::vector<std::string> access_points_;
    int currentIdx_;

    //Subscriber Callbacks
    void reachGoalCallback(const std_msgs::Bool::ConstPtr& reach_goal_msg);

    //Other background functions
    std::vector<std::vector<float>> get_skeleton();
    std::vector<std::vector<float>> generate_waypoints(const std::vector<std::vector<float>> &skeleton);
    void rearrange_wpts();
    void get_access_points();
};
