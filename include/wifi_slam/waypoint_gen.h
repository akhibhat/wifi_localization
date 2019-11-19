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

    std::vector<float> startPoint_;
    std::vector<geometry_msgs::Pose> rearrange_wpts();
    std::vector<std::string> get_access_points();
private:
    ros::NodeHandle nh_;

    //Publishers
    ros::Publisher waypoint_pub_;

    //Subscribers
    ros::Subscriber reach_goal_sub_;

    //Global constants
    float interval_;
    float almostequal_;

    //Global variables
    std::string filePath_;
    std::string accessPath_;
    std::vector<std::vector<float>> edges_;
    int currentIdx_;
    std::vector<std::string> accessPoints_;
    std::vector<geometry_msgs::Pose> waypoints_;

    //Subscriber Callbacks
    void reachGoalCallback(const std_msgs::Bool::ConstPtr& reach_goal_msg);
    std::vector<std::vector<float>> get_skeleton();
    std::vector<std::vector<float>> generate_waypoints(const std::vector<std::vector<float>> &skeleton);

    //Other background functions
};
