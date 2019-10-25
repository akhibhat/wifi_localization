#include <fstream>
#include <iostream>
#include <vector>
#include <cmath>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>

typedef struct WifiMap{
    std::vector<std::vector<float>> vertices;
    std::vector<std::vector<float>> edges;
    std::vector<std::string> access_points;
    Eigen::MatrixXf mean_wifi;
    Eigen::MatrixXf std_wifi;
} WifiMap;

class WifiMapGen
{
public:
    WifiMapGen(ros::NodeHandle &nh);
    ~WifiMapGen();
    void initialize();

private:
    ros::NodeHandle nh_;

    //Publishers
    ros::Publisher reach_local_goal_pub_;
    ros::Publisher reach_start_pub_;
    ros::Publisher start_mission_pub_;
    ros::Publisher twist_pub_;

    //Subscribers
    ros::Subscriber odom_sub_;
    ros::Subscriber reach_local_goal_sub_;
    ros::Subscriber end_mission_sub_;

    //Global constants
    float interval_;
    float almostequal_;

    //Global variables
    std::string filePath_;
    std::string accessPath_;
    std::vector<float> currentGoal_;
    std::vector<std::vector<float>> waypoints_;
    std::vector<std::vector<float>> edges_;
    std::vector<std::string> access_points_;
    Eigen::MatrixXf mean_wifi_;
    Eigen::MatrixXf std_wifi_;
    int currentIdx_;
    const char* wifi_cmd_;
    const char* acc_pts_cmd_;

    //Subscriber callbacks
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);
    void localGoalCallback(const std_msgs::Bool::ConstPtr& local_goal_msg);
    void missionCallback(const std_msgs::Bool::ConstPtr& mission_msg);
    
    //Other background functions
    std::vector<std::vector<float>> get_skeleton();
    std::vector<std::vector<float>> generate_waypoints(const std::vector<std::vector<float>> &skeleton);
    int rearrange_wpts(const std::vector<std::vector<float>> &vertices);
    std::string get_data(const char* cmd);
    void get_access_points();

    //Map struct
    WifiMap map_;
};
