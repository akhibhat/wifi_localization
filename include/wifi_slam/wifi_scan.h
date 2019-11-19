#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <wifi_slam/waypoint_gen.h>
#include <wifi_slam/WifiInfo.h>

typedef struct WifiMap{
    std::vector<geometry_msgs::Pose> vertices;
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
        WaypointGen *waypoint_gen_;

        //Publishers
        ros::Publisher reach_local_goal_pub_;
        ros::Publisher twist_pub_;
        ros::Publisher collect_info_pub_;

        //Subscribers
        ros::Subscriber odom_sub_;
        ros::Subscriber next_goal_sub_;
        ros::Subscriber wifi_info_sub_;

        //Global constants
        float interval_;
        float almostequal_;
        std::vector<float> startPoint_;
        float drive_vel_;
        float angular_max_;
        float angular_min_;
        std::string accessPath_;
        std::string meanFile_;
        std::string stdFile_;

        //Global variables
        const char* wifi_cmd_;
        const char* acc_pts_cmd_;
        std::vector<std::string> accessPoints_;
        geometry_msgs::Pose currentGoal_;
        geometry_msgs::Twist stopMsg_;
        int currentIdx_;

        //Subscriber callbacks
        void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);
        void nextGoalCallback(const geometry_msgs::Pose::ConstPtr& waypoint_msg);
        void wifiInfoCallback(const wifi_slam::WifiInfo::ConstPtr& wifi_msg);

        //Other background functions
        std::string get_data(const char* cmd);
        void rotateTurtle(float goal_orient, geometry_msgs::Quaternion curr_quat);
        void driveStraight(float vel, float dist);
        void moveTurtle(double linearVelocity, double angVelocity);
        void writeData();

        //Map Struct
        WifiMap map_;
};
