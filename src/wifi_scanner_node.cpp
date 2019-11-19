#include <fstream>
#include <iostream>
#include <vector>
#include <cmath>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <wifi_slam/wifi_scan.h>

int main(int argc, char **argv){

    ros::init(argc, argv, "wifi_scanner");

    ros::NodeHandle nh;

    WifiMapGen wmapnode(nh);
    wmapnode.initialize();

    ros::spin();

    return 0;
}
