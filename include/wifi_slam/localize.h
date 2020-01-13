#include<iostream.h>
#include<ros/ros.h>
#include<vector>

class Localize
{
    public:
        Localize(ros::NodeHandle &nh);
        ~Localize();
        void initialize();
    private:
        ros::NodeHandle nh_;

}
