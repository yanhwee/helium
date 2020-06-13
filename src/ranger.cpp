#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Range.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ranger");
    ros::NodeHandle n;
    ros::Publisher rangefinder_pub = n.advertise<sensor_msgs::Range>(sensor_msgs::Range(
        
    ))
}