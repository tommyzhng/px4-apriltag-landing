#include <ros/ros.h>
#include "px4_apriltag_landing.hpp"

int main(int argc, char **argv) // argc -> # args, argv -> array of args
{
    ros::init(argc, argv, "px4_apriltag_landing_node");
    ros::NodeHandle nh("~"); 
    ros::Rate rate(60);

    // initialize
    ApriltagLandingNode apriltagLandingNode(nh);
    
    while (ros::ok())
    {
        ros::spinOnce();
        apriltagLandingNode.UpdateTarget();
        rate.sleep();
    }
    return 0;
}