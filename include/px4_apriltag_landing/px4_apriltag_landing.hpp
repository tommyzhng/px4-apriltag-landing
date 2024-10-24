#ifndef APRILTAG_LANDING_APRILTAG_LANDING_HPP
#define APRILTAG_LANDING_APRILTAG_LANDING_HPP

#include <ros/ros.h>
#include <mavros_msgs/LandingTarget.h>
#include "apriltag_ros/AprilTagDetectionArray.h"

class ApriltagLandingNode
{
public:
    // public functions
    void UpdateTarget(void);
    ApriltagLandingNode(ros::NodeHandle& nh);
    ~ApriltagLandingNode(); // default deconstructor

private:
    // private functions and member vars / private vars
    ros::Subscriber tagArraySub;
    ros::Publisher landingTargetPub;

    void DetectionsCb(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);
    void PubLandingTarget(void);
    void PIDLoop(void);

    // vars to store
    apriltag_ros::AprilTagDetection tagBig;
    apriltag_ros::AprilTagDetection tagSmol;
    mavros_msgs::LandingTarget landingTarget;
};

#endif
