#include "px4_apriltag_landing.hpp"

ApriltagLandingNode::ApriltagLandingNode(ros::NodeHandle& nh)
{
    // initialize some stuff
    tagArraySub = nh.subscribe("/tag_detections", 1, &ApriltagLandingNode::DetectionsCb, this);
    landingTargetPub = nh.advertise<mavros_msgs::LandingTarget>("/mavros/landing_target/raw", 1);
};

ApriltagLandingNode::~ApriltagLandingNode()
{
    // default deconstructor
}

void ApriltagLandingNode::DetectionsCb(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
{
    // store specific id to one tag member var tag36h11
    for (int i = 0; i < msg->detections.size(); i++)
    {
        if (msg->detections[i].id[0] == 0)
        {
            tagBig = msg->detections[i];
        }
        else if (msg->detections[i].id[0] == 1)
        {
            tagSmol = msg->detections[i];
        }
    }
};

void ApriltagLandingNode::PIDLoop(void)
{

}

void ApriltagLandingNode::PubLandingTarget(void)
{
    // publish the landing target
    landingTargetPub.publish(landingTarget);
};

void ApriltagLandingNode::UpdateTarget(void)
{
    // update the target
    PubLandingTarget();
};