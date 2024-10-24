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

void ApriltagLandingNode::ParsePoseToTarget(void)
{
    // take x, y, z pose and orientation and convert to mavros_msgs::LandingTarget
    landingTarget.header.stamp = ros::Time::now();
    landingTarget.header.frame_id = "fcu";
    landingTarget.target_num = 0;
    
    landingTarget.type = mavros_msgs::LandingTarget::VISION_FIDUCIAL;
    landingTarget.frame = mavros_msgs::LandingTarget::LOCAL_NED;
    landingTarget.distance = tagBig.pose.pose.pose.position.z;
    landingTarget.pose = tagBig.pose.pose.pose;
};

void ApriltagLandingNode::PubLandingTarget(void)
{
    // publish the landing target
    landingTargetPub.publish(landingTarget);
};

void ApriltagLandingNode::UpdateTarget(void)
{
    // update the target
    ParsePoseToTarget();
    PubLandingTarget();
};