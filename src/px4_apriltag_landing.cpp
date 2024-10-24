#include "px4_apriltag_landing.hpp"

ApriltagLandingNode::ApriltagLandingNode(ros::NodeHandle& nh)
{
    // initialize some stuff
    tagArraySub = nh.subscribe("/tag_detections", 1, &ApriltagLandingNode::DetectionsCb, this);
    localVelPub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);
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

void ApriltagLandingNode::ChooseTarget(void)
{
    //for switching between multiple tags
    if (tagBig.pose.pose.pose.position.z > 0)
    {
        _tagPose.x() = tagBig.pose.pose.pose.position.x;
        _tagPose.y() = tagBig.pose.pose.pose.position.y;
        _tagPose.z() = tagBig.pose.pose.pose.position.z;
    }
}

void ApriltagLandingNode::PIDLoop(void)
{
    _error = _tagPose; // try to achieve 0,0,0 distance with control loop
    _derror = _error - _lastError;
    _outputVel = kp * _error + kd * _derror;
    _lastError = _error;
}

void ApriltagLandingNode::PubLandingTarget(void)
{
    // publish the landing target
    mavros_msgs::PositionTarget msg;
    msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    msg.type_mask = mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY | mavros_msgs::PositionTarget::IGNORE_PZ | mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ | mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    msg.velocity.x = _outputVel.x();
    msg.velocity.y = _outputVel.y();
    msg.velocity.z = 0.1; // descend at 0.1 m/s
    msg.yaw = _tagYaw;
    localVelPub.publish(msg);
};

void ApriltagLandingNode::UpdateTarget(void)
{
    // update the target
    ChooseTarget();
    PIDLoop();
    PubLandingTarget();
};