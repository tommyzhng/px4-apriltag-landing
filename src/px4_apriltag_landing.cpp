#include "px4_apriltag_landing.hpp"

ApriltagLandingNode::ApriltagLandingNode(ros::NodeHandle& nh)
{
    // initialize some stuff
    nh.param("/pid_params/kp", _kp, 0.1f);
    nh.param("/pid_params/ki", _ki, 0.1f);
    nh.param("/pid_params/kd", _kd, 0.1f);
    nh.param("/pid_params/sample_time", _sampleTime, 0.1f);

    tagArraySub = nh.subscribe("/tag_detections", 1, &ApriltagLandingNode::DetectionsCb, this);
    // subscribe to local position
    localPosSub = nh.subscribe("/mavros/local_position/pose", 1, &ApriltagLandingNode::PoseCb, this);
    localVelPub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);
};

ApriltagLandingNode::~ApriltagLandingNode()
{
    // default deconstructor
}
Eigen::Vector3f ApriltagLandingNode::Quat2EulerAngles(const Eigen::Quaternionf& q) {
    Eigen::Vector3f angles;    //yaw pitch roll
    const auto x = q.x();
    const auto y = q.y();
    const auto z = q.z();
    const auto w = q.w();

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    angles[0] = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (w * y - z * x);
    if (std::abs(sinp) >= 1)
        angles[1] = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles[1] = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    angles[2] = std::atan2(siny_cosp, cosy_cosp);
    return angles;
}

void ApriltagLandingNode::DetectionsCb(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
{
    // if there are no detections, set descent rate to 0
    if (msg->detections.size() == 0){
        _descentRate = 0;
        return;
    } else {
        if (_dronePose.z() > 2) {
            _descentRate = -0.3;
        } else {
            _descentRate = -0.1;
        }
    }
    // store specific id to one tag member var tag36h11
    for (int i = 0; i < msg->detections.size(); i++) {
        if (msg->detections[i].id[0] == 0) {
            tagBig = msg->detections[i];
        }
        else if (msg->detections[i].id[0] == 1)
        {
            tagSmol = msg->detections[i];
        }
    }
}

void ApriltagLandingNode::PoseCb(const geometry_msgs::PoseStamped& msg)
{
    _dronePose.z() = msg.pose.position.z;
}

void ApriltagLandingNode::ChooseTarget(void)
{
    //for switching between multiple tags
    if (tagBig.pose.pose.pose.position.z > 0)
    {
        _tagPose.x() = -tagBig.pose.pose.pose.position.y;
        _tagPose.y() = -tagBig.pose.pose.pose.position.x;
        _tagPose.z() = tagBig.pose.pose.pose.position.z;
        Eigen::Quaternionf q(tagBig.pose.pose.pose.orientation.w, tagBig.pose.pose.pose.orientation.x, tagBig.pose.pose.pose.orientation.y, tagBig.pose.pose.pose.orientation.z);
        _tagOrientationEuler = Quat2EulerAngles(q).z();
    }
}

void ApriltagLandingNode::PIDLoop(void)
{
    ros::Time now = ros::Time::now();
    double dt =  (now - _lastTime).toSec();
    if (dt < _sampleTime){                          // sample time check
        return;
    }
    _error = _tagPose;                              // try to achieve 0,0,0 distance with control loop
    _derror = (_error - _lastError) / dt;           // derivative
    _outputVel = _kp * _error + _kd * _derror;      // simple PD controller for position

    // write another pid loop for yaw rate
    //_outputYawRate = _kp * _tagOrientationEuler; // simple P controller for yaw rate

    _lastError = _error;
    _lastTime = now;
}

void ApriltagLandingNode::PubLandingTarget(void)
{
    // publish the landing target
    mavros_msgs::PositionTarget msg;
    msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    msg.type_mask = mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY | mavros_msgs::PositionTarget::IGNORE_PZ | mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ | mavros_msgs::PositionTarget::IGNORE_YAW;
    msg.velocity.x = _outputVel.x();
    msg.velocity.y = _outputVel.y();
    msg.velocity.z = _descentRate; // descend at 0.1 m/s if detected
    //msg.yaw_rate = _outputYawRate;
    localVelPub.publish(msg);
};

void ApriltagLandingNode::UpdateTarget(void)
{
    // update the target
    ChooseTarget();
    PIDLoop();
    PubLandingTarget();
};