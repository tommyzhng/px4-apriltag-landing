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

// Eigen::Vector3f ApriltagLandingNode::Quat2EulerAngles(const Eigen::Quaternionf& q) {
//     Eigen::Vector3f angles;    //yaw pitch roll
//     const auto x = q.x();
//     const auto y = q.y();
//     const auto z = q.z();
//     const auto w = q.w();

//     // roll (x-axis rotation)
//     double sinr_cosp = 2 * (w * x + y * z);
//     double cosr_cosp = 1 - 2 * (x * x + y * y);
//     angles[0] = std::atan2(sinr_cosp, cosr_cosp);

//     // pitch (y-axis rotation)
//     double sinp = 2 * (w * y - z * x);
//     if (std::abs(sinp) >= 1)
//         angles[1] = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
//     else
//         angles[1] = std::asin(sinp);

//     // yaw (z-axis rotation)
//     double siny_cosp = 2 * (w * z + x * y);
//     double cosy_cosp = 1 - 2 * (y * y + z * z);
//     angles[2] = std::atan2(siny_cosp, cosy_cosp);
//     return angles;
// }

void ApriltagLandingNode::DetectionsCb(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
{
    numDetections_ = msg->detections.size();
    // store specific id -> big tag is 0, small tag is 1 in 16h5 family
    for (int i = 0; i < msg->detections.size(); i++) {
        if (msg->detections[i].id[0] == 0) {
            tagBig_ = msg->detections[i];
        }
        else if (msg->detections[i].id[0] == 1)
        {
            tagSmol_ = msg->detections[i];
        }
    }
}

void ApriltagLandingNode::DronePoseCb(const geometry_msgs::PoseStamped& msg)
{
    _dronePose.x() = msg.pose.position.x;
    _dronePose.y() = msg.pose.position.y;
    _dronePose.z() = msg.pose.position.z;
}

void ApriltagLandingNode::PubLandingTarget(void)
{
    // publish the landing target
    mavros_msgs::PositionTarget msg;
    msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    msg.type_mask = mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY | mavros_msgs::PositionTarget::IGNORE_PZ | mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ | mavros_msgs::PositionTarget::IGNORE_YAW;
    msg.velocity.x = _outputVel.x();
    msg.velocity.y = _outputVel.y();
    msg.velocity.z = _outputVel.z(); // descend at 0.1 m/s if detected
    //msg.yaw_rate = _outputYawRate;
    localVelPub.publish(msg);
}

void ApriltagLandingNode::UpdateTarget(void)
{
    switch (state_)
    {
    case State::NoTag:
        if (numDetections_ > 0)
        {
            SwitchState(State::Approach);
        }
        break;
    case State::Approach:
        // use position control to approach the tag
        ChooseTarget();
        break;
    case State::TrackBigTag:
        ChooseTarget();
        PIDLoop();
        PubLandingTarget();
        if (numDetections_ == 0)
        {
            SwitchState(State::NoTag);
        }
        else if (numDetections_ > 1)
        {
            SwitchState(State::TrackSmolTag);
        }
        break;
    case State::TrackSmolTag:
        ChooseTarget();
        PIDLoop();
        PubLandingTarget();
        if (numDetections_ == 0)
        {
            SwitchState(State::NoTag);
        }
        else if (numDetections_ == 1)
        {
            SwitchState(State::TrackBigTag);
        }
        break;
    case State::Landed:
        break;
    default:
        break;
    }
}

void ApriltagLandingNode::ChooseTarget(void)
{
    //for switching between multiple tags
    if (tagBig.pose.pose.pose.position.z > 0)
    {
        tagPose_.x() = -tagBig.pose.pose.pose.position.y;
        tagPose_.y() = -tagBig.pose.pose.pose.position.x;
        tagPose_.z() = tagBig.pose.pose.pose.position.z;
        //Eigen::Quaternionf q(tagBig.pose.pose.pose.orientation.w, tagBig.pose.pose.pose.orientation.x, tagBig.pose.pose.pose.orientation.y, tagBig.pose.pose.pose.orientation.z);
        //tagOrientationEuler_ = Quat2EulerAngles(q).z();
    }
}

void ApriltagLandingNode::TagPoseWorld(const apriltag_ros::AprilTagDetection& tag)
{
    // convert tag pose from camera frame to world frame
    Eigen::Matrix3d R;
    R << 0, 1, 0,
         1, 0, 0,
         0, 0, -1;
    Eigen::Quaterniond quatNED(R);

    auto vehiclePosition = Eigen
}

void ApriltagLandingNode::PIDLoop(void)
{
    ros::Time now = ros::Time::now();
    double dt =  (now - _lastTime).toSec();
    _error = tagPose_;                              // try to achieve 0,0,0 distance with control loop
    _ierror = _error * dt;                          // i
    _outputVel = _kp * _error + _ki * _ierror;      // simple PI controller for position
    _outputVel.z() = descentRate_;                 // descend at 0.1 m/s

    // write another pid loop for yaw rate
    _outputYawRate = _kp * tagPose_.z(); // simple P controller for yaw rate

    _lastError = _error;
    _lastTime = now;
}


// State Machine

std::string ApriltagLandingNode::StateFb(State state)
{
    switch (state)
    {
    case State::NoTag:
        return "NoTag";
        break;
    case State::Approach:  
        return "Approach";
        break;
    case State::TrackBigTag:
        return "TrackBigTag";
        break;
    case State::TrackSmolTag:
        return "TrackSmolTag";
        break;
    case State::Landed:
        return "Landed";
        break;
    default:
        return "Unknown";
        break;
    }
}

void ApriltagLandingNode::SwitchState(State state)
{
    state_ = state;
    ROS_INFO("Switching to state: %s", StateFb(state).c_str());
}