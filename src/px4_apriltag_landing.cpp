#include "px4_apriltag_landing.hpp"

ApriltagLandingNode::ApriltagLandingNode(ros::NodeHandle& nh)
{
    // initialize some stuff
    nh.param("/pid_params/kp", ki_, 0.1f);
    nh.param("/pid_params/ki", ki_, 0.1f);
    nh.param("/pid_params/kd", kd_, 0.1f);
    nh.param("/pid_params/sample_time", sampleTime_, 0.1f);

    tagArraySub_ = nh.subscribe("/tag_detections", 1, &ApriltagLandingNode::DetectionsCb, this);
    // subscribe to local position
    dronePoseSub_ = nh.subscribe("/mavros/local_position/pose", 1, &ApriltagLandingNode::DronePoseCb, this);
    localVelPub_ = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);
};

// Eigen::Vector3d ApriltagLandingNode::Quat2EulerAngles(const Eigen::Quaternionf& q) {
//     Eigen::Vector3d angles;    //yaw pitch roll
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
            const auto& temp_pos = msg->detections[i].pose.pose.pose.position;
            const auto& temp_quat = msg->detections[i].pose.pose.pose.orientation;

            tagBig_.position = Eigen::Vector3d(temp_pos.x, temp_pos.y, temp_pos.z);
            tagBig_.orientation = Eigen::Quaterniond(temp_quat.w, temp_quat.x, temp_quat.y, temp_quat.z);
        }
        else if (msg->detections[i].id[0] == 1)
        {
            const auto& temp_pos = msg->detections[i].pose.pose.pose.position;
            const auto& temp_quat = msg->detections[i].pose.pose.pose.orientation;

            tagSmol_.position = Eigen::Vector3d(temp_pos.x, temp_pos.y, temp_pos.z);
            tagSmol_.orientation = Eigen::Quaterniond(temp_quat.w, temp_quat.x, temp_quat.y, temp_quat.z);
        }
    }
}

void ApriltagLandingNode::DronePoseCb(const geometry_msgs::PoseStamped& msg)
{
    dronePosition_.x() = msg.pose.position.x;
    dronePosition_.y() = msg.pose.position.y;
    dronePosition_.z() = msg.pose.position.z;
    droneOrientation_.w() = msg.pose.orientation.w;
    droneOrientation_.x() = msg.pose.orientation.x;
    droneOrientation_.y() = msg.pose.orientation.y;
    droneOrientation_.z() = msg.pose.orientation.z;
}

void ApriltagLandingNode::PubLandingTarget(void)
{
    // publish the landing target
    mavros_msgs::PositionTarget msg;
    msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    msg.type_mask = mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY | mavros_msgs::PositionTarget::IGNORE_PZ | mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ | mavros_msgs::PositionTarget::IGNORE_YAW;
    msg.velocity.x = outputVel_.x();
    msg.velocity.y = outputVel_.y();
    msg.velocity.z = outputVel_.z(); // descend at 0.1 m/s if detected
    //msg.yaw_rate = outputYawRate_;
    localVelPub_.publish(msg);
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
        curTag_ = tagBig_;
        break;
    case State::TrackBigTag:
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

void ApriltagLandingNode::TagPoseLocal(const Apriltag& tag)
{
    // convert tag pose from camera frame to world frame
    Eigen::Matrix3d R;
    R << 0, 1, 0,
         1, 0, 0,
         0, 0, -1;
    Eigen::Quaterniond quatNED(R);

    Eigen::Affine3d droneTransform = Eigen::Translation3d(dronePosition_) * droneOrientation_;
    Eigen::Affine3d cameraTransform = Eigen::Translation3d(0, 0, 0) * quatNED;
    Eigen::Affine3d tagTransform = Eigen::Translation3d(tag.position) * tag.orientation;
    Eigen::Affine3d tagWorldTransform = droneTransform * cameraTransform * tagTransform;

    localTag_ = {
        .position = tagWorldTransform.translation(),
        .orientation = Eigen::Quaterniond(tagWorldTransform.rotation())
    };
}

void ApriltagLandingNode::PIDLoop(void)
{
    ros::Time now = ros::Time::now();
    double dt =  (now - lastTime_).toSec();
    error_ = curTag_.position;                              // try to achieve 0,0,0 distance with control loop
    ierror_ = error_ * dt;                          // i
    outputVel_ = ki_ * error_ + ki_ * ierror_;      // simple PI controller for position
    outputVel_.z() = descentRate_;                 // descend at 0.1 m/s

    // write another pid loop for yaw rate
    outputYawRate_ = ki_ * curTag_.position.z(); // simple P controller for yaw rate
    lastTime_ = now;
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