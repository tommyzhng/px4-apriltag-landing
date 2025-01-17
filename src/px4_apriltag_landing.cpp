#include "px4_apriltag_landing.hpp"

ApriltagLandingNode::ApriltagLandingNode(ros::NodeHandle& nh)
{
    // initialize some stuff
    nh.param("/big_tag/kp", ki_, 0.1f);
    nh.param("/big_tag/ki", ki_, 0.1f);
    nh.param("/smol_tag/kp", kp1_, 0.1f);
    nh.param("/smol_tag/ki", ki_, 0.1f);
    nh.param("/big_tag/sample_time", sampleTime_, 0.1f);

    tagArraySub_ = nh.subscribe("/tag_detections", 1, &ApriltagLandingNode::DetectionsCb, this);
    dronePoseSub_ = nh.subscribe("/mavros/local_position/pose", 1, &ApriltagLandingNode::DronePoseCb, this);
    commandClient_ = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
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
    // store specific id -> big tag is 0, small tag is 1 in 16h5 family
    // reset detections to 0
    detections_ = {0,0};
    for (int i = 0; i < msg->detections.size(); i++) {
        if (msg->detections[i].id[0] == 0) {
            const auto& temp_pos = msg->detections[i].pose.pose.pose.position;
            const auto& temp_quat = msg->detections[i].pose.pose.pose.orientation;

            tagBig_.position = Eigen::Vector3d(temp_pos.x, temp_pos.y, temp_pos.z);
            tagBig_.orientation = Eigen::Quaterniond(temp_quat.w, temp_quat.x, temp_quat.y, temp_quat.z);
            detections_(0) = 1;
        }
        else if (msg->detections[i].id[0] == 1)
        {
            const auto& temp_pos = msg->detections[i].pose.pose.pose.position;
            const auto& temp_quat = msg->detections[i].pose.pose.pose.orientation;

            tagSmol_.position = Eigen::Vector3d(temp_pos.x, temp_pos.y, temp_pos.z);
            tagSmol_.orientation = Eigen::Quaterniond(temp_quat.w, temp_quat.x, temp_quat.y, temp_quat.z);
            detections_(1) = 1;
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

void ApriltagLandingNode::PubVelocityTarget(void)
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

void ApriltagLandingNode::PubPositionTarget(double x, double y, double z)
{
    // publish the landing target
    mavros_msgs::PositionTarget msg;
    msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    msg.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX | 
                    mavros_msgs::PositionTarget::IGNORE_AFY | 
                    mavros_msgs::PositionTarget::IGNORE_AFZ | 
                    mavros_msgs::PositionTarget::IGNORE_YAW;

    msg.position.x = x;
    msg.position.y = y;
    msg.position.z = z;
    //msg.velocity.z = outputVel_.z(); // descend at 0.1 m/s if detected
    //msg.yaw_rate = outputYawRate_;
    localVelPub_.publish(msg);
}

void ApriltagLandingNode::UpdateTarget(void)
{
    // TBD: some timeout/watchdog logic to ensure tag is not lost

    switch (state_)
    {
    case State::NoTag:

        if (detections_ != Eigen::Vector2i(0,0)) // if there is a detection
        {
            SwitchState(State::Approach);
        }
        break;

    case State::LostTag:
    
        // if tag is lost, go to last known position
        PubPositionTarget(localTag_.position.x(), localTag_.position.y(), lastAlt_);

        switch (lastState_)
        {
        case (State::TrackBigTag):
            if (detections_(0) == 1)
            {
                SwitchState(State::TrackBigTag);
                break;
            }
            break;
        case (State::TrackSmolTag):
            if (detections_(1) == 1)
            {
                SwitchState(State::TrackSmolTag);
                break;
            }
            break;
        default:
            break;
        }   

    case State::Approach:

        // use position control to approach the tag
        // approach the big tag first
        lastState_ = State::Approach;
        if (detections_(0) == 1)
        {
            TagPoseLocal(tagBig_); // now localTag_ is the bigTag_s position in NED
            lastAlt_ = dronePosition_.z(); // last alt since tag detection for lost tag state
        }
        // still approach the tag if its lost
        ROS_INFO("Tag position in NED: %f, %f, %f", localTag_.position.x(), localTag_.position.y(), localTag_.position.z());
        PubPositionTarget(localTag_.position.x(), localTag_.position.y(), apprThreshold_);

        if (dronePosition_.z() <= apprThreshold_ && detections_(0) == 1) // if drone has crossed altitude boundary and has detected a big tag
        {
            SwitchState(State::TrackBigTag);
            descentRate_ = -0.4;
        }
        break;

    case State::TrackBigTag:

        // use velocity control to track the big tag with position as secondary source in LostTag state

        lastState_ = State::TrackBigTag;
        if ((detections_(0)) != 1)
        {
            SwitchState(State::LostTag);
            break;  
        }
        
        // controller loop
        PIDLoop(tagBig_);
        PubVelocityTarget();
        TagPoseLocal(tagBig_); // as a backup for lost tag state
        lastAlt_ = dronePosition_.z(); // last alt since tag detection for lost tag state

        if (dronePosition_.z() < smolThreshold_ && detections_(1) == 1) // if drone has crossed altitude boundary and has detected a small tag
        {
            SwitchState(State::TrackSmolTag);
            // turn gains down (very professional adaptive controller T-T)
            kp_ = kp1_;
            ki_ = ki1_;
            descentRate_ = -0.15;
        }
        break;

    case State::TrackSmolTag:

        // use velocity control to track the small tag with position as secondary source in LostTag state
        
        lastState_ = State::TrackSmolTag;
        if ((detections_(1)) != 1)
        {
            // if drone is not close to ground, switch
            if (dronePosition_.z() > 0.5)
            {
                SwitchState(State::LostTag);
            }
            break;  
        }

        // controller loop
        PIDLoop(tagSmol_);
        PubVelocityTarget();
        TagPoseLocal(tagSmol_); // as a backup for lost tag state
        lastAlt_ = dronePosition_.z(); // last alt since tag detection for lost tag state

        if (tagSmol_.position.z() < 0.1 && dronePosition_.z() < 0.5) // if drone is close to the ground
        {
            // best effort to center the drone on the tag
            if (std::abs(tagSmol_.position.x()) < 0.1 && std::abs(tagSmol_.position.y()) < 0.1){
                SwitchState(State::Landed);
                descentRate_ = 0.5;
            } else {
                descentRate_ = 0.01;
            }
        }
        
        break;
    case State::Landed:

        // cut all motors (send a disarm command)
        ROS_INFO("Landing...");
        PubPositionTarget(dronePosition_.x(), dronePosition_.y(), -5);

        // mavros_msgs::CommandLong disarm_cmd;
        // disarm_cmd.request.command = mavros_msgs::CommandLong::Request::MAV_CMD_COMPONENT_ARM_DISARM;
        // disarm_cmd.request.param1 = 0;      // Disarm (0 for disarm, 1 for arm)
        // disarm_cmd.request.param2 = 21196; // Force disarm code (bypass checks)

        // if (commandClient_.call(disarm_cmd) && disarm_cmd.response.success)
        // {
        //     ROS_INFO("Successfully landed");
        // }

    default:
        break;
    }   
}


void ApriltagLandingNode::TagPoseLocal(const Apriltag& tag)
{
    // convert tag pose from camera frame to world frame
    Eigen::Matrix3d R;
    R << 0, -1, 0,
         1, 0, 0,
         0, 0, 1;
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

void ApriltagLandingNode::PIDLoop(Apriltag curTag)
{
    ros::Time now = ros::Time::now();
    double dt =  (now - lastTime_).toSec();
    error_ = curTag.position;                       // try to achieve 0,0,0 distance with control loop
    ierror_ = error_ * dt;                          // i
    outputVel_ = ki_ * error_ + ki_ * ierror_;      // simple PI controller for position
    outputVel_.z() = descentRate_;                 // descend at set rate

    // write another pid loop for yaw rate
    outputYawRate_ = ki_ * curTag.position.z(); // simple P controller for yaw rate
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