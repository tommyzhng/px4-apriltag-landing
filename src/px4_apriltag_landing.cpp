#include "px4_apriltag_landing.hpp"

ApriltagLandingNode::ApriltagLandingNode(ros::NodeHandle& nh)
{
    // initialize some stuff
    nh.param("/big_tag/kp", kp_, 0.1f);
    nh.param("/big_tag/ki", ki_, 0.1f);
    nh.param("/big_tag/sample_time", sampleTime_, 0.1f);
    nh.param("/apprThreshold", apprThreshold_, 3.0f);
    nh.param("/smolThreshold", smolThreshold_, 1.0f);
    nh.param("/apprDescentRate", apprDescentRate_, -0.5f);
    nh.param("/bigDescentRate", bigDescentRate_, -0.4f);
    nh.param("/smolDescentRate", smolDescentRate_, -0.1f);
    nh.param("/timeoutThreshold", timeoutThreshold_, 1.0f);

    ROS_INFO("kp: %f, ki: %f", kp_, ki_);
    
    tagArraySub_ = nh.subscribe("/tag_detections", 1, &ApriltagLandingNode::DetectionsCb, this);
    dronePoseSub_ = nh.subscribe("/mavros/global_position/local", 1, &ApriltagLandingNode::DronePoseCb, this);
    droneAbsPoseSub_ = nh.subscribe("/mavros/global_position/global", 1, &ApriltagLandingNode::DroneAbsPoseCb, this);
    droneHeadingSub_ = nh.subscribe("/mavros/global_position/compass_hdg", 1, &ApriltagLandingNode::DroneHeadingCb, this);
    commandClient_ = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
    localSetpointPub_ = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1);
    globalSetpointPub_ = nh.advertise<mavros_msgs::GlobalPositionTarget>("/mavros/setpoint_raw/global", 1);
    setParam_ = nh.serviceClient<mavros_msgs::ParamSet>("/mavros/param/set");
};

ApriltagLandingNode::~ApriltagLandingNode()
{
    CallParam("MPC_Z_VEL_MAX_DN", 2.0);
    CallParam("MPC_XY_VEL_MAX", 5.0);
}

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
            tagBig_.orientation = Eigen::Quaterniond(temp_quat.x, temp_quat.y, temp_quat.z, temp_quat.w);
            tagBig_.time = msg->detections[i].pose.header.stamp.toSec();
            detections_(0) = 1;
        }
        else if (msg->detections[i].id[0] == 1)
        {
            const auto& temp_pos = msg->detections[i].pose.pose.pose.position;
            const auto& temp_quat = msg->detections[i].pose.pose.pose.orientation;

            tagSmol_.position = Eigen::Vector3d(temp_pos.x, temp_pos.y, temp_pos.z);
            tagSmol_.orientation = Eigen::Quaterniond(temp_quat.x, temp_quat.y, temp_quat.z, temp_quat.w);
            tagSmol_.time = msg->detections[i].pose.header.stamp.toSec();
            detections_(1) = 1;
        }

    }
}

void ApriltagLandingNode::DronePoseCb(const nav_msgs::Odometry& msg)
{
    dronePosition_.x() = msg.pose.pose.position.x;
    dronePosition_.y() = msg.pose.pose.position.y;
    dronePosition_.z() = msg.pose.pose.position.z;
    droneOrientation_.w() = msg.pose.pose.orientation.w;
    droneOrientation_.x() = msg.pose.pose.orientation.x;
    droneOrientation_.y() = msg.pose.pose.orientation.y;
    droneOrientation_.z() = msg.pose.pose.orientation.z;
}

void ApriltagLandingNode::DroneAbsPoseCb(const sensor_msgs::NavSatFix& msg)
{
    dronePositionGlobal_.x() = msg.latitude;
    dronePositionGlobal_.y() = msg.longitude;
    dronePositionGlobal_.z() = msg.altitude; // this is a wrong altitude (did not subtract the geoid)

    lat_factor_ = 111320;
    long_factor_ = 40075000 * cos(dronePositionGlobal_.x() * M_PI / 180.0) / 360.0;

}

void ApriltagLandingNode::DroneHeadingCb(const std_msgs::Float64& msg)
{
    droneHeading_ = msg.data * M_PI / 180.0;
}

void ApriltagLandingNode::PubVelocityTarget(void)
{
    // publish the landing target
    mavros_msgs::PositionTarget msg;
    msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
    msg.type_mask = mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY | mavros_msgs::PositionTarget::IGNORE_PZ | mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ | mavros_msgs::PositionTarget::IGNORE_YAW;
    // print out the velocity
    msg.velocity.y = -outputVel_.x();
    msg.velocity.x = -outputVel_.y();
    msg.velocity.z = outputVel_.z();
    //msg.yaw_rate = outputYawRate_;
    localSetpointPub_.publish(msg);
}

void ApriltagLandingNode::PubPositionTarget(double x, double y, double z)
{
    // publish the landing target
    mavros_msgs::PositionTarget msg;
    msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    msg.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX | 
                    mavros_msgs::PositionTarget::IGNORE_AFY | 
                    mavros_msgs::PositionTarget::IGNORE_AFZ | 
                    mavros_msgs::PositionTarget::IGNORE_PZ |
                    mavros_msgs::PositionTarget::IGNORE_YAW;

    msg.position.x = x;
    msg.position.y = y;
    //msg.position.z = z;
    msg.velocity.z = outputVel_.z(); // descend at 0.1 m/s if detected
    //msg.yaw_rate = outputYawRate_;
    localSetpointPub_.publish(msg);
}

void ApriltagLandingNode::PubGlobalTarget(double lat, double lon, double z)
{
    // publish the landing target
    mavros_msgs::GlobalPositionTarget msg;
    msg.coordinate_frame = 6;
    msg.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX | 
                    mavros_msgs::PositionTarget::IGNORE_AFY | 
                    mavros_msgs::PositionTarget::IGNORE_AFZ | 
                    mavros_msgs::PositionTarget::IGNORE_VX |
                    mavros_msgs::PositionTarget::IGNORE_VY |
                    mavros_msgs::PositionTarget::IGNORE_VZ |
                    mavros_msgs::PositionTarget::IGNORE_YAW;

    msg.latitude = lat;
    msg.longitude = lon;
    msg.altitude = z;

    //msg.yaw_rate = outputYawRate_;
    globalSetpointPub_.publish(msg);
}

void ApriltagLandingNode::CallParam(const std::string& param_id, double value)
{
    mavros_msgs::ParamSet param;
    param.request.param_id = param_id;
    param.request.value.real = value;
    setParam_.call(param);
}

void ApriltagLandingNode::ArmDisarm(int arm)
{
    mavros_msgs::CommandLong arm_cmd;
    arm_cmd.request.command = 400; // MAV_CMD_COMPONENT_ARM_DISARM
    arm_cmd.request.param1 = arm;      // disarm (0 for disarm, 1 for arm)
    arm_cmd.request.param2 = 21196; // force (bypass checks)

    if (commandClient_.call(arm_cmd) && arm_cmd.response.success)
    {
        ROS_INFO("Successfully armed / disarmed");
    }
}

bool ApriltagLandingNode::TimeoutWatchdog(Apriltag curTag)
{
    // if no detections for 2 seconds, switch to no tag state
    if ((ros::Time::now() - ros::Time(curTag.time)).toSec() > timeoutThreshold_)
    {
        return true;
    }
    return false;
}

void ApriltagLandingNode::UpdateTarget(void)
{
    switch (state_)
    {
    case State::NoTag: {
        // if there is a detection for two loops
        if (detections_ != Eigen::Vector2i(0,0)) {
            SwitchState(State::Approach);
            lastState_ = State::Approach;
        }
        break;

    }

    case State::LostTag: {
    
        // if tag is lost, go to last known position
        PubGlobalTarget(globalTag_.position.x(), globalTag_.position.y(), lastAlt_);

        switch (lastState_)
        {
        case (State::TrackBigTag):
            if (detections_(0) == 1) {
                SwitchState(State::TrackBigTag);
                lastState_ = State::TrackBigTag;
                descentRate_ = bigDescentRate_;
                break;
            }
            break;
        case (State::TrackSmolTag):
            if (detections_(1) == 1) {
                SwitchState(State::TrackSmolTag);
                lastState_ = State::TrackSmolTag;
                descentRate_ = 0;
                break;
            }
            break;
        default:
            break;
        }   
        break;

    }

    case State::Approach: {

        // use position control to approach the tag
        ROS_INFO("current state: %s", StateFb(state_).c_str());
        lastAlt_ = dronePosition_.z(); // last alt since tag detection for lost tag state

        CallParam("MPC_Z_VEL_MAX_DN", abs((apprDescentRate_)));        // limit z velocity 
        TagPoseGlobal(tagBig_); // now globalTag_ is the bigTag_s position in NED
        PubGlobalTarget(globalTag_.position.x(), globalTag_.position.y(), apprThreshold_);

        if (dronePosition_.z() <= apprThreshold_+0.5 && detections_(0) == 1) { // if drone has crossed altitude boundary and has detected a big tag
            SwitchState(State::TrackBigTag);
            lastState_ = State::TrackBigTag;
            CallParam("MPC_Z_VEL_MAX_DN", 1.5);
            descentRate_ = bigDescentRate_;
        }

        // print position of global tag
        ROS_INFO("global tag position: %f, %f", globalTag_.position.x(), globalTag_.position.y());
        break;

    }

    case State::TrackBigTag: {

        // use velocity control to track the big tag with position as secondary source in LostTag state
        ROS_INFO("current state: %s", StateFb(state_).c_str());
        lastAlt_ = dronePosition_.z(); // last alt since tag detection for lost tag state
        bool lostTag = TimeoutWatchdog(tagBig_);
        if (lostTag == true) {
            SwitchState(State::LostTag);
            descentRate_ = 0.0;
            PubVelocityTarget();
            break;  
        }
        
        // controller loop
        PIDLoop(tagBig_);
        PubVelocityTarget();
        TagPoseGlobal(tagBig_); // as a backup for lost tag state


        if (dronePosition_.z() < smolThreshold_ && detections_(1) == 1) { // if drone has crossed altitude boundary and has detected a small tag
            SwitchState(State::TrackSmolTag);
            lastState_ = State::TrackSmolTag;
            kp_ = kp_ * 0.6;
            ki_ = ki_ * 0.6;
            descentRate_ = smolDescentRate_;
        }
        else if (dronePosition_.z() < smolThreshold_ && detections_(1) != 1) {
            descentRate_ = 0;
        }
        break;
    
    }

    case State::TrackSmolTag: {

        // use velocity control to track the small tag with position as secondary source in LostTag state
        ROS_INFO("current state: %s", StateFb(state_).c_str());
        lastAlt_ = dronePosition_.z(); // last alt since tag detection for lost tag state
        TagPoseGlobal(tagSmol_); // as a backup for lost tag state
        bool lostTagSmol = TimeoutWatchdog(tagSmol_);
        if (lostTagSmol == true) {
            // if drone is not close to ground, switch
            if (dronePosition_.z() > 1) {
                SwitchState(State::LostTag);
                descentRate_ = 0.0;
                PubVelocityTarget();
            }
            break;  
        }

        // controller loop
        PIDLoop(tagSmol_);
        PubVelocityTarget();

        if (dronePosition_.z() < 0.2) { // if drone is close to the ground
            // best effort to center the drone on the tag
            if (std::abs(tagSmol_.position.x()) < 0.1 && std::abs(tagSmol_.position.y()) < 0.1){
                SwitchState(State::Landed);
            } else {
                descentRate_ = 0.2;
            }  
        }
        break;
    
    }

    case State::Landed: {

        // cut all motors (send a disarm command)
        ROS_INFO("Landing...");
        ArmDisarm(0);
        PubVelocityTarget();
        //PubPositionTarget(dronePosition_.x(), dronePosition_.y(), -5);

        // mavros_msgs::CommandLong disarm_cmd;
        // disarm_cmd.request.command = mavros_msgs::CommandLong::Request::MAV_CMD_COMPONENT_ARM_DISARM;
        // disarm_cmd.request.param1 = 0;      // Disarm (0 for disarm, 1 for arm)
        // disarm_cmd.request.param2 = 21196; // Force disarm code (bypass checks)

        // if (commandClient_.call(disarm_cmd) && disarm_cmd.response.success)
        // {
        //     ROS_INFO("Successfully landed");
        // }

        // shutdown node
        ROS_INFO("Hope you landed on target :)");
        ros::shutdown();
        break;
    } 

    default:
        break;
    }   
}


void ApriltagLandingNode::TagPoseGlobal(const Apriltag& tag)
{
    // input tag has units meters
    // output in global long lat
    double dlong = (tag.position.x() * cos(droneHeading_) - tag.position.y() * sin(droneHeading_)) / long_factor_;
    double dlat = (-tag.position.y() * cos(droneHeading_) - tag.position.x() * sin(droneHeading_)) / lat_factor_;

    globalTag_.position.x() = dronePositionGlobal_.x() + dlat;
    globalTag_.position.y() = dronePositionGlobal_.y() + dlong;
}

void ApriltagLandingNode::PIDLoop(Apriltag curTag)
{
    ros::Time now = ros::Time::now();
    double dt =  (now - lastTime_).toSec();
    error_ = curTag.position;                       // try to achieve 0,0,0 distance with control loop
    ierror_ = error_ * dt;                          // i
    ierror_ = ierror_.cwiseMin(intLimit_).cwiseMax(-intLimit_); // clamp integral term

    outputVel_ = kp_ * error_ + ki_ * ierror_;      // simple PI controller for position
    outputVel_.z() = descentRate_;                 // descend at set rate


    // diagnostics
    ROS_INFO("kp: %f, ki: %f", kp_, ki_);
    ROS_INFO("descnet rate: %f", descentRate_);
    ROS_INFO("vel out: %f, %f, %f", outputVel_.x(), outputVel_.y(), outputVel_.z());


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
    case State::LostTag:
        return "LostTag";
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