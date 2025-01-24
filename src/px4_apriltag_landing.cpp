#include "px4_apriltag_landing.hpp"

ApriltagLandingNode::ApriltagLandingNode(ros::NodeHandle& nh)
{
    // initialize some stuff
    nh.param("/big_tag/kp", kp_, 0.1f);
    nh.param("/big_tag/ki", ki_, 0.1f);
    nh.param("/smol_tag/kp1", kp1_, 0.1f);
    nh.param("/smol_tag/ki1", ki1_, 0.1f);
    nh.param("/big_tag/sample_time", sampleTime_, 0.1f);
    nh.param("/apprThreshold", apprThreshold_, 3.0f);
    nh.param("/smolThreshold", smolThreshold_, 1.0f);
    nh.param("/apprDescentRate", apprDescentRate_, -0.5f);
    nh.param("/bigDescentRate", bigDescentRate_, -0.4f);
    nh.param("/smolDescentRate", smolDescentRate_, -0.1f);

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
            detections_(0) = 1;
        }
        else if (msg->detections[i].id[0] == 1)
        {
            const auto& temp_pos = msg->detections[i].pose.pose.pose.position;
            const auto& temp_quat = msg->detections[i].pose.pose.pose.orientation;

            tagSmol_.position = Eigen::Vector3d(temp_pos.x, temp_pos.y, temp_pos.z);
            tagSmol_.orientation = Eigen::Quaterniond(temp_quat.x, temp_quat.y, temp_quat.z, temp_quat.w);
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
    msg.type_mask = 0b110111000000;

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

void ApriltagLandingNode::UpdateTarget(void)
{
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
        PubGlobalTarget(globalTag_.position.x(), globalTag_.position.y(), lastAlt_);

        switch (lastState_)
        {
        case (State::TrackBigTag):
            if (detections_(0) == 1)
            {
                SwitchState(State::TrackBigTag);
                descentRate_ = bigDescentRate_;
                break;
            }
            break;
        case (State::TrackSmolTag):
            if (detections_(1) == 1)
            {
                SwitchState(State::TrackSmolTag);
                descentRate_ = 0;
                break;
            }
            break;
        default:
            break;
        }   
        break;

    case State::Approach:

        // use position control to approach the tag
        // approach the big tag first
        ROS_INFO("current state: %s", StateFb(state_).c_str());
        // limit z velocity 
        CallParam("MPC_Z_VEL_MAX_DN", abs((apprDescentRate_)));
        CallParam("MPC_JERK_MAX", 1.0);

        lastState_ = State::Approach;
        if (detections_(0) == 1)
        {
            TagPoseGlobal(tagBig_); // now globalTag_ is the bigTag_s position in NED
            lastAlt_ = dronePosition_.z(); // last alt since tag detection for lost tag state
            outputVel_.z() = apprDescentRate_;
        }
        // still approach the tag if its lost
        // print position of global tag
        ROS_INFO("global tag position: %f, %f", globalTag_.position.x(), globalTag_.position.y());
        PubGlobalTarget(globalTag_.position.x(), globalTag_.position.y(), apprThreshold_);
        if (dronePosition_.z() <= apprThreshold_+0.5 && detections_(0) == 1) // if drone has crossed altitude boundary and has detected a big tag
        {
            SwitchState(State::TrackBigTag);
            CallParam("MPC_Z_VEL_MAX_DN", 2.0);
            
            descentRate_ = bigDescentRate_;
        }
        // if it has crossed altitude boundary but not detected a big tag
        else if (dronePosition_.z() <= apprThreshold_+0.5 && detections_(0) != 1)
        {
            // stop descending
            CallParam("MPC_Z_VEL_MAX_DN", 0);
        }

        break;

    case State::TrackBigTag:

        // use velocity control to track the big tag with position as secondary source in LostTag state
        ROS_INFO("current state: %s", StateFb(state_).c_str());
        lastState_ = State::TrackBigTag;
        if ((detections_(0)) != 1)
        {
            SwitchState(State::LostTag);
            descentRate_ = 0.0;
            PubVelocityTarget();
            break;  
        }
        
        // controller loop
        PIDLoop(tagBig_);
        PubVelocityTarget();
        TagPoseGlobal(tagBig_); // as a backup for lost tag state
        lastAlt_ = dronePosition_.z(); // last alt since tag detection for lost tag state

        if (dronePosition_.z() < smolThreshold_ && detections_(1) == 1) // if drone has crossed altitude boundary and has detected a small tag
        {
            SwitchState(State::TrackSmolTag);
            // turn gains down (very professional adaptive controller T-T)
            kp_ = kp_ ;
            ki_ = ki1_;
            descentRate_ = smolDescentRate_;
        }
        else if (dronePosition_.z() < smolThreshold_ && detections_(1) != 1)
        {
            descentRate_ = 0;
        }
        break;

    case State::TrackSmolTag:

        // use velocity control to track the small tag with position as secondary source in LostTag state
        ROS_INFO("current state: %s", StateFb(state_).c_str());
        lastState_ = State::TrackSmolTag;
        if ((detections_(1)) != 1)
        {
            // if drone is not close to ground, switch
            if (dronePosition_.z() > 0.5)
            {
                SwitchState(State::LostTag);
                descentRate_ = 0.0;
                PubVelocityTarget();
            }
            break;  
        }

        // controller loop
        PIDLoop(tagSmol_);
        PubVelocityTarget();
        TagPoseGlobal(tagSmol_); // as a backup for lost tag state
        lastAlt_ = dronePosition_.z(); // last alt since tag detection for lost tag state

        if (dronePosition_.z() < 0.2) // if drone is close to the ground
        {
            // best effort to center the drone on the tag
            if (std::abs(tagSmol_.position.x()) < 0.1 && std::abs(tagSmol_.position.y()) < 0.1){
                SwitchState(State::Landed);
            } else {
                descentRate_ = 0.2;
            }
            
        }
        
        break;
    case State::Landed:

        // cut all motors (send a disarm command)
        ROS_INFO("Landing...");
        CallParam("MPC_JERK_MAX", 5.0);
        outputVel_.z() = -5;
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
    outputVel_ = kp_ * error_ + ki_ * ierror_;      // simple PI controller for position
    outputVel_.z() = descentRate_;                 // descend at set rate
    ROS_INFO("descnet rate: %f", descentRate_);
    ROS_INFO("error: %f, %f, %f", error_.x(), error_.y(), error_.z());
    // gains
    ROS_INFO("kp: %f, ki: %f", kp_, ki_);

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