#ifndef APRILTAG_LANDING_APRILTAG_LANDING_HPP
#define APRILTAG_LANDING_APRILTAG_LANDING_HPP

#include <ros/ros.h>
#include <mavros_msgs/LandingTarget.h>
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "geometry_msgs/PoseStamped.h"
#include <mavros_msgs/PositionTarget.h>
// include eigen
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <cmath>


class ApriltagLandingNode
{
public:
    // public functions
    void UpdateTarget(void);
    ApriltagLandingNode(ros::NodeHandle& nh);
    ~ApriltagLandingNode() = default; // default deconstructor

private:
    // private functions and member vars / private vars
    ros::Subscriber tagArraySub_;
    ros::Publisher localVelPub_;

    // ROS
    void PubLandingTarget(void);

    // state machine
    enum class State {
        NoTag,
        Approach,
        TrackBigTag,
        TrackSmolTag,
        Landed
    }
    std::string StateFb(State state);
    void SwitchState(State state);
    State state_ = State::NoTag;
    void TagPoseWorld(const apriltag_ros::AprilTagDetection& tag);

    // drone 
    void DronePoseCb(const geometry_msgs::PoseStamped& msg);
    Eigen::Vector3f _dronePose{0,0,0};
    float descentRate_{0.1}; // m/s

    // apriltag 
    void DetectionsCb(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);
    apriltag_ros::AprilTagDetection tagBig_;
    apriltag_ros::AprilTagDetection tagSmol_;
    Eigen::Vector3f tagPose_{0,0,0};
    float numDetections_{0};

    void ChooseTarget(void);

    // pid
    float _kp{1};
    float _ki{0.1};
    float _kd{0};
    float _sampleTime{1/30};
    Eigen::Vector3f _error{0,0,0};
    Eigen::Vector3f _ierror{0,0,0};
    Eigen::Vector3f _outputVel{0,0,0};
    float _outputYawRate{0};
    
    void PIDLoop(void);             // simple PD Controller


    // other helper funcs
    // Eigen::Vector3f Quat2EulerAngles(const Eigen::Quaternionf& q);
};

#endif
