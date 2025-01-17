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
    ~ApriltagLandingNode(); // default deconstructor

private:
    // private functions and member vars / private vars
    ros::Subscriber tagArraySub;
    ros::Publisher localVelPub;

    Eigen::Vector3f Quat2EulerAngles(const Eigen::Quaternionf& q);
    void DetectionsCb(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);
    void PubLandingTarget(void);
    void ChooseTarget(void);
    void PIDLoop(void);             // simple PD Controller

    // vars to store
    apriltag_ros::AprilTagDetection tagBig;
    apriltag_ros::AprilTagDetection tagSmol;
    Eigen::Vector3f _tagPose{0,0,0};
    float _tagOrientationEuler{0};
    float _descentRate{0.1}; // m/s

    // pid params
    float _kp{0.1};
    float _ki{0.1};
    float _kd{0.1};
    float _sampleTime{0.1};
    ros::Time _lastTime{0};
    Eigen::Vector3f _error{0,0,0};
    Eigen::Vector3f _derror{0,0,0};
    Eigen::Vector3f _lastError{0,0,0};
    Eigen::Vector3f _outputVel{0,0,0};
    float _outputYawRate{0};
};

#endif
