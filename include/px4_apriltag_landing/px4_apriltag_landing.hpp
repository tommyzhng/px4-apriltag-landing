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
    // ROS
    void PubVelocityTarget(void);
    void PubPositionTarget(void);
    ros::Subscriber tagArraySub_;
    ros::Subscriber dronePoseSub_;
    ros::Publisher localVelPub_;

    // apriltag 
    struct Apriltag
    {
        Eigen::Vector3d position;
        Eigen::Quaterniond orientation{1,0,0,0};
    };
    void DetectionsCb(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);
    Apriltag tagBig_;
    Apriltag tagSmol_;
    Apriltag curTag_;
    Apriltag localTag_;
    Eigen::Vector2i detections_{0,0};

    // drone 
    void DronePoseCb(const geometry_msgs::PoseStamped& msg);
    Eigen::Vector3d dronePosition_{0,0,0};
    Eigen::Quaterniond droneOrientation_{1,0,0,0};
    float descentRate_{0.1}; // m/s


    // state machine
    enum class State {
        NoTag,
        Approach,
        TrackBigTag,
        TrackSmolTag,
        Landed
    };
    std::string StateFb(State state);
    void SwitchState(State state);
    void TagPoseLocal(const Apriltag& tag);
    State state_ = State::NoTag;
    float altThreshold_ = 2.0;

    // pid
     void PIDLoop(void);             // simple PD Controller
    float kp_{1};
    float ki_{0.1};
    float kd_{0};
    float sampleTime_{1/30};
    ros::Time lastTime_;
    Eigen::Vector3d error_{0,0,0};
    Eigen::Vector3d ierror_{0,0,0};
    Eigen::Vector3d outputVel_{0,0,0};
    float outputYawRate_{0};
    


    // other helper funcs
    // Eigen::Vector3d Quat2EulerAngles(const Eigen::Quaterniond& q);
};

#endif
