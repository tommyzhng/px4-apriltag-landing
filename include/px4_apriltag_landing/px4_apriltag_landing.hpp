#ifndef APRILTAG_LANDING_APRILTAG_LANDING_HPP
#define APRILTAG_LANDING_APRILTAG_LANDING_HPP

#include <ros/ros.h>
#include <mavros_msgs/LandingTarget.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/CommandLong.h>
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/NavSatFix.h"

#include "apriltag_ros/AprilTagDetectionArray.h"


// include eigen
#include <eigen3/Eigen/Dense>
#include <math.h>

class ApriltagLandingNode
{
public:
    // public functions
    void UpdateTarget(void);
    ApriltagLandingNode(ros::NodeHandle& nh);
    ~ApriltagLandingNode(); 

private:
    // ROS subs and pubs
    ros::Subscriber tagArraySub_;
    ros::Subscriber dronePoseSub_;
    ros::Subscriber droneAbsPoseSub_;
    ros::Subscriber droneHeadingSub_;
    ros::Publisher localSetpointPub_;
    ros::Publisher globalSetpointPub_;
    ros::ServiceClient commandClient_;
    ros::ServiceClient setParam_;

    // apriltag 
    struct Apriltag
    {
        Eigen::Vector3d position;
        Eigen::Quaterniond orientation{1,0,0,0};
        float time;
    };
    void DetectionsCb(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);
    bool TimeoutWatchdog(Apriltag curTag);
    Apriltag tagBig_;
    Apriltag tagSmol_;
    Apriltag globalTag_;
    Eigen::Vector2i detections_{0,0};
    float timeoutThreshold_{1.0};

    // drone 
    void DronePoseCb(const nav_msgs::Odometry& msg);
    void DroneAbsPoseCb(const sensor_msgs::NavSatFix& msg);
    void DroneHeadingCb(const std_msgs::Float64& msg);
    void PubVelocityTarget(void);
    void PubPositionTarget(double x, double y, double z);
    void PubGlobalTarget(double lat, double lon, double z);
    void CallParam(const std::string& param_id, double value);
    void ArmDisarm(int arm);
    Eigen::Vector3d dronePosition_{0,0,0};
    Eigen::Vector3d dronePositionGlobal_{0,0,0};
    Eigen::Quaterniond droneOrientation_{1,0,0,0};
    double droneHeading_{0};
    int lat_factor_{111320};
    int long_factor_{40075000};


    // state machine
    enum class State {
        NoTag,
        LostTag,
        Approach,
        TrackBigTag,
        TrackSmolTag,
        Landed
    };
    std::string StateFb(State state);
    void SwitchState(State state);
    void TagPoseGlobal(const Apriltag& tag);
    State state_ = State::NoTag;
    State lastState_ = State::NoTag;
    float apprThreshold_ = 3.0;
    float smolThreshold_ = 1.0;

    // pid
    void PIDLoop(Apriltag curTag);             // simple PD Controller
    float kp_{0.0} , ki_{0.0}, kd_{0.0};
    float kp1_{0.0}, ki1_{0.0}, kd1_{0.0};

    float sampleTime_{1/30};
    ros::Time lastTime_;
    Eigen::Vector3d error_{0,0,0};
    Eigen::Vector3d ierror_{0,0,0};
    Eigen::Vector3d derror_{0,0,0};
    Eigen::Vector3d lastError_{0,0,0};
    Eigen::Vector3d outputVel_{0,0,0};
    Eigen::Vector3d intLimit_{0.8,0.8,0.8};
    
    float lastAlt_{0};
    float apprDescentRate_{-0.5}; // m/s
    float bigDescentRate_{-0.4}; // m/s
    float smolDescentRate_{-0.1}; // m/s
    float descentRate_{0}; // m/s
    float outputYawRate_{0};
    


    // other helper funcs
    // Eigen::Vector3d Quat2EulerAngles(const Eigen::Quaterniond& q);
};

#endif
