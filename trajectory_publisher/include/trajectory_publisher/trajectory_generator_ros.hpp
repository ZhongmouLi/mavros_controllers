


#pragma once

#include <stdio.h>
#include <Eigen/Dense>
#include <cstdlib>
#include <sstream>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavconn/mavlink_dialect.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <yaml-cpp/yaml.h>
#include <array>
#include "controller_msgs/FlatTarget.h"
#include "trajectory_publisher/trajectory_generator.hpp"
#include "trajectory_publisher/base/common_ros.h"


class TrajectoryGeneratorROS {
private:
    // ROS node handles
    ros::NodeHandle nh_;             // Global (non-private) node handle
    ros::NodeHandle nh_private_;     // Private node handle (~ namespace)

    // ROS publishers
    ros::Publisher reference_pub_;        // Publishes geometry_msgs::TwistStamped with position + velocity
    ros::Publisher raw_reference_pub_;    // Publishes mavros_msgs::PositionTarget with full raw setpoint

    // ROS service server
    ros::ServiceServer start_service_;    // Service to start/stop trajectory generation

    // ROS subscribers
    ros::Subscriber takeoff_pose_sub_;    // Subscribes to takeoff pose (geometry_msgs::PoseStamped)

    // ROS timers
    ros::Timer loop_timer_;   // Timer for slower loop (e.g., visualization, updates)
    ros::Timer ref_timer_;    // Timer for fast loop (publishing references at high rate)

    // Trajectory yaml config file path
    std::string yaml_path_;

    // Trajectory generator
    std::shared_ptr<TrajectoryGenerator> ptr_traj_generator_;  // Smart pointer holding the trajectory generator

    // Current target state (calculated at each time step)
    Eigen::Vector3d p_targ_;  // Target position [x, y, z]
    Eigen::Vector3d v_targ_;  // Target velocity [vx, vy, vz]
    Eigen::Vector3d a_targ_;  // Target acceleration [ax, ay, az]

    // Circle trajectory configuration
    Eigen::Vector3d initial_post_{0,0,0};  // Center position of the circle
    Eigen::Vector3d axis_;    // Axis of rotation (normal vector)
    double radius_;           // Radius of the circle
    double omega_;            // Angular speed (rad/s)

    // // Polynomial trajectory configuration
    // Eigen::Vector3d target_post_{0,0,1}; // Target position for polynomial trajectory
    
    // double travelling_time_{1.0};        // Duration of the polynomial trajectory

    // Timing
    ros::Time start_time_;    // Time when the trajectory was started

    // Control flag
    bool is_active_ = false;        // Whether the system is currently active (publishing) or paused

    // initial position for trajectory flag
    bool is_initial_position_set_ = false;  // Flag to check if initial position is set

public:
    // Constructor: initializes publishers, services, timers, and generator
    TrajectoryGeneratorROS(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);


    void inputTrajectoryConfig();

    // Service callback: handles /start service requests to start or stop publishing
    bool startCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);


    // Callback for takeoff pose: sets the initial position for the trajectory generator
    void takeoffPoseCallback(const geometry_msgs::PoseStamped& msg);
    
    // Updates the current position, velocity, and acceleration based on elapsed time
    void updateReference();

    // Slow loop timer callback (optional use for visualization or future extensions)
    void loopCallback(const ros::TimerEvent&);

    // Fast loop timer callback: publishes reference states (position, velocity, acceleration)
    void refCallback(const ros::TimerEvent&);
};
