// geom_control_ros.hpp
#pragma once

#include "geometric_controller/geom_control_base.hpp"
#include "geometric_controller/common_ros.hpp"
#include "geometric_controller/lib_gain_learning.hpp"
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CompanionProcessStatus.h>
#include <mavros_msgs/SetMode.h>
#include <nav_msgs/Path.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Float32.h>
#include <vector>

/**
 * @brief ROS wrapper class for geomControlBase
 * 
 * It handles all ROS publishers, subscribers, services, and timers.
 */
class adaptiveControlROS : public geomControlBase
{
    
private:
    // ------------------ ROS Node Handles ------------------

    ros::NodeHandle nh_;          ///< Public node handle
    ros::NodeHandle nh_private_;  ///< Private node handle

    // ------------------ Subscribers ------------------

    ros::Subscriber referenceSub_;      ///< Subscribe to target pose & velocity, calls targetCallback()
    ros::Subscriber yawreferenceSub_;   ///< Subscribe to target yaw, calls yawtargetCallback()
    ros::Subscriber mavstateSub_;        ///< Subscribe to MAVROS state, calls mavstateCallback()
    ros::Subscriber mavposeSub_;         ///< Subscribe to MAV pose (vicon/mavros), calls mavposeCallback()
    ros::Subscriber mavtwistSub_;        ///< Subscribe to MAV velocity, calls mavtwistCallback()

    // ------------------ Publishers ------------------

    ros::Publisher angularVelPub_;       ///< Publish body rate and thrust commands (command/bodyrate_command)
    ros::Publisher referencePosePub_;    ///< Publish reference pose for visualization (reference/pose)
    ros::Publisher target_pose_pub_;     ///< Publish target position to MAVROS (mavros/setpoint_position/local)
    ros::Publisher posehistoryPub_;      ///< Publish path history (geometric_controller/path)
    ros::Publisher systemstatusPub_;     ///< Publish system status (mavros/companion_process/status)

    // ------------------ Service Clients ------------------

    ros::ServiceClient arming_client_;   ///< Client to arm the drone (mavros/cmd/arming)
    ros::ServiceClient set_mode_client_; ///< Client to set flight mode (mavros/set_mode)

    // ------------------ Service Servers ------------------

    ros::ServiceServer ctrltriggerServ_; ///< Service to trigger controller (trigger_rlcontroller)
    ros::ServiceServer land_service_;    ///< Service to command landing (land)

    // ------------------ Timers ------------------

    ros::Timer cmdloop_timer_;            ///< Timer for command loop (10ms rate)
    ros::Timer statusloop_timer_;         ///< Timer for status loop (1s rate)

    // ------------------ Other Members ------------------

    mavros_msgs::State current_state_;    ///< Current MAVROS state
    mavros_msgs::CommandBool arm_cmd_;    ///< Arming service request
    mavros_msgs::SetMode offb_set_mode_;  ///< Set mode service request
    ros::Time last_request_;              ///< Last service request time
    ros::Time reference_request_now_;     ///< Latest target update time
    ros::Time reference_request_last_;    ///< Previous target update time

    ros::Time take_off_now_;           ///< Current time for takeoof
    ros::Time take_off_begin_;           ///< Current time for takeoof

    ros::Time pre_takeoff_now_;           ///< Current time for pre-takeoff
    ros::Time pre_takeoff_begin_;           ///< Current time for pre-takeoff


    bool take_off_flag_ = false;         ///< Flag to indicate if takeoff is in progress
    bool pre_takeoff_flag_ = false;         ///< Flag to indicate if pre-takeoff is in progress


    bool mission_flag_ = false;         ///< Flag to indicate if takeoff is in progress

    ros::Time mission_now_;           ///< Current time for pre-takeoff
    ros::Time mission_begin_;           ///< Current time for pre-takeoff
        
    
    ros::Time mavpose_receive_last_;    ///< Previous target update time

    std::vector<geometry_msgs::PoseStamped> posehistory_vector_; ///< History of poses for trajectory visualization

    // ------------------ Control Mode ------------------
    bool sim_enable_ = true; ///< Flag to indicate if in simulation mode

    // ------------------ Control Methods ------------------

    // adaptive gain learning
    std::shared_ptr<AdaptiveGain> ptr_adaptive_gain = nullptr;

    /**
     * @brief Publish reference pose for visualization
     */
    void pubReferencePose(const Eigen::Vector3d& target_position, const Eigen::Vector4d& target_attitude);

    /**
     * @brief Publish body rate commands to the flight controller
     */
    void pubControlCommands(const Eigen::Vector4d& cmd, const Eigen::Vector4d& target_attitude);

    /**
     * @brief Update and publish pose history for trajectory visualization
     */
    void updateAndPublishPoseHistory();

    /**
     * @brief publish system status
     */    
    
    void pubSystemStatus();

    void pubTargetPose2PX4Controller(const Eigen::Vector3d& target_position);




public:
    /**
     * @brief Constructor
     * @param nh ROS public node handle
     * @param nh_private ROS private node handle
     */
    adaptiveControlROS(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

    /**
     * @brief Destructor
     */
    ~adaptiveControlROS();

    // ------------------ ROS Subscriber Callbacks ------------------

    /**
     * @brief Callback for receiving target position and velocity setpoints (reference/setpoint)
     */
    void targetCallback(const geometry_msgs::TwistStamped& msg);

    /**
     * @brief Callback for receiving yaw target (reference/yaw)
     */
    void yawtargetCallback(const std_msgs::Float32& msg);

    /**
     * @brief Callback for receiving MAV pose (vicon/drone or mavros/local_position/pose)
     */
    // void mavposeCallback(const geometry_msgs::TransformStamped::ConstPtr& msg_vicon);
    void mavposeCallback(const geometry_msgs::PoseStamped &msg);

    /**
     * @brief Callback for receiving MAV twist (mavros/local_position/velocity_local)
     */
    void mavtwistCallback(const geometry_msgs::TwistStamped& msg);

    /**
     * @brief Callback for receiving MAV state (mavros/state)
     */
    void mavstateCallback(const mavros_msgs::State::ConstPtr& msg);

    // ------------------ ROS Service Callbacks ------------------

    /**
     * @brief Service callback for switching control mode (trigger_rlcontroller)
     */
    bool ctrltriggerCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

    /**
     * @brief Service callback to trigger landing (land)
     */
    bool landCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

    // ------------------ ROS Timer Callbacks ------------------

    /**
     * @brief Main control command loop timer callback (0.01s)
     */
    void cmdloopCallback(const ros::TimerEvent& event);

    /**
     * @brief System status monitoring timer callback (1s)
     */
    void statusloopCallback(const ros::TimerEvent& event);


    // ------------------ adaptive gain update ------------------
    bool updatePostControlGain();
};