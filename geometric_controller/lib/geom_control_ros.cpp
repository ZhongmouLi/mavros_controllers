#include "geometric_controller/geom_control_ros.hpp"
#include <algorithm>
#include <fstream>

geomControlROS::geomControlROS(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : geomControlBase(), // Call the base class constructor
      nh_(nh), 
      nh_private_(nh_private)
{
    // ------------------ Setup Subscribers ------------------
    referenceSub_ = nh_.subscribe("reference/setpoint", 1, &geomControlROS::targetCallback, this, ros::TransportHints().tcpNoDelay());
    yawreferenceSub_ = nh_.subscribe("reference/yaw", 1, &geomControlROS::yawtargetCallback, this, ros::TransportHints().tcpNoDelay());
    mavstateSub_ = nh_.subscribe("mavros/state", 1, &geomControlROS::mavstateCallback, this, ros::TransportHints().tcpNoDelay());
    mavVICONposeSub_ = nh_.subscribe("vicon/drone", 1, &geomControlROS::mavVICONposeCallback, this, ros::TransportHints().tcpNoDelay());
    mavGPSposeSub_ = nh_.subscribe("mavros/local_position/pose", 1, &geomControlROS::mavGPSposeCallback, this, ros::TransportHints().tcpNoDelay());
    mavtwistSub_ = nh_.subscribe("mavros/local_position/velocity_local", 1, &geomControlROS::mavtwistCallback, this, ros::TransportHints().tcpNoDelay());

    // ------------------ Setup Publishers ------------------
    angularVelPub_ = nh_.advertise<mavros_msgs::AttitudeTarget>("command/bodyrate_command", 1);
    referencePosePub_ = nh_.advertise<geometry_msgs::PoseStamped>("reference/pose", 1);
    // to remove
    target_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    posehistoryPub_ = nh_.advertise<nav_msgs::Path>("geometric_controller/path", 10);
    systemstatusPub_ = nh_.advertise<mavros_msgs::CompanionProcessStatus>("mavros/companion_process/status", 1);

    // ------------------ Setup Services ------------------
    ctrltriggerServ_ = nh_.advertiseService("trigger_rlcontroller", &geomControlROS::ctrltriggerCallback, this);
    land_service_ = nh_.advertiseService("land", &geomControlROS::landCallback, this);

    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    // ------------------ Setup Timers ------------------
    cmdloop_timer_ = nh_.createTimer(ros::Duration(0.01), &geomControlROS::cmdloopCallback, this);
    statusloop_timer_ = nh_.createTimer(ros::Duration(1.0), &geomControlROS::statusloopCallback, this);

    // Initialize time variables
    last_request_ = ros::Time::now();
    reference_request_now_ = ros::Time::now();
    reference_request_last_ = ros::Time::now();

    mavpose_receive_last_ = ros::Time::now();


    // ------------------ Load Parameters ------------------
    
    // System identification
    std::string mav_name;
    nh_private_.param<std::string>("mavname", mav_name, "iris");

    nh_private_.param<bool>("use_vicon", use_vicon_, true);

    nh_private_.param<bool>("use_gps", use_gps_, false);

    // Controller mode
    int ctrl_mode;
    nh_private_.param<int>("ctrl_mode", ctrl_mode, ERROR_QUATERNION);
    setControlMode(ctrl_mode);
    
    // Simulation and yaw mode
    nh_private_.param<bool>("enable_sim", sim_enable_, false);
    
    bool velocity_yaw;
    nh_private_.param<bool>("velocity_yaw", velocity_yaw, false);
    setVelocityYawMode(velocity_yaw);
    
    // Maximum acceleration
    double max_fb_acc;
    nh_private_.param<double>("max_acc", max_fb_acc, 9.0);
    setMaxFeedbackAcceleration(max_fb_acc);
    
    // Initial yaw heading
    double yaw_heading;
    nh_private_.param<double>("yaw_heading", yaw_heading, 0.0);
    inputTargetYawAngle(yaw_heading);
    
    // Drag coefficients
    double dx, dy, dz;
    nh_private_.param<double>("drag_dx", dx, 0.0);
    nh_private_.param<double>("drag_dy", dy, 0.0);
    nh_private_.param<double>("drag_dz", dz, 0.0);
    Eigen::Vector3d drag_coeffs(dx, dy, dz);
    setDragCoefficients(drag_coeffs);
    
    // Attitude controller parameters
    double attctrl_constant;
    nh_private_.param<double>("attctrl_constant", attctrl_constant, 0.1);
    setAttitudeControllerGain(attctrl_constant);
    
    // Thrust mapping parameters
    double norm_thrust_const, norm_thrust_offset;
    nh_private_.param<double>("normalizedthrust_constant", norm_thrust_const, 0.05);
    nh_private_.param<double>("normalizedthrust_offset", norm_thrust_offset, 0.1);
    setThrustParameters(norm_thrust_const, norm_thrust_offset);
    
    // Position controller gains
    double Kp_x, Kp_y, Kp_z;
    nh_private_.param<double>("Kp_x", Kp_x, 8.0);
    nh_private_.param<double>("Kp_y", Kp_y, 8.0);
    nh_private_.param<double>("Kp_z", Kp_z, 10.0);
    
    // Velocity controller gains
    double Kv_x, Kv_y, Kv_z;
    nh_private_.param<double>("Kv_x", Kv_x, 1.5);
    nh_private_.param<double>("Kv_y", Kv_y, 1.5);
    nh_private_.param<double>("Kv_z", Kv_z, 3.3);
    
    // Position integral controller gains
    double KposI_x, KposI_y, KposI_z;
    nh_private_.param<double>("KposI_x", KposI_x, 0.1);
    nh_private_.param<double>("KposI_y", KposI_y, 0.1);
    nh_private_.param<double>("KposI_z", KposI_z, 0.1);
    
    // Set control gains (note the negative signs as in original code)
    Eigen::Vector3d Kpos(-Kp_x, -Kp_y, -Kp_z);
    Eigen::Vector3d Kvel(-Kv_x, -Kv_y, -Kv_z);
    Eigen::Vector3d KposI(-KposI_x, -KposI_y, -KposI_z);
    
    setPostControlPGains(Kpos);
    setPostControlDGains(Kvel);
    setPostControlIGains(KposI);
    
    // Initial target position
    double init_pos_x, init_pos_y, init_pos_z;
    nh_private_.param<double>("init_pos_x", init_pos_x, 0.0);
    nh_private_.param<double>("init_pos_y", init_pos_y, 0.0);
    nh_private_.param<double>("init_pos_z", init_pos_z, 2.0);
    
    // Set initial target position and zero velocity/acceleration
    Eigen::Vector3d init_pos(init_pos_x, init_pos_y, init_pos_z);
    Eigen::Vector3d zero_vel(0.0, 0.0, 0.0);
    Eigen::Vector3d zero_acc(0.0, 0.0, 0.0);
    inputTargetPositionVelAcc(init_pos, zero_vel, zero_acc);
    
    // Pose history window
    int posehistory_window;
    nh_private_.param<int>("posehistory_window", posehistory_window, 200);
    posehistory_vector_.reserve(posehistory_window);

    // Set takeoff height and time
    double takeoff_height;
    nh_private_.param<double>("takeoff_height", takeoff_height, 1.0);
    double takeoff_time;
    nh_private_.param<double>("takeoff_time", takeoff_time, 5.0);

    setTakeoffHeightAndTime(takeoff_height, takeoff_time);

}

geomControlROS::~geomControlROS()
{
    // Destructor
}

void geomControlROS::targetCallback(const geometry_msgs::TwistStamped& msg)
{
    reference_request_last_ = reference_request_now_;
    reference_request_now_ = ros::Time::now();
    
    Eigen::Vector3d target_pos = toEigen(msg.twist.angular);
    Eigen::Vector3d target_vel = toEigen(msg.twist.linear);
    Eigen::Vector3d target_acc = Eigen::Vector3d::Zero();
    
    // Use accessor methods to get current target position and velocity
    Eigen::Vector3d prev_pos = targetPosition();
    Eigen::Vector3d prev_vel = targetVelocity();
    
    // Update position, velocity, acceleration
    inputTargetPositionVelAcc(target_pos, target_vel, target_acc);
    updatePreviousTargetPositionVelAcc(prev_pos, prev_vel);
}

void geomControlROS::yawtargetCallback(const std_msgs::Float32& msg)
{
    inputTargetYawAngle(static_cast<double>(msg.data));
}

void geomControlROS::mavstateCallback(const mavros_msgs::State::ConstPtr& msg)
{
    current_state_ = *msg;
    
    // Update flight state based on MAVROS state
    if (current_state_.armed) {
        setFlightArmingState(FlightArmingState::ARMED);
        if (current_state_.mode == "OFFBOARD") {
            setFlightOffboardState(FlightOffboardState::OFFBOARD_ENABLED);
        } else {
            setFlightOffboardState(FlightOffboardState::OFFBOARD_DISABLED);
        }
    } else {
        setFlightArmingState(FlightArmingState::DISARMED);
        setFlightOffboardState(FlightOffboardState::OFFBOARD_DISABLED);
    };


    if (current_state_.armed == true) 
    {
        ROS_INFO_STREAM_THROTTLE(5.0, "Armed state: True ");
    }
    else
    {
        ROS_INFO_STREAM_THROTTLE(5.0, "Armed state: False ");
    };

    ROS_INFO_STREAM_THROTTLE(5.0, "Mode state "  << current_state_.mode );
}

// void geometricCtrl::mavposeCallback(const geometry_msgs::PoseStamped &msg) {
//     if (!received_home_pose) {
//       received_home_pose = true;
//       home_pose_ = msg.pose;
//       ROS_INFO_STREAM("Home pose initialized to: " << home_pose_);
//     }
//     mavPos_ = toEigen(msg.pose.position);
//     mavAtt_(0) = msg.pose.orientation.w;
//     mavAtt_(1) = msg.pose.orientation.x;
//     mavAtt_(2) = msg.pose.orientation.y;
//     mavAtt_(3) = msg.pose.orientation.z;
//   }

void geomControlROS::mavGPSposeCallback(const geometry_msgs::PoseStamped &msg)
{
    if (!use_gps_) return;    // Ignore GPS pose if not enabled


    if ((ros::Time::now() - mavpose_receive_last_).toSec() > 2.0) {  // e.g., 1 second without pose
        ROS_WARN_STREAM_THROTTLE(2.0, "No drone pose is received yet");
    }
    // Directly use pose from PoseStamped message
    geometry_msgs::Pose pose = msg.pose;

    // Check and set home position if not set
    if (!isHomePositionSet()) {
        setHomePosition(toEigen(pose.position));
        ROS_INFO_STREAM("Home pose initialized to: " << toEigen(pose.position).transpose());
    }

    // Update current MAV position and attitude
    updateMavPositionAttitude(toEigen(pose.position), toEigen(pose.orientation));

    mavpose_receive_last_ = ros::Time::now();  // Update last received time

    // Add current pose to history vector
    // geometry_msgs::PoseStamped pose_stamped = msg;
    // pose_stamped.header.stamp = ros::Time::now();
    // pose_stamped.header.frame_id = "map";  // Ensure consistent frame, if needed

    // posehistory_vector_.insert(posehistory_vector_.begin(), pose_stamped);
    // if (posehistory_vector_.size() > 200) { // Limit history size
    //     posehistory_vector_.pop_back();
    // }

    ROS_INFO_STREAM_THROTTLE(5.0, "Drone pose is from GPS");
}



void geomControlROS::mavVICONposeCallback(const geometry_msgs::TransformStamped::ConstPtr& msg_vicon)
{

    if (!use_vicon_) return;    // Ignore VICON pose if not enabled

    if ((ros::Time::now() - mavpose_receive_last_).toSec() > 2.0) {  // e.g., 1 second without pose
        ROS_WARN_STREAM_THROTTLE(2.0, "No drone pose is received yet");
    }

    geometry_msgs::Pose pose;
    pose.position.x = msg_vicon->transform.translation.x;
    pose.position.y = msg_vicon->transform.translation.y;
    pose.position.z = msg_vicon->transform.translation.z;
    pose.orientation = msg_vicon->transform.rotation;

    if (!isHomePositionSet()) {
        setHomePosition(toEigen(pose.position));
        ROS_INFO_STREAM("Home pose initialized to: " << toEigen(pose.position).transpose());
    }

    updateMavPositionAttitude(toEigen(pose.position), toEigen(pose.orientation));
    
 
    mavpose_receive_last_ = ros::Time::now();  // Update last received time

    ROS_INFO_STREAM_THROTTLE(5.0, "Drone pose is from VICON");
}


void geomControlROS::mavtwistCallback(const geometry_msgs::TwistStamped& msg)
{
    updateMavVelRate(toEigen(msg.twist.linear), toEigen(msg.twist.angular));
}


bool geomControlROS::ctrltriggerCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
    // Use the setter method to update the control mode
    setControlMode(req.data ? ERROR_GEOMETRIC : ERROR_QUATERNION);
    
    res.success = true;
    res.message = "Controller mode switched";
    return true;
}

bool geomControlROS::landCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
    setMissionState(MissionState::LANDING);
    res.success = true;
    res.message = "Landing initiated";
    return true;
}

void geomControlROS::cmdloopCallback(const ros::TimerEvent& event)
{
    switch (mission_state_) {

    case MissionState::ARM_OFFBOARD: {
        // Check if home position is set, vehicle is armed, and OFFBOARD mode is enabled
        if (!isHomePositionSet()) {
            ROS_INFO_THROTTLE(2.0, "ARM_OFFBOARD: waiting for home position...");
            break;
        }

        // in simulation, we sent target position to the PX4 controller
        pubTargetPose2PX4Controller(homePosition());

        // set the mission state to PRE_TAKEOFF
        if (flight_arming_state_ == FlightArmingState::DISARMED || flight_offboard_state_ == FlightOffboardState::OFFBOARD_DISABLED) {
            ROS_INFO_THROTTLE(2.0, "ARM_OFFBOARD: waiting for Arm and Offboard...");
        }
        else {
            ROS_INFO("ARM_OFFBOARD to PRE_TAKEOFF");
            setMissionState(MissionState::PRE_TAKEOFF);
        }

        break;

    }    

    case MissionState::PRE_TAKEOFF:{


        if (flight_arming_state_ == FlightArmingState::DISARMED) {
            ROS_INFO_THROTTLE(1.0, "PRE_TAKEOFF: waiting for vehicle to be armed...");
            break;
        }
        if (flight_offboard_state_ == FlightOffboardState::OFFBOARD_DISABLED) {
            ROS_INFO_THROTTLE(1.0, "PRE_TAKEOFF: waiting for OFFBOARD mode...");
            break;
        }

        // get the current time

        if(!pre_takeoff_flag_) {
            pre_takeoff_begin_ = ros::Time::now();
            pre_takeoff_flag_ = true;
            ROS_INFO("PRE_TAKEOFF initiated");
        } 

        // Compute the time since pre-takeoff began
        pre_takeoff_now_ = ros::Time::now();
        double pre_takeoff_current_step = (pre_takeoff_now_ - pre_takeoff_begin_).toSec();


        // do pre-takeoff preparation if pre_takeoff_current_step is less than 5 seconds
        if (pre_takeoff_current_step < 5) {
            
            computeControlCmds4PreTakeoff();
            // State transition happens in doPreTakeoff() based on flight_arming_state_ and flight_offboard_state_
            pubControlCommands(bodyRateCommand(), Eigen::Vector4d(1,0,0,0)); // Using identity quaternion for now

            ROS_INFO_STREAM_THROTTLE(1.0, "PRE_TAKEOFF: spinning up motors and preparing for takeoff for " << (5 - pre_takeoff_current_step) << " seconds");

            // for tests
            // pubTargetPose2PX4Controller(targetPosition());

        } else {
            // set the mission state to TAKEOFF
            setMissionState(MissionState::TAKEOFF);
            pre_takeoff_flag_ = false;
        }


        break;

        }

        case MissionState::TAKEOFF: {
            // Check if takeoff has been initiated
            if (!take_off_flag_) {
                take_off_begin_ = ros::Time::now();
                take_off_flag_ = true;
                ROS_INFO("TAKEOFF initiated");
                break;
            }
        
            // Compute the time since takeoff began
            take_off_now_ = ros::Time::now();
            double take_off_current_step = (take_off_now_ - take_off_begin_).toSec();
        
            // Compute the target position, velocity, and acceleration for takeoff
            computeTrajectory4Takeoff(take_off_current_step);
            // ROS_INFO_STREAM_THROTTLE(0.5, "TAKEOFF: time step  is " << take_off_current_step);
            
            // computeControlCmds4Takeoff();
            computeControlCmds4Takeoff();
        

            // pubTargetPose2PX4Controller(targetPosition());
            pubControlCommands(bodyRateCommand(), attitudeCommand() ); // Using identity quaternion for now
            

            // check if takeoff is completed by checking the error between the target position and current position
            // if yes, set the mission state to MISSION_EXECUTION
            auto error = (takeoffTargetPosition() - mavPost()).norm();

            if ((take_off_current_step >= (takeoff_time_ + 5)) && (error < 0.5)) {
                ROS_INFO("TAKEOFF 2 MISSION");
                setMissionState(MissionState::MISSION_EXECUTION);
            }
            else {
                // Publish the reference pose for visualization
                ROS_INFO_STREAM_THROTTLE(2.0, "TAKEOFF: takeoff in progress and target post is "  << takeoffTargetPosition().transpose() << " and current error is " << error);
            }
            break;
        }     
        
    case MissionState::MISSION_EXECUTION:{

        // Check if takeoff has been initiated
        if (!mission_flag_) {
                mission_begin_ = ros::Time::now();
                mission_flag_ = true;
                ROS_INFO("MISSION initiated");
                break;
        }

        mission_now_ = ros::Time::now();
        double mission_step = (mission_now_ - mission_begin_).toSec();


        ROS_INFO_STREAM_THROTTLE(2.0, "Mission: begins for  " << mission_step << " seconds");

        // for tests
        // pubTargetPose2PX4Controller(targetPosition());


        computeControlCmds4Mission();
        
        // Publish control commands
        pubControlCommands(bodyRateCommand(), attitudeCommand()); 
        
        // Update and publish pose history
        updateAndPublishPoseHistory();
        
        break;
        }
        
    case MissionState::LANDING: {
        geometry_msgs::PoseStamped landing_msg;
        landing_msg.header.stamp = ros::Time::now();
        
        // Use the accessor method to get home position
        Eigen::Vector3d home = homePosition();
        landing_msg.pose.position.x = home(0);
        landing_msg.pose.position.y = home(1);
        landing_msg.pose.position.z = home(2) + 1.0;
        
        landing_msg.pose.orientation.w = 1.0;
        landing_msg.pose.orientation.x = 0.0;
        landing_msg.pose.orientation.y = 0.0;
        landing_msg.pose.orientation.z = 0.0;
        target_pose_pub_.publish(landing_msg);

        setMissionState(MissionState::LANDED);
        break;
    }
    case MissionState::LANDED:
        ROS_INFO_THROTTLE(1.0, "Landed. Please disarm manually.");
        break;
    }
}

void geomControlROS::statusloopCallback(const ros::TimerEvent& event)
{
    if (sim_enable_) {
        // Simulation mode logic for arming and enabling OFFBOARD
        ROS_INFO_THROTTLE(5.0, "Simulation: arm and OFFBOARD mode enabled automatically");
        // First try to switch to OFFBOARD mode if not already in it
        if (current_state_.mode != "OFFBOARD" && 
            (ros::Time::now() - last_request_ > ros::Duration(5.0))) {
            offb_set_mode_.request.custom_mode = "OFFBOARD";
            if (set_mode_client_.call(offb_set_mode_) && offb_set_mode_.response.mode_sent) {
                ROS_INFO("Offboard request is sent");
            }
            last_request_ = ros::Time::now();
        } 
        // Then try to arm if not already armed
        else if (!current_state_.armed && 
                 (ros::Time::now() - last_request_ > ros::Duration(5.0))) {
            arm_cmd_.request.value = true;
            if (arming_client_.call(arm_cmd_) && arm_cmd_.response.success) {
                ROS_INFO("Arming request is sent");
            }
            last_request_ = ros::Time::now();
        }
    } 
    else {
        // Real hardware mode - don't automatically arm or change modes
        // Any hardware-specific status updates can go here
        ROS_INFO_THROTTLE(5.0, "Waiting for being armed and OFFBOARD mode using TRANSMITTER.");
    }
    
    // // Update the state variables based on current_state_ regardless of simulation mode
    // // Update the state variables based on current_state_
    // if (current_state_.armed) {
    //     setFlightArmingState(FlightArmingState::ARMED);
    // } else {
    //     setFlightArmingState(FlightArmingState::DISARMED);
    // }

    // // Set offboard state based on flight mode, independent of arming status
    // if (current_state_.mode == "OFFBOARD") {
    //     setFlightOffboardState(FlightOffboardState::OFFBOARD_ENABLED);
    // } else {
    //     setFlightOffboardState(FlightOffboardState::OFFBOARD_DISABLED);
    // }
    // // Publish system status
    // pubSystemStatus();
}

void geomControlROS::pubReferencePose(const Eigen::Vector3d& target_position, const Eigen::Vector4d& target_attitude)
{
    // lockstep
}


void geomControlROS::pubTargetPose2PX4Controller(const Eigen::Vector3d& target_position)
{
    // lockstep
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = target_position[0];
    pose.pose.position.y = target_position[1];
    pose.pose.position.z = target_position[2];
    pose.pose.orientation.w = 1.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";  // Ensure consistent frame, if needed
    target_pose_pub_.publish(pose);

    ROS_INFO_STREAM_THROTTLE(1.0, "Publishing target pose to PX4 controller: " << target_position.transpose());

}


void geomControlROS::pubControlCommands(const Eigen::Vector4d& cmd, const Eigen::Vector4d& target_attitude)
{
    mavros_msgs::AttitudeTarget msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.body_rate.x = cmd(0);
    msg.body_rate.y = cmd(1);
    msg.body_rate.z = cmd(2);
    msg.type_mask = msg.IGNORE_PITCH_RATE + msg.IGNORE_ROLL_RATE + msg.IGNORE_YAW_RATE;
    msg.orientation.w = target_attitude(0);
    msg.orientation.x = target_attitude(1);
    msg.orientation.y = target_attitude(2);
    msg.orientation.z = target_attitude(3);
    msg.thrust = cmd(3);
    angularVelPub_.publish(msg);
}

void geomControlROS::updateAndPublishPoseHistory()
{
    // nav_msgs::Path path_msg;
    // path_msg.header.stamp = ros::Time::now();
    // path_msg.header.frame_id = "map";
    // path_msg.poses = posehistory_vector_;
    // posehistoryPub_.publish(path_msg);
}

void geomControlROS::pubSystemStatus() {
  
    mavros_msgs::CompanionProcessStatus msg;
    msg.header.stamp = ros::Time::now();
    msg.component = 196;  // MAV_COMPONENT_ID_AVOIDANCE (standard value for companion computers)
    
    // Convert MissionState enum to appropriate status code
    // switch (mission_state_) {
    //     case MissionState::WAITING_FOR_HOME_POSE:
    //         msg.state = mavros_msgs::CompanionProcessStatus::MAV_STATE_STANDBY;
    //         break;
    //     case MissionState::MISSION_EXECUTION:
    //         msg.state = mavros_msgs::CompanionProcessStatus::MAV_STATE_ACTIVE;
    //         break;
    //     case MissionState::LANDING:
    //         msg.state = mavros_msgs::CompanionProcessStatus::MAV_STATE_CRITICAL;
    //         break;
    //     case MissionState::LANDED:
    //         msg.state = mavros_msgs::CompanionProcessStatus::MAV_STATE_FLIGHT_TERMINATION;
    //         break;
    //     default:
    //         msg.state = mavros_msgs::CompanionProcessStatus::MAV_STATE_UNINIT;
    // }
    
    // systemstatusPub_.publish(msg);
  }


void geomControlROS::dynamicReconfigureCallback(geometric_controller::GeometricControllerConfig &config,
                                               uint32_t level) {

  // obtain the current values of controller gains
  Eigen::Vector3d Kpos = kPPosController();
  Eigen::Vector3d Kvel = kVPosController();
  double max_feedback_acc = maxFeedbackAcceleration();

  
  if(max_feedback_acc != config.max_acc) {
      max_feedback_acc = config.max_acc;
      setMaxFeedbackAcceleration(max_feedback_acc);
      ROS_INFO("Reconfigure request : max_feedback_acc  = %.2f  ", config.max_acc);
    }
    if (Kpos[0] != -config.Kp_x) {
        Kpos[0] = -config.Kp_x;
        ROS_INFO("Reconfigure request : Kp_x  = %.2f  ", config.Kp_x);
    } else if (Kpos[1] != -config.Kp_y) {
        Kpos[1] = -config.Kp_y;
        ROS_INFO("Reconfigure request : Kp_y  = %.2f  ", config.Kp_y);
    } else if (Kpos[2] != -config.Kp_z) {
        Kpos[2] = -config.Kp_z;
        ROS_INFO("Reconfigure request : Kp_z  = %.2f  ", config.Kp_z);
    } else if (Kvel[0] != -config.Kv_x) {
        Kvel[0] = -config.Kv_x;
        ROS_INFO("Reconfigure request : Kv_x  = %.2f  ", config.Kv_x);
    } else if (Kvel[1] != -config.Kv_y) {
        Kvel[1] = -config.Kv_y;
        ROS_INFO("Reconfigure request : Kv_y =%.2f  ", config.Kv_y);
    } else if (Kvel[2] != -config.Kv_z) {
        Kvel[2] = -config.Kv_z;
        ROS_INFO("Reconfigure request : Kv_z  = %.2f  ", config.Kv_z);
    }

    setPostControlPGains(Kpos);
    setPostControlDGains(Kvel);
}  