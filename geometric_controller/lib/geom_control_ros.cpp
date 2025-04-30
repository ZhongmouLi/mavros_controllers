#include "geometric_controller/geom_control_ros.hpp"

geomControlROS::geomControlROS(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : geomControlBase(), // Call the base class constructor
      nh_(nh), 
      nh_private_(nh_private)
{
    // ------------------ Setup Subscribers ------------------
    referenceSub_ = nh_.subscribe("reference/setpoint", 1, &geomControlROS::targetCallback, this, ros::TransportHints().tcpNoDelay());
    yawreferenceSub_ = nh_.subscribe("reference/yaw", 1, &geomControlROS::yawtargetCallback, this, ros::TransportHints().tcpNoDelay());
    mavstateSub_ = nh_.subscribe("mavros/state", 1, &geomControlROS::mavstateCallback, this, ros::TransportHints().tcpNoDelay());
    mavposeSub_ = nh_.subscribe("vicon/drone", 1, &geomControlROS::mavposeCallback, this, ros::TransportHints().tcpNoDelay());
    mavtwistSub_ = nh_.subscribe("mavros/local_position/velocity_local", 1, &geomControlROS::mavtwistCallback, this, ros::TransportHints().tcpNoDelay());

    // ------------------ Setup Publishers ------------------
    angularVelPub_ = nh_.advertise<mavros_msgs::AttitudeTarget>("command/bodyrate_command", 1);
    referencePosePub_ = nh_.advertise<geometry_msgs::PoseStamped>("reference/pose", 1);
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
    }
}

void geomControlROS::mavposeCallback(const geometry_msgs::TransformStamped::ConstPtr& msg_vicon)
{
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
    
    // Add current pose to history vector
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.header.frame_id = "map";
    pose_stamped.pose = pose;
    
    posehistory_vector_.insert(posehistory_vector_.begin(), pose_stamped);
    if (posehistory_vector_.size() > 200) { // Limit history size
        posehistory_vector_.pop_back();
    }
}

void geomControlROS::mavtwistCallback(const geometry_msgs::TwistStamped& msg)
{
    updateMavVelRate(toEigen(msg.twist.linear), toEigen(msg.twist.angular));
}

bool geomControlROS::ctrltriggerCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
    // Use the setter method to update the control mode
    controlMode(req.data ? ERROR_GEOMETRIC : ERROR_QUATERNION);
    
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
    case MissionState::WAITING_FOR_HOME_POSE:
        doPreTakeoff();
        // State transition happens in doPreTakeoff() based on flight_arming_state_ and flight_offboard_state_
        pubRateCommands(bodyRateCommand(), Eigen::Vector4d(1,0,0,0)); // Using identity quaternion for now

        break;
        
    case MissionState::MISSION_EXECUTION:
        doExecteMission();
        
        // Publish control commands
        pubRateCommands(bodyRateCommand(), Eigen::Vector4d(1,0,0,0)); // Using identity quaternion for now
        
        // Update and publish pose history
        updateAndPublishPoseHistory();
        
        break;
        
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
    // Only try to arm and switch to offboard mode if not already done
    if (flight_arming_state_ == FlightArmingState::DISARMED && 
        (ros::Time::now() - last_request_ > ros::Duration(5.0))) {
        arm_cmd_.request.value = true;
        if (arming_client_.call(arm_cmd_) && arm_cmd_.response.success) {
            ROS_INFO("Vehicle armed");
        }
        last_request_ = ros::Time::now();
    }

    if (flight_offboard_state_ == FlightOffboardState::OFFBOARD_DISABLED && 
        flight_arming_state_ == FlightArmingState::ARMED &&
        (ros::Time::now() - last_request_ > ros::Duration(5.0))) {
        offb_set_mode_.request.custom_mode = "OFFBOARD";
        if (set_mode_client_.call(offb_set_mode_) && offb_set_mode_.response.mode_sent) {
            ROS_INFO("Offboard enabled");
        }
        last_request_ = ros::Time::now();
    }
}

void geomControlROS::pubReferencePose(const Eigen::Vector3d& target_position, const Eigen::Vector4d& target_attitude)
{
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.pose.position.x = target_position(0);
    msg.pose.position.y = target_position(1);
    msg.pose.position.z = target_position(2);
    msg.pose.orientation.w = target_attitude(0);
    msg.pose.orientation.x = target_attitude(1);
    msg.pose.orientation.y = target_attitude(2);
    msg.pose.orientation.z = target_attitude(3);
    referencePosePub_.publish(msg);
}

void geomControlROS::pubRateCommands(const Eigen::Vector4d& cmd, const Eigen::Vector4d& target_attitude)
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
    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "map";
    path_msg.poses = posehistory_vector_;
    posehistoryPub_.publish(path_msg);
}