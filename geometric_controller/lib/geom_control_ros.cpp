// geom_control_ros.cpp

#include "geometric_controller/geom_control_ros.hpp"

geomControlROS::geomControlROS(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private)
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
    double dt = (reference_request_now_ - reference_request_last_).toSec();

    Eigen::Vector3d target_pos = toEigen(msg.twist.angular);
    Eigen::Vector3d target_vel = toEigen(msg.twist.linear);
    Eigen::Vector3d target_acc = Eigen::Vector3d::Zero();

    if (dt > 0.0) {
        target_acc = (target_vel - targetVel_) / dt;
    }

    updatePreviousTargePostionVelAconst(targetPos_, targetVel_);
    inputTargetPostionVelAcc(target_pos, target_vel, target_acc);
}

void geomControlROS::yawtargetCallback(const std_msgs::Float32& msg)
{
    inputTargeYawAngle(static_cast<double>(msg.data));
}

void geomControlROS::mavstateCallback(const mavros_msgs::State::ConstPtr& msg)
{
    current_state_ = *msg;
}

void geomControlROS::mavposeCallback(const geometry_msgs::TransformStamped::ConstPtr& msg_vicon)
{
    geometry_msgs::Pose pose;
    pose.position.x = msg_vicon->transform.translation.x;
    pose.position.y = msg_vicon->transform.translation.y;
    pose.position.z = msg_vicon->transform.translation.z;
    pose.orientation = msg_vicon->transform.rotation;

    if (!home_position_set_) {
        setHomePosition(toEigen(pose.position));
        ROS_INFO_STREAM("Home pose initialized to: " << home_position_.transpose());
    }

    updateMavPostionAttitude(toEigen(pose.position), toEigen(pose.orientation));
}

void geomControlROS::mavtwistCallback(const geometry_msgs::TwistStamped& msg)
{
    updateMavVelRate(toEigen(msg.twist.linear), toEigen(msg.twist.angular));
}

bool geomControlROS::ctrltriggerCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
    ctrl_mode_ = req.data ? ERROR_GEOMETRIC : ERROR_QUATERNION;
    res.success = true;
    res.message = "Controller mode switched";
    return true;
}

bool geomControlROS::landCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
    node_state_ = LANDING;
    res.success = true;
    res.message = "Landing initiated";
    return true;
}

void geomControlROS::cmdloopCallback(const ros::TimerEvent& event)
{
    switch (node_state_) {
    case WAITING_FOR_HOME_POSE:
        doPreTakeoff();
        if (current_state_.mode == "OFFBOARD" && current_state_.armed) {
            node_state_ = MISSION_EXECUTION;
        }
        break;
    case MISSION_EXECUTION:
        doExecteMission();
        break;
    case LANDING: {
        geometry_msgs::PoseStamped landing_msg;
        landing_msg.header.stamp = ros::Time::now();
        landing_msg.pose.position.x = home_position_(0);
        landing_msg.pose.position.y = home_position_(1);
        landing_msg.pose.position.z = home_position_(2) + 1.0;
        landing_msg.pose.orientation.w = 1.0;
        landing_msg.pose.orientation.x = 0.0;
        landing_msg.pose.orientation.y = 0.0;
        landing_msg.pose.orientation.z = 0.0;
        target_pose_pub_.publish(landing_msg);

        node_state_ = LANDED;
        break;
    }
    case LANDED:
        ROS_INFO_THROTTLE(1.0, "Landed. Please disarm manually.");
        break;
    }
}

void geomControlROS::statusloopCallback(const ros::TimerEvent& event)
{
    if (!current_state_.armed && (ros::Time::now() - last_request_ > ros::Duration(5.0))) {
        arm_cmd_.request.value = true;
        if (arming_client_.call(arm_cmd_) && arm_cmd_.response.success) {
            ROS_INFO("Vehicle armed");
        }
        last_request_ = ros::Time::now();
    }

    if (current_state_.mode != "OFFBOARD" && (ros::Time::now() - last_request_ > ros::Duration(5.0))) {
        offb_set_mode_.request.custom_mode = "OFFBOARD";
        if (set_mode_client_.call(offb_set_mode_) && offb_set_mode_.response.mode_sent) {
            ROS_INFO("Offboard enabled");
        }
        last_request_ = ros::Time::now();
    }
}
