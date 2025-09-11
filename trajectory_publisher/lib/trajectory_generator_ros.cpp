/****************************************************************************
 *
 *   Copyright (c) 2018-2021 Jaeyoung Lim. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVIStakeoffPoseCallbackED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @brief Trajectory Publisher
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */
 #include "trajectory_publisher/trajectory_generator_ros.hpp"


 TrajectoryGeneratorROS::TrajectoryGeneratorROS(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
     : nh_(nh), nh_private_(nh_private), is_active_(false) {
 
     // Load parameters

     
    // load trajectory definition from yaml file    
    std::string pkg_path = ros::package::getPath("trajectory_generator");
    std::string yaml_path_default = pkg_path + "/config/example.yaml";

    nh.param<std::string>("config_file", yaml_path_, yaml_path_default);
    ROS_INFO("Loading trajectory configuration from: %s", yaml_path_.c_str());

     // Publishers
     reference_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("reference/setpoint", 1);
    //  raw_reference_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 1);
 
     // Service
     start_service_ = nh_.advertiseService("trajectory_generator/start", &TrajectoryGeneratorROS::startCallback, this);

    //  // Subscribers
     takeoff_pose_sub_ = nh_.subscribe("reference/takeoff_pose", 1, &TrajectoryGeneratorROS::takeoffPoseCallback, this, ros::TransportHints().tcpNoDelay());

     // Timers
     loop_timer_ = nh_.createTimer(ros::Duration(0.1), &TrajectoryGeneratorROS::loopCallback, this);
     ref_timer_ = nh_.createTimer(ros::Duration(0.01), &TrajectoryGeneratorROS::refCallback, this);
 
     ROS_INFO("TrajectoryGeneratorROS initialized, waiting for /start service call...");

     // create trajectory generator
     ptr_traj_generator_ = std::make_shared<TrajectoryGenerator>(0.01);


    // Initialize generator from yaml
    inputTrajectoryConfig();
 }


 
 bool TrajectoryGeneratorROS::startCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
     if (req.data) {
         start_time_ = ros::Time::now();
         is_active_ = true;
         res.success = true;
         res.message = "Trajectory started";
         ROS_INFO("Trajectory started");
     } else {
         is_active_ = false;
         res.success = true;
         res.message = "Trajectory stopped";
         ROS_INFO("Trajectory stopped");
     }
     return true;
 }
 
 void TrajectoryGeneratorROS::updateReference() {

      // Get the elapsed time since the trajectory started
     double elapsed = (ros::Time::now() - start_time_).toSec();

     if (!ptr_traj_generator_) { ROS_ERROR_THROTTLE(1.0, "generator not initialized"); return; }

     ptr_traj_generator_->chooseAndConfigureByTime(elapsed);

     // compute the trajectory at the elapsed time
     ptr_traj_generator_->computeTrajectoryAtTime(elapsed);

    // Get the target position, velocity, and acceleration
    p_targ_ = ptr_traj_generator_->targetPosition();
    v_targ_ = ptr_traj_generator_->targetVelocity();
    a_targ_ = ptr_traj_generator_->targetAcceleration();

 }

 void TrajectoryGeneratorROS::takeoffPoseCallback(const geometry_msgs::PoseStamped& msg)
{
    // ROS_INFO("fuck takeoffPoseCallback 1");
    // Update the takeoff position based on the received message
    initial_post_ = toEigen(msg.pose.position);

    if (!ptr_traj_generator_->isInitPositionSet())
    {
        ptr_traj_generator_->setInitPosition(initial_post_);
    }

    ROS_INFO_STREAM_THROTTLE(5.0, "Takeoff position is set to: " << initial_post_.transpose());
    //  ROS_INFO("fuck takeoffPoseCallback 2");
}

void TrajectoryGeneratorROS::inputTrajectoryConfig()
{
    YAML::Node cfg = YAML::LoadFile(yaml_path_);

    const auto segments = cfg["trajectory"]["segments"];

    for (const auto& seg : segments) {
        std::string id   = seg["id"].as<std::string>();
        std::string type = seg["type"].as<std::string>();
        double dur       = seg["duration"].as<double>();

        if (type == "POLYNOMIAL") {
            PolynomialTrajStrct poly;
            poly.id = id;
            poly.type = TrajectoryType::POLYNOMIAL;
            poly.duration = dur;

            poly.start_position = seg["start"]["position"].as<std::array<double,3>>();
            poly.start_yaw      = seg["start"]["yaw"].as<double>();
            poly.end_position   = seg["end"]["position"].as<std::array<double,3>>();
            poly.end_yaw        = seg["end"]["yaw"].as<double>();

            // Print with ROS_INFO (printf-style)
            ROS_INFO(
            "Loaded POLYNOMIAL segment: id=%s dur=%.3f "
            "start=[%.3f, %.3f, %.3f] start_yaw=%.3f "
            "end=[%.3f, %.3f, %.3f] end_yaw=%.3f",
            poly.id.c_str(), poly.duration,
            poly.start_position[0], poly.start_position[1], poly.start_position[2], poly.start_yaw,
            poly.end_position[0],   poly.end_position[1],   poly.end_position[2],   poly.end_yaw
            );

            ptr_traj_generator_->inputTrajectoryStrcutData(poly);
        }
        else if (type == "CIRCLE") {
            CircleTrajStrct circle;
            circle.id = id;
            circle.type = TrajectoryType::CIRCLE;
            circle.duration = dur;

            circle.circle_axis  = seg["circle_axis"].as<std::array<double,3>>();
            circle.circle_omega = seg["circle_omega"].as<double>();

            if (seg["initial_pos"]) {
                circle.initial_pos = seg["initial_pos"].as<std::array<double,3>>();
            } else {
                circle.initial_pos = seg["intial_post_x"].as<std::array<double,3>>(); // fallback typo
            }

            ptr_traj_generator_->inputTrajectoryStrcutData(circle);
        }
    }
}


void TrajectoryGeneratorROS::loopCallback(const ros::TimerEvent&) {
     if (!is_active_) return;
     // Add any trajectory visualization here if needed
 }
 
 void TrajectoryGeneratorROS::refCallback(const ros::TimerEvent&) {

    // check initial position and active state
    // If the initial position is not set or the trajectory is not active, do not publish
    if (!ptr_traj_generator_->isInitPositionSet())
    {
        ROS_INFO_THROTTLE(1.0, "Initial position of trajectory is not set, waiting for /start service call...");

        return;
    }
    else if (!is_active_)
     {
        ROS_INFO_THROTTLE(1.0, "Trajectory is not active, waiting for /start service call...");

        return;
      }
        // This callback runs at 100Hz, so we can compute the trajectory at this rate
        // Get the elapsed time since the trajectory started
     
      // compute the reference state
     updateReference();
 
     geometry_msgs::TwistStamped twist_msg;
     twist_msg.header.stamp = ros::Time::now();
     twist_msg.header.frame_id = "map";
     twist_msg.twist.angular.x = p_targ_(0);
     twist_msg.twist.angular.y = p_targ_(1);
     twist_msg.twist.angular.z = p_targ_(2);
     twist_msg.twist.linear.x = v_targ_(0);
     twist_msg.twist.linear.y = v_targ_(1);
     twist_msg.twist.linear.z = v_targ_(2);
     reference_pub_.publish(twist_msg);
 

     ROS_INFO_STREAM_THROTTLE(2, "Publishing reference position at" << p_targ_.transpose());
     ROS_INFO_STREAM_THROTTLE(2, "Publishing reference vel at" << v_targ_.transpose() );
    //  ROS_DEBUG_STREAM("Publishing reference acc at" << a_targ_.transpose() );
 }
 