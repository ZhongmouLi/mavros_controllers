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
    //  nh_private_.param("circle_origin_x", origin_.x(), 0.0);
    //  nh_private_.param("circle_origin_y", origin_.y(), 0.0);
    //  nh_private_.param("circle_origin_z", origin_.z(), 1.0);
     nh_private_.param("circle_axis_x", axis_[0], 0.0);
     nh_private_.param("circle_axis_y", axis_[1], 0.0);
     nh_private_.param("circle_axis_z", axis_[2], 1.0);
     nh_private_.param("circle_omega", omega_, 1.0);
     nh_private_.param("radius", radius_, 1.0);
    //  nh_private_.param("intial_post_x", initial_post_[0], 0.0);
    //  nh_private_.param("intial_post_y", initial_post_[1], 0.0);
    //  nh_private_.param("intial_post_z", initial_post_[2], 1.0);
     

    generator_ = std::make_shared<TrajectoryGenerator>(0.01, 1);
    generator_->setTrajectoryType("CIRCLE");
    generator_->initializeGenerator();
    // generator_->setHomePosition(initial_post_);
    generator_->setCircleTrajectory(axis_, radius_, omega_);

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
     // compute the trajectory at the elapsed time
     generator_->computeTrajectoryAtTime(elapsed);

    // Get the target position, velocity, and acceleration
    p_targ_ = generator_->targetPosition();
    v_targ_ = generator_->targetVelocity();
    a_targ_ = generator_->targetAcceleration();

 }

 void TrajectoryGeneratorROS::takeoffPoseCallback(const geometry_msgs::PoseStamped& msg)
{
    // Update the takeoff position based on the received message
    initial_post_ = toEigen(msg.pose.position);
    generator_->setHomePosition(initial_post_);
    ROS_INFO_STREAM_THROTTLE(5.0, "Takeoff position is set to: " << initial_post_.transpose());
    is_initial_position_set_ = true;
    
}

void TrajectoryGeneratorROS::loopCallback(const ros::TimerEvent&) {
     if (!is_active_) return;
     // Add any trajectory visualization here if needed
 }
 
 void TrajectoryGeneratorROS::refCallback(const ros::TimerEvent&) {

    // check initial position and active state
    // If the initial position is not set or the trajectory is not active, do not publish
    if (!is_initial_position_set_)
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
 
    //  mavros_msgs::PositionTarget raw_msg;
    //  raw_msg.header.stamp = ros::Time::now();
    //  raw_msg.header.frame_id = "map";
    //  raw_msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    //  raw_msg.type_mask = 0;
    //  raw_msg.position.x = p_targ_(0);
    //  raw_msg.position.y = p_targ_(1);
    //  raw_msg.position.z = p_targ_(2);
    //  raw_msg.velocity.x = v_targ_(0);
    //  raw_msg.velocity.y = v_targ_(1);
    //  raw_msg.velocity.z = v_targ_(2);
    //  raw_msg.acceleration_or_force.x = a_targ_(0);
    //  raw_msg.acceleration_or_force.y = a_targ_(1);
    //  raw_msg.acceleration_or_force.z = a_targ_(2);
    //  raw_reference_pub_.publish(raw_msg);

     ROS_DEBUG_STREAM("Publishing reference position at" << p_targ_.transpose());
     ROS_DEBUG_STREAM("Publishing reference vel at" << v_targ_.transpose() );
    //  ROS_DEBUG_STREAM("Publishing reference acc at" << a_targ_.transpose() );
 }
 