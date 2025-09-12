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
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @brief Shape Trajectory Library
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

 #ifndef TRAJECTORY_PUBLISHER_SHAPETRAJECTORY_H
 #define TRAJECTORY_PUBLISHER_SHAPETRAJECTORY_H
 
 #include "trajectory_publisher/base/trajectory.h"
 
 #define TRAJ_ZERO 0
 #define TRAJ_CIRCLE 1
 #define TRAJ_LAMNISCATE 2
 #define TRAJ_STATIONARY 3
 
 class shapetrajectory : public trajectory {
 private:
 //   int type_;
    int N;
 //   double dt_;
   //  double T_;
 //   
 //   Eigen::Vector3d traj_origin_;
 private:
 Eigen::Vector3d radial_;
 Eigen::Vector3d normal_axis_; // Normal axis of the rotation defined in Rodrigues’ rotation formula
 double omega_; // Angular velocity of circular motion
 //   double traj_radius_, traj_omega_;
 
  public:
   shapetrajectory(const double &dt, int type);
   virtual ~shapetrajectory();
   void initPrimitives(Eigen::Vector3d normal_axis, const double &radius, double omega);


   void initPrimitives(const Eigen::Vector3d &start_point, const Eigen::Vector3d &normal_axis, const double &radius, const double &omega);


   void generatePrimitives(Eigen::Vector3d pos);
   void generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel);
   void generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d jerk);
   void generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d acc, Eigen::Vector3d jerk);
   Eigen::Vector3d getPosition(const double &time);
   Eigen::Vector3d getVelocity(const double &time);
   Eigen::Vector3d getAcceleration(const double &time);


   double circleRadius() const {
     return radial_[0];
   };
 
   Eigen::Vector3d circleAxis() const {
     return normal_axis_;
   };
 
   double circleOmega() const {
     return omega_;
   };
 
   // double getsamplingTime() { return dt_; };
   // double getDuration() { return T_; };
   // nav_msgs::Path getSegment();
   // geometry_msgs::PoseStamped vector3d2PoseStampedMsg(Eigen::Vector3d position, Eigen::Vector4d orientation);
 };
 #endif  // TRAJECTORY_PUBLISHER_SHAPETRAJECTORY_H
 