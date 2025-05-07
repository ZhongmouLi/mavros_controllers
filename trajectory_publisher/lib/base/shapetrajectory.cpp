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

#include "trajectory_publisher/base/shapetrajectory.h"
 shapetrajectory::shapetrajectory(const double &dt, int type) : trajectory(dt, type)  {
  omega_ = 2.0;
  normal_axis_ << 0.0, 0.0, 1.0;
  radial_ << 1.0, 0.0, 0.0;
  // traj_origin_ << 0.0, 0.0, 1.0;
};

shapetrajectory::~shapetrajectory(){

};

// void shapetrajectory::initPrimitives(Eigen::Vector3d pos, Eigen::Vector3d axis, double omega) {
//   // Generate primitives based on current state for smooth trajectory
//   traj_origin_ = pos;
//   omega_ = omega;
//   T_ = 2 * 3.14 / omega_;
//   normal_axis_ = axis;
//   radial_ << 2.0, 0.0, 0.0;
// }

void shapetrajectory::initPrimitives(Eigen::Vector3d normal_axis, const double &radius, double omega)
{
  // assign the vector to be rotated with radius
  radial_<<radius,0.0,0.0;

  // assign the axis of rotation, or normal axis of rotation
  normal_axis_ = normal_axis;
  
  //
  omega_ = omega;
}


void shapetrajectory::generatePrimitives(Eigen::Vector3d pos) {}

void shapetrajectory::generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel) {}

void shapetrajectory::generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d jerk) {}

void shapetrajectory::generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d acc,
                                         Eigen::Vector3d jerk) {}

Eigen::Vector3d shapetrajectory::getPosition(double time) {
  Eigen::Vector3d position;
  double theta;
  double pi = std::acos(-1.0);

  switch (type_) {
    case TRAJ_ZERO:

      position << 0.0, 0.0, 0.0;
      break;

    case TRAJ_CIRCLE:
      theta = omega_ * time;
      position = std::cos(theta) * radial_ + std::sin(theta) * normal_axis_.cross(radial_) +
                 (1 - std::cos(theta)) * normal_axis_.dot(radial_) * normal_axis_ + initialPosition();
      break;

    case TRAJ_LAMNISCATE:  // Lemniscate of Genero

      theta = omega_ * time;
      position = std::cos(theta) * radial_ + std::sin(theta) * std::cos(theta) * normal_axis_.cross(radial_) +
                 (1 - std::cos(theta)) * normal_axis_.dot(radial_) * normal_axis_ + initialPosition();
      break;
    case TRAJ_STATIONARY:  // Lemniscate of Genero

      position = initialPosition();
      break;
  }
  return position;
}

Eigen::Vector3d shapetrajectory::getVelocity(double time) {
  Eigen::Vector3d velocity;
  double theta;

  switch (type_) {
    case TRAJ_CIRCLE:

      velocity = omega_ * normal_axis_.cross(getPosition(time));
      break;
    case TRAJ_STATIONARY:

      velocity << 0.0, 0.0, 0.0;
      break;

    case TRAJ_LAMNISCATE:  // Lemniscate of Genero

      theta = omega_ * time;
      velocity = omega_ *
                 (-std::sin(theta) * radial_ +
                  (std::pow(std::cos(theta), 2) - std::pow(std::sin(theta), 2)) * normal_axis_.cross(radial_) +
                  (std::sin(theta)) * normal_axis_.dot(radial_) * normal_axis_);
      break;

    default:
      velocity << 0.0, 0.0, 0.0;
      break;
  }
  return velocity;
}

Eigen::Vector3d shapetrajectory::getAcceleration(double time) {
  Eigen::Vector3d acceleration;

  switch (type_) {
    case TRAJ_CIRCLE:

      acceleration = omega_ * normal_axis_.cross(getVelocity(time));
      break;
    case TRAJ_STATIONARY:

      acceleration << 0.0, 0.0, 0.0;
      break;
    default:
      acceleration << 0.0, 0.0, 0.0;
      break;
  }
  return acceleration;
}