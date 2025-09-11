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
 * @brief Polynomial Trajectory
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

 #include "trajectory_publisher/base/polynomialtrajectory.h"

polynomialtrajectory::polynomialtrajectory(const double&dt) : trajectory(dt, /*type=*/0)
{

};
 
polynomialtrajectory::~polynomialtrajectory() {}
 

void polynomialtrajectory::initPrimitives(const Eigen::Vector3d &target_post, const double &travelling_time)
{

    travelling_time_ = travelling_time;

    distance_initial2target_post_ = target_post - intial_post_;

}

Eigen::Vector3d polynomialtrajectory::getPosition(const double &current_time)
{
  double factor = current_time/travelling_time_;

  double r;

   if (factor<= 1)
    {
            r = 10 * pow(factor, 3) -15 * pow(factor, 4) + 6 * pow(factor, 5);

    }
    else
    {
             r = 1;         
    }


    Eigen::Vector3d v_r = Eigen::Vector3d(1, 1, 1) * r ;

    // a.cwiseProduct(b)
    Eigen::Vector3d target_position = intial_post_ + v_r.cwiseProduct(distance_initial2target_post_);


    return target_position;
};

Eigen::Vector3d polynomialtrajectory::getVelocity(const double &current_time)
{
  double factor = current_time/travelling_time_;

  double dr;
  if (factor<= 1)
  {

            dr = 30 * pow(current_time, 2)/pow(travelling_time_,3)  - 60 * pow(current_time, 3)/pow(travelling_time_,4) + 30 * pow(current_time, 4)/pow(travelling_time_,5);


  }
  else
  {

             dr = 0;          
  }
  
    Eigen::Vector3d v_dr= Eigen::Vector3d(1, 1, 1) * dr ;


    Eigen::Vector3d  target_velocity=  v_dr.cwiseProduct(distance_initial2target_post_);

    return target_velocity;

};

Eigen::Vector3d polynomialtrajectory::getAcceleration(const double &current_time) 
{
  double factor = current_time/travelling_time_;

  double ddr;
  if (factor<= 1)
    {
           
            ddr = 60 * current_time/pow(travelling_time_, 3) - 180 * pow(current_time, 2)/pow(travelling_time_, 4) + 120 * pow(current_time, 3)/pow(travelling_time_,5);

    }
    else
    {

             ddr = 0;
          
    }

    Eigen::Vector3d v_ddr= Eigen::Vector3d(1, 1, 1) * ddr ;

    Eigen::Vector3d  target_acceleration= v_ddr.cwiseProduct(distance_initial2target_post_);

    return target_acceleration;

};