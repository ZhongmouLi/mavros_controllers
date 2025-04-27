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
 * @brief Geometric Controller
 *
 * Geometric controller
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

 #ifndef GEOME_CONTROLLER_H
 #define GEOME_CONTROLLER_H
 #include <iostream>
 #include <chrono>  
 #include "common.h"
 
 #define ERROR_QUATERNION 1
 #define ERROR_GEOMETRIC 2
 
 
 class geomControlBase {
 
     private:
 
     // MAV mission state
     enum FlightState {WAITING_FOR_HOME_POSE, MISSION_EXECUTION, LANDING, LANDED } node_state_;
 
 
     Eigen::Vector3d home_position_; // home position of MAV where it is placed
     bool home_position_set_ = false; // if home position is set
 
     bool received_home_pose;
 
     // target position, velocity, acceleration ans snap
     Eigen::Vector3d targetPos_, targetVel_, targetAcc_, targetSnap_;
 
     // target attitude
     // note it can also be used as control input
     Eigen::Vector4d q_des;
 
     // targe yaw angle
     double mavYaw_;

     bool velocity_yaw_ = false; // if true, use velocity yaw angle
 
     // previous target position, velocity
     Eigen::Vector3d targetPos_prev_, targetVel_prev_;
 
 
 
     // MAV postion and linear velocity
     Eigen::Vector3d mavPos_{0,0,0};
     Eigen::Vector3d mavVel_{0,0,0};
 
     // MAV attitude and angular velocity
     Eigen::Vector4d mavAtt_;
     Eigen::Vector3d mavRate_;
 
 
     // controller mode
     int ctrl_mode_; 

     bool feedthrough_enable_; // flat to use feedthrough or not
     bool fail_detec_, ctrl_enable_;
     bool landing_commanded_;

     // timer for position control
     std::chrono::steady_clock::time_point poscontrol_now_, poscontrol_last_;

     // position control initialized
    bool poscontrol_initialized_ = false;
 
     // control gains
     Eigen::Vector3d Kpos_{8,8,10};
     Eigen::Vector3d Kvel_{1.5,1.5,3.3};
     Eigen::Vector3d KposI_{0.1,0.1,0.1};
     Eigen::Vector3d D_{0,0,0}; // rotor drag
 
     // I gain for position control 
     Eigen::Vector3d error_pose_I{0,0,0};
 
     // max feedforward acceleration
     double max_fb_acc_ = 9.0;

     //
     double attctrl_tau_ = 0.1; 

     // mapping thrust force to thrust
     double norm_thrust_const_ = 0.05;
     double norm_thrust_offset_ =0.1;
     // commands
     // control inputs sent to MAV 
     Eigen::Vector4d cmdBodyRate_;  //{wx, wy, wz, Thrust}
 
     public: 

         void setHomePosition(const Eigen::Vector3d &home_position);

         void setPostControlPGains(const Eigen::Vector3d &Kpos) {Kpos_ = Kpos; };
        
         void setPostControlDGains(const Eigen::Vector3d &Kvel) {Kvel_ = Kvel; };

         void setPostControlIGains(const Eigen::Vector3d &KposI) {KposI_ = KposI; };
        
         // get target position, velocity, acceleration 
         void inputTargetPostionVelAcc(const Eigen::Vector3d &target_pos, const Eigen::Vector3d &target_vel,
             const Eigen::Vector3d &target_acc);

         void inputTargeYawAngle(const double &targe_yaw) ;    
 
         void updatePreviousTargePostionVelAconst(Eigen::Vector3d &target_pos_previous, const Eigen::Vector3d &target_vel_previous);
 
         // update mav with current position and attitude   
         void updateMavPostionAttitude(const Eigen::Vector3d &mav_pos, const Eigen::Vector4d &mav_att); 
 
         // update mav with current linear velocity and angular velocity
         void updateMavVelRate(const Eigen::Vector3d &mav_vel, const Eigen::Vector3d &mav_rate);

         void doPreTakeoff();

         void doExecteMission();

         ~geomControlBase();

         geomControlBase();

;
 
     private:
         // controlPosition calculates desired acc from reference position, vel and acc.
         Eigen::Vector3d controlPosition(const Eigen::Vector3d &target_pos, const Eigen::Vector3d &target_vel,
                                     const Eigen::Vector3d &target_acc);

        Eigen::Vector3d poscontroller(const Eigen::Vector3d &pos_error, const Eigen::Vector3d &vel_error);                                     
 

        Eigen::Vector4d geometric_attcontroller(const Eigen::Vector4d &ref_att, const Eigen::Vector3d &ref_acc,
            Eigen::Vector4d &curr_att); 

        Eigen::Vector4d attcontroller(const Eigen::Vector4d &ref_att, const Eigen::Vector3d &ref_acc,
                Eigen::Vector4d &curr_att); 

         // compute control input body rate and thrust command
         void computeBodyRateCmd(Eigen::Vector4d &bodyrate_cmd, const Eigen::Vector3d &a_des);
 
         static const Eigen::Vector3d g_; // declaration only

         static Eigen::Vector4d acc2quaternion(const Eigen::Vector3d &vector_acc, const double &yaw);

         static double getVelocityYaw(const Eigen::Vector3d velocity) { return atan2(velocity(1), velocity(0)); }         
 };
 
 #endif
  