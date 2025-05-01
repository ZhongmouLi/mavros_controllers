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

 #ifndef GEOM_CONTROLLER_H
 #define GEOM_CONTROLLER_H
 #include <iostream>
 #include <chrono>  
 #include "common.h"
 
 #define ERROR_QUATERNION 1
 #define ERROR_GEOMETRIC 2
 
 
 class geomControlBase {
 
     private:
 
     // MAV mission state
    //  enum MissionState {WAITING_FOR_HOME_POSE, MISSION_EXECUTION, LANDING, LANDED } node_state_ = WAITING_FOR_HOME_POSE; // by default it wait for home pose
 
 
     Eigen::Vector3d home_position_; // home position of MAV where it is placed
     bool home_position_set_ = false; // if home position is set and by default it is false
 
     bool received_home_pose;
 
     // target position, velocity, acceleration ans snap
     Eigen::Vector3d targetPos_{0,0,0}, targetVel_{0,0,0}, targetAcc_{0,0,0}, targetSnap_{0,0,0};
 

 
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
     int ctrl_mode_ = ERROR_QUATERNION; 

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
     Eigen::Vector4d cmdBodyRate_{0,0,0,0};  //{wx, wy, wz, Thrust}

          // target attitude
     // note it can also be used as control input
     Eigen::Vector4d q_des_{1,0,0,0};   //{w,x,y,z}
    //  msg.orientation.w = target_attitude(0);
    //  msg.orientation.x = target_attitude(1);
    //  msg.orientation.y = target_attitude(2);
    //  msg.orientation.z = target_attitude(3);

    public:
         enum class MissionState {
             WAITING_FOR_HOME_POSE,
             MISSION_EXECUTION,
             LANDING,
             LANDED
         };
         
         enum class FlightArmingState {
             DISARMED,
             ARMED
         };
         
         enum class FlightOffboardState {
             OFFBOARD_ENABLED,
             OFFBOARD_DISABLED
         };

    protected: // Allow derived classes to access state
         MissionState mission_state_ = MissionState::WAITING_FOR_HOME_POSE;

         FlightArmingState flight_arming_state_ = FlightArmingState::DISARMED;

         FlightOffboardState flight_offboard_state_ = FlightOffboardState::OFFBOARD_DISABLED;
         
         // Protected setters/getters for inherited classes
         void setMissionState(const MissionState &state) { mission_state_ = state; };

         void setFlightArmingState(const FlightArmingState &flight_arming_state) { 
             flight_arming_state_ = flight_arming_state; 
         };

         void setFlightOffboardState(const FlightOffboardState &flight_offboard_state) { 
             flight_offboard_state_ = flight_offboard_state; 
         };

     public: 

         void setHomePosition(const Eigen::Vector3d &home_position);
         
         bool isHomePositionSet() const { return home_position_set_; }         


         void setPostControlPGains(const Eigen::Vector3d &Kpos) {Kpos_ = Kpos; };
        
         void setPostControlDGains(const Eigen::Vector3d &Kvel) {Kvel_ = Kvel; };

         void setPostControlIGains(const Eigen::Vector3d &KposI) {KposI_ = KposI; };
        
         // get target position, velocity, acceleration 
         void inputTargetPositionVelAcc(const Eigen::Vector3d &target_pos, const Eigen::Vector3d &target_vel,
             const Eigen::Vector3d &target_acc);

         void inputTargetYawAngle(const double &target_yaw) ;    
 
         void updatePreviousTargetPositionVelAcc(const Eigen::Vector3d &target_pos_previous, const Eigen::Vector3d &target_vel_previous);
 
         // update mav with current position and attitude   
         void updateMavPositionAttitude(const Eigen::Vector3d &mav_pos, const Eigen::Vector4d &mav_att); 
 
         // update mav with current linear velocity and angular velocity
         void updateMavVelRate(const Eigen::Vector3d &mav_vel, const Eigen::Vector3d &mav_rate);

         Eigen::Vector4d bodyRateCommand() const { return cmdBodyRate_; }

         Eigen::Vector4d attitudeCommand() const { return q_des_; };

         void computeControlCmds4PreTakeoff();

         void computeControlCmds4Mission();

         bool isLanded() const;

         ~geomControlBase();

         geomControlBase();

    protected:
            void setVelocityYawMode(const bool &enable) { velocity_yaw_ = enable; }
            void setDragCoefficients(const Eigen::Vector3d &drag_coeffs) { D_ = drag_coeffs; }
            void setAttitudeControllerGain(double gain) { attctrl_tau_ = gain; }
            void setThrustParameters(double thrust_const, double thrust_offset) {
                norm_thrust_const_ = thrust_const;
                norm_thrust_offset_ = thrust_offset;
            }
            void setMaxFeedbackAcceleration(double max_acc) { max_fb_acc_ = max_acc; }     

    public:
        Eigen::Vector3d targetPosition() const { return targetPos_; }
        Eigen::Vector3d targetVelocity() const { return targetVel_; }
        Eigen::Vector3d homePosition() const { return home_position_; }
        int controlMode() const { return ctrl_mode_; }
        void setControlMode(int mode) { ctrl_mode_ = mode; }


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
  