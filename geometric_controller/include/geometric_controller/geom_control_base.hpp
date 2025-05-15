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
    // Home position of MAV and flag
    Eigen::Vector3d home_position_;
    bool home_position_set_ = false;
    bool received_home_pose;

    // Target position, velocity, acceleration, and snap
    Eigen::Vector3d targetPos_{0,0,0}, targetVel_{0,0,0}, targetAcc_{0,0,0}, targetSnap_{0,0,0};

    // Target yaw angle
    double mavYaw_;
    bool velocity_yaw_ = false;

    // Previous target position and velocity
    Eigen::Vector3d targetPos_prev_, targetVel_prev_;

    // MAV position, velocity, attitude and rate
    Eigen::Vector3d mavPos_{0,0,0};
    Eigen::Vector3d mavVel_{0,0,0};
    Eigen::Vector4d mavAtt_;
    Eigen::Vector3d mavRate_;
 
    // Control configuration
    int ctrl_mode_ = ERROR_QUATERNION;
    bool feedthrough_enable_, fail_detec_, ctrl_enable_, landing_commanded_;

    // Timing and control state
    std::chrono::steady_clock::time_point poscontrol_now_, poscontrol_last_;
    bool poscontrol_initialized_ = false;

    // Control gains
    Eigen::Vector3d Kpos_{8,8,10}, Kvel_{1.5,1.5,3.3}, KposI_{0.1,0.1,0.1}, D_{0,0,0};
    Eigen::Vector3d error_pose_I{0,0,0};

    // Thrust and attitude control parameters
    double max_fb_acc_ = 9.0, attctrl_tau_ = 0.1;
    double norm_thrust_const_ = 0.05, norm_thrust_offset_ = 0.1;

    // Output commands
    Eigen::Vector4d cmdBodyRate_{0,0,0,0};
    Eigen::Vector4d q_des_{1,0,0,0};
    
    
public:
    // Enum declarations for mission and flight states
    enum class MissionState { ARM_OFFBOARD, PRE_TAKEOFF, TAKEOFF, MISSION_EXECUTION, LANDING, LANDED };
    enum class FlightArmingState { DISARMED, ARMED };
    enum class FlightOffboardState { OFFBOARD_ENABLED, OFFBOARD_DISABLED };

protected: // Allow derived classes to access state

    double takeoff_height_ = 0.5;

    double takeoff_time_ = 5.0; // time to take off

    // Flight state variables
    MissionState mission_state_ = MissionState::ARM_OFFBOARD;
    FlightArmingState flight_arming_state_ = FlightArmingState::DISARMED;
    FlightOffboardState flight_offboard_state_ = FlightOffboardState::OFFBOARD_DISABLED;

    // Setters for state variables (used by derived classes)
    void setMissionState(const MissionState &state){ mission_state_ = state; };
    void setFlightArmingState(const FlightArmingState &flight_arming_state) { flight_arming_state_ = flight_arming_state;};
    void setFlightOffboardState(const FlightOffboardState &flight_offboard_state){flight_offboard_state_ = flight_offboard_state; };        

protected:
    // Advanced control configuration for derived controllers
    void setVelocityYawMode(const bool &enable) { velocity_yaw_ = enable; };
    void setDragCoefficients(const Eigen::Vector3d &drag_coeffs) { D_ = drag_coeffs; }
    void setAttitudeControllerGain(double gain){ attctrl_tau_ = gain; }
    void setThrustParameters(double thrust_const, double thrust_offset){
                norm_thrust_const_ = thrust_const;
                norm_thrust_offset_ = thrust_offset;
            };
    void setMaxFeedbackAcceleration(double max_acc){ max_fb_acc_ = max_acc; }    ;
    void computeTrajectory4Takeoff(const double &current_time);
    void computeControlCmds4Takeoff();
    void setTakeoffHeightAndTime(const double &takeoff_height, const double &takeoff_time){ takeoff_height_ = takeoff_height; takeoff_time_ = takeoff_time;};
  

public:  

    // Set home position of MAV
    void setHomePosition(const Eigen::Vector3d &home_position);

    // Return if home position has been set
    bool isHomePositionSet() const  { return home_position_set_; } ;

    // Set position controller gains
    void setPostControlPGains(const Eigen::Vector3d &Kpos)  {Kpos_ = Kpos; };
    void setPostControlDGains(const Eigen::Vector3d &Kvel) {Kvel_ = Kvel; };
    void setPostControlIGains(const Eigen::Vector3d &KposI)  {KposI_ = KposI; };

    // Input target trajectory information
    void inputTargetPositionVelAcc(const Eigen::Vector3d &target_pos, const Eigen::Vector3d &target_vel, const Eigen::Vector3d &target_acc);
    void inputTargetYawAngle(const double &target_yaw);
    void updatePreviousTargetPositionVelAcc(const Eigen::Vector3d &target_pos_previous, const Eigen::Vector3d &target_vel_previous);

    // Update MAV state
    void updateMavPositionAttitude(const Eigen::Vector3d &mav_pos, const Eigen::Vector4d &mav_att);
    void updateMavVelRate(const Eigen::Vector3d &mav_vel, const Eigen::Vector3d &mav_rate);

    // Return control commands
    Eigen::Vector4d bodyRateCommand() const  { return cmdBodyRate_; };
    Eigen::Vector4d attitudeCommand() const  { return q_des_; };

    // Compute commands for different mission stages
    void computeControlCmds4PreTakeoff();
    void computeControlCmds4Mission();

    // Check landing state  
    bool isLanded() const;

    // Constructor and destructor
    geomControlBase();
    ~geomControlBase();

    // Accessor methods for MAV state
    Eigen::Vector3d mavPost() const { return mavPos_; }
    Eigen::Vector3d mavVel() const { return mavVel_; }
    Eigen::Vector4d mavAtt() const { return mavAtt_; }
    Eigen::Vector3d mavRate() const { return mavRate_; }

    // Accessor methods for target state
    Eigen::Vector3d targetPosition() const { return targetPos_; };
    Eigen::Vector3d targetVelocity() const { return targetVel_; };
    Eigen::Vector3d targetAcceleration() const { return targetAcc_; };
    Eigen::Vector3d homePosition() const { return home_position_; };

    Eigen::Vector3d takeoffTargetPosition() const { return home_position_ + Eigen::Vector3d(0, 0, 1) * takeoff_height_; };
           
    int controlMode() const { return ctrl_mode_; };
    void setControlMode(int mode) { ctrl_mode_ = mode; };

    Eigen::Vector3d kPPosController() const { return Kpos_; }
    Eigen::Vector3d kVPosController() const { return Kvel_; }
    double maxFeedbackAcceleration() const { return max_fb_acc_; }

private:

    // Core internal control logic
    Eigen::Vector3d controlPosition(const Eigen::Vector3d &target_pos, const Eigen::Vector3d &target_vel, const Eigen::Vector3d &target_acc);
    Eigen::Vector3d poscontroller(const Eigen::Vector3d &pos_error, const Eigen::Vector3d &vel_error);
    Eigen::Vector4d geometric_attcontroller(const Eigen::Vector4d &ref_att, const Eigen::Vector3d &ref_acc, Eigen::Vector4d &curr_att);
    Eigen::Vector4d attcontroller(const Eigen::Vector4d &ref_att, const Eigen::Vector3d &ref_acc, Eigen::Vector4d &curr_att);
    void computeBodyRateCmd(Eigen::Vector4d &bodyrate_cmd, const Eigen::Vector3d &a_des);

    // Utility constants and methods
    static const Eigen::Vector3d g_;
    static Eigen::Vector4d acc2quaternion(const Eigen::Vector3d &vector_acc, const double &yaw);
    static double getVelocityYaw(const Eigen::Vector3d velocity) { return atan2(velocity(1), velocity(0)); }  ;


};
# endif
//  