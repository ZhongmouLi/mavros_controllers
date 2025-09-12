#pragma once
#include <iostream>
#include <variant>
#include <chrono>
#include "common.h"

class geomControlBase {

public:
    // Constructor and destructor
    geomControlBase() = default;
    ~geomControlBase() = default;

    // Public interface
    /*---------------------Drone status----------------------------*/
    // set home position and its flag
    void setHomePosition(const Eigen::Vector3d &home_position); // set variable home_position_
    bool isHomePositionSet() const; // depends on variable home_position_set_
    Eigen::Vector3d homePosition() const; // home position of mav

    // input target post, vel and acc
    void inputTargetPositionVelAcc(const Eigen::Vector3d &target_pos, const Eigen::Vector3d &target_vel, const Eigen::Vector3d &target_acc);
    // input target yaw angle
    void inputTargetYawAngle(const double &target_yaw);
    // decide if yaw angle is obtained by
    void setVelocityYawMode(const bool &enable);
    // update previous target post and vel
    void updatePreviousTargetPositionVelAcc(const Eigen::Vector3d &target_pos_previous, const Eigen::Vector3d &target_vel_previous);

    // obtain target post, vel and acc
    [[nodiscard]] Eigen::Vector3d targetPosition() const;
    [[nodiscard]] Eigen::Vector3d targetVelocity() const;
    [[nodiscard]] Eigen::Vector3d targetAcceleration() const;

    // update mav status
    // update mav position and attitude
    void updateMavPositionAttitude(const Eigen::Vector3d &mav_pos, const Eigen::Vector4d &mav_att);

    // update mav linear and angular velocity
    void updateMavVelRate(const Eigen::Vector3d &mav_vel, const Eigen::Vector3d &mav_rate);



    // define Mission state
    enum class MissionState {TO_ARM_OFFBOARD, PRE_TAKEOFF, TAKEOFF, MISSION_EXECUTION, LANDING, LANDED };
    // define Flight arming state: armed or disarmed
    enum class FlightArmingState { DISARMED, ARMED };
    // define Flgiht offboard state: offboard enabled or offboard disabled
    enum class FlightOffboardState { OFFBOARD_ENABLED, OFFBOARD_DISABLED };


    bool isLanded() const;

    /*---------------------Controller----------------------------*/

    // controm mode between {AttitudeControl, BodyrateControl}
    // in AttitudeControl, cmd sent to drone is thrust + attitude
    // in BodyrateControl, cmd sent to drone is thrust + bodyrate
    enum class ControlMode {AttitudeControl, BodyrateControl};

    ControlMode controlMode() const;

    // set controm mode between {AttitudeControl, BodyrateControl}
    void setControlMode(const ControlMode &mode);

    // set linear mapping paramemers to map acc_z cmd to normalised throttle (-1,1)
    void setThrustParameters(const double &thrust_const, const double &thrust_offset);

    void setDragCoefficients(const Eigen::Vector3d &drag_coeffs);

    // set maximal accleration for feedback
    void setMaxFeedbackAcceleration(const double &max_acc);

    /*-------position control------------*/
    // compute target acc to track target_pos, target_vel, and  target_acc
    // a_des = target_acc +  feedback_acc computed by poscontroller
    Eigen::Vector3d computeDesAcc(const Eigen::Vector3d &target_pos, const Eigen::Vector3d &target_vel, const Eigen::Vector3d &target_acc);

    // set PID gains for position control
    void setPostControlPGains(const Eigen::Vector3d &Kpos);
    void setPostControlDGains(const Eigen::Vector3d &Kvel);
    void setPostControlIGains(const Eigen::Vector3d &KposI);

    /*-------attitude and bodyrate control------------*/
    // compute control_input_ based on reference acc
    void computeControlInputs(const Eigen::Vector3d &a_des);


    // set gain for attitude control
    void setAttitudeControllerGain(const double &attitude_gain);

    /*-------control mission------------*/
    // compute cmd to compensate gravity of drone to takeoff
    void computeControlCmds4PreTakeoff();

    // set takeoff heiht and time
    void setTakeoffHeightAndTime(const double &takeoff_height, const double &takeoff_time);
    // compute takeoff trajecotry based on current time
    void computeTrajectory4Takeoff(const double &current_time);
    // compute cmd to takeoff following takeoff trajectory
    void computeControlCmds4Takeoff();
    // get takeoff post
    Eigen::Vector3d takeoffTargetPosition() const;

    // compute cmd to track trajectory defined in mission
    void computeControlCmds4Mission();

protected:
    // mav mission_sate
    MissionState mission_state_ = MissionState::TO_ARM_OFFBOARD;
    // mav arming sate
    FlightArmingState flight_arming_state_ = FlightArmingState::DISARMED;
    // mav offboard state
    FlightOffboardState flight_offboard_state_ = FlightOffboardState::OFFBOARD_DISABLED;

    void setMissionState(const MissionState &state);

    void setFlightArmingState(const FlightArmingState &flight_arming_state);

    void setFlightOffboardState(const FlightOffboardState &flight_offboard_state);

    // Control command structures of mode AttitudeControl and mode BodyrateControl
    struct ThrustAttitude {
        double thrust;
        Eigen::Quaterniond attitude;
    };


    struct ThrustBodyrate {
        double thrust;
        Eigen::Vector3d bodyrate;
    };

    // uniform control input = ThrustAttitude or ThrustBodyrate
    using ControlInput = std::variant<ThrustAttitude, ThrustBodyrate>;

private:

    /*----------------------------MAV status-------------------------*/
    // Home position
    Eigen::Vector3d home_position_;
    bool home_position_set_ = false;

    // mav state variables
    Eigen::Vector3d mavPos_{0,0,0};
    Eigen::Vector3d mavVel_{0,0,0};
    Eigen::Vector4d mavAtt_;
    Eigen::Vector3d mavRate_;

    // mav yaw angle
    double mavYaw_;
    // if mavYaw_ is estimated from linear velocity
    bool velocity_yaw_ = false;

    // Target trajectory variables
    Eigen::Vector3d targetPos_{0,0,0}, targetVel_{0,0,0}, targetAcc_{0,0,0}, targetSnap_{0,0,0};
    Eigen::Vector3d targetPos_prev_, targetVel_prev_;


    // Takeoff parameters
    double takeoff_height_ = 0.5;
    double takeoff_time_ = 5.0;

    /*----------------------------Controller-------------------------*/

    /*----------Position Controller----------*/
    // choose controlPosition only keep target_acc or not
    // true : result  = target_acc
    // false : result  = target_acc +  feedback_acc
    bool feedthrough_enable_ = false;

    // feedback control
    // compute feedback acc
    Eigen::Vector3d posControllerPID(const Eigen::Vector3d &pos_error, const Eigen::Vector3d &vel_error);

    // help timer as the first run
    bool poscontrol_initialized_ = false;
    // Time tracking for control loop
    std::chrono::steady_clock::time_point poscontrol_now_, poscontrol_last_;

    // limit of feedback acc
    double max_fb_acc_ = 9.0;

    // control gains
    Eigen::Vector3d Kpos_{8,8,10};
    Eigen::Vector3d Kvel_{1.5,1.5,3.3};
    Eigen::Vector3d KposI_{0.1,0.1,0.1};

    Eigen::Vector3d D_{0,0,0};
    Eigen::Vector3d error_pose_I{0,0,0};


    /*----------Attitude/Bodyrate Controller----------*/
    // Control mode enums
    // in AttitudeControl, cmd sent to drone is thrust + attitude
    // in BodyrateControl, cmd sent to drone is thrust + bodyrate

    ControlMode control_mode_ = ControlMode::AttitudeControl;

    ControlInput control_input_;

    // // compute control_input_ based on reference acc
    // void computeControlInputs(const Eigen::Vector3d &a_des);

    // param to transform thurst force into normalised thrust
    double norm_thrust_const_ = 0.05;
    double norm_thrust_offset_ = 0.1;

    /*----------Attitude Controller----------*/
    // compute normalised thrust and reference attitude
    ThrustAttitude computeThrustAttitudeCmd(const Eigen::Vector3d &ref_acc, const double &current_yaw);

    // compute target attitude in quaternion
    Eigen::Quaterniond computeRefAttitudeQuaterion(const Eigen::Vector3d &ref_acc, const double &current_yaw);

    // control gain of attitude
    double attctrl_tau_ = 0.1;

    /*----------Bodyrate Controller----------*/
    // compute normalised thrust and bodyrate
    ThrustBodyrate computeThurstBodyrateCmd(const Eigen::Quaterniond &ref_att, const Eigen::Vector3d &ref_acc, const Eigen::Vector4d &curr_att);

    // two methods to compute bodyrate
    enum class BodyrateControlMethod {Geometric, Basic};
    BodyrateControlMethod bodyrate_method_ = BodyrateControlMethod::Basic;

    // geometir method to compute bodyrate
    ThrustBodyrate geometricAttcontroller(const Eigen::Quaterniond &ref_att, const Eigen::Vector3d &ref_acc, const Eigen::Vector4d &curr_att);
    // basic method to compute bodyrate
    ThrustBodyrate attController(const Eigen::Quaterniond &ref_att, const Eigen::Vector3d &ref_acc, const Eigen::Vector4d &curr_att);

    // general methods and param
    static const Eigen::Vector3d g_;
    static Eigen::Quaterniond acc2quaternion(const Eigen::Vector3d &vector_acc, const double &yaw);
    static double computeYawFromVelocity(const Eigen::Vector3d velocity) { return atan2(velocity(1), velocity(0)); };


public:
    ControlInput controlInput() const {return control_input_;};
    
    double takeoffTime() const {return takeoff_time_;}

    Eigen::Vector3d kPPosController() const {return Kpos_;};

    Eigen::Vector3d kVPosController() const {return Kvel_;};

    double maxFeedbackAcceleration() const {return max_fb_acc_;};

    // obtain mav current post, vel, attitude and bodyrate
    [[nodiscard]] Eigen::Vector3d mavPost() const;
    [[nodiscard]] Eigen::Vector3d mavVel() const;
    [[nodiscard]] Eigen::Vector4d mavAtt() const;
    [[nodiscard]] Eigen::Vector3d mavRate() const;
};
