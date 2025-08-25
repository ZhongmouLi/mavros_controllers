#include "geometric_controller/geom_control_base.hpp"



const Eigen::Vector3d geomControlBase::g_{0.0, 0.0, -9.8}; // gravity vector

// geomControlBase::~geomControlBase() {
//     // Destructor
//   }

// geomControlBase::geomControlBase(): home_position_set_(false)
// {
// }


// return control mode of mav
geomControlBase::ControlMode geomControlBase::controlMode() const
{
    return control_mode_;
};


// set home position
void geomControlBase::setHomePosition(const Eigen::Vector3d &home_position) {
    home_position_ = home_position;
    targetPos_ = home_position_; // set home position as initial target position
    home_position_set_ = true;
}

bool geomControlBase::isLanded() const {
    // Old way:
    // return (node_state_ == LANDED);
    
    // New way:
    return (mission_state_ == MissionState::LANDED);
}


// input target position, velocity and acceleration
void geomControlBase::inputTargetPositionVelAcc(const Eigen::Vector3d &target_pos, const Eigen::Vector3d &target_vel,
    const Eigen::Vector3d &target_acc) {
    targetPos_ = target_pos;
    targetVel_ = target_vel;
    targetAcc_ = target_acc;
}

// input previous target position and velocity
void geomControlBase::updatePreviousTargetPositionVelAcc(const Eigen::Vector3d &target_pos_previous, const Eigen::Vector3d &target_vel_previous) {
    targetPos_prev_ = target_pos_previous;
    targetVel_prev_ = target_vel_previous;
}

// input targe yaw angle 
void geomControlBase::inputTargetYawAngle(const double &target_yaw) {
    mavYaw_ = target_yaw;
}


// update mav with current position and attitude   
void geomControlBase::updateMavPositionAttitude(const Eigen::Vector3d &mav_pos, const Eigen::Vector4d &mav_att) {
    mavPos_ = mav_pos;
    mavAtt_ = mav_att;
}

// update mav with current linear velocity and angular velocity
void geomControlBase::updateMavVelRate(const Eigen::Vector3d &mav_vel, const Eigen::Vector3d &mav_rate) {
    mavVel_ = mav_vel;
    mavRate_ = mav_rate;
}

// do pre takeoff preparation
// this function is called when the drone is in pre takeoff state
// it will set the home position as the destination and calculate the desired acceleration and compute the control input body rate command
// it switch to mission execution state when the drone is armed and in offboard mode
void geomControlBase::computeControlCmds4PreTakeoff(){

    // ROS_INFO_STREAM_THROTTLE(2, "home_position is "<< home_position_);
    if (home_position_set_) {
        Eigen::Vector3d desired_acc = computeDesAcc(home_position_,  Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    
        computeControlInputs(desired_acc);

    }
    else {
        std::cout << "home position is not set" << std::endl;

        return;
    }
        

}


void geomControlBase::computeTrajectory4Takeoff(const double &current_time) {

    double factor = current_time/takeoff_time_;

    // std::cout << "current_time is " << current_time << std::endl;

    // std::cout << "factor is " << factor << std::endl;

    // std::cout << "takeoff_time_ is " << takeoff_time_ << std::endl;

    double r, dr, ddr;

    if (factor<= 1)
    {
            r = 10 * pow(factor, 3) -15 * pow(factor, 4) + 6 * pow(factor, 5);

            dr = 30 * pow(current_time, 2)/pow(takeoff_time_,3)  - 60 * pow(current_time, 3)/pow(takeoff_time_,4) + 30 * pow(current_time, 4)/pow(takeoff_time_,5);

            ddr = 60 * current_time/pow(takeoff_time_, 3) - 180 * pow(current_time, 2)/pow(takeoff_time_, 4) + 120 * pow(current_time, 3)/pow(takeoff_time_,5);

            // std::cout << "<1 r is " << factor << std::endl;

            // std::cout << "<1 dr is " << dr << std::endl;

            // std::cout << "<1  ddr is " << ddr << std::endl;
    }
    else
    {
             r = 1;
             dr = 0;
             ddr = 0;
            //  std::cout << ">=1 r is " << factor << std::endl;

            //  std::cout << ">=1 dr is " << dr << std::endl;
 
            //  std::cout << ">=1  ddr is " << ddr << std::endl;             
    }

    Eigen::Vector3d target_position_takeoff = home_position_ + Eigen::Vector3d(0, 0, 1) * r * takeoff_height_;

    Eigen::Vector3d target_velocity_takeoff =  Eigen::Vector3d(0, 0, 1) *dr* takeoff_height_;

    Eigen::Vector3d target_acceleration_takeoff= Eigen::Vector3d(0, 0, 1) *ddr * takeoff_height_;

    inputTargetPositionVelAcc(target_position_takeoff, target_velocity_takeoff, target_acceleration_takeoff);
}


void geomControlBase::computeControlCmds4Takeoff() {

    if (mission_state_ != MissionState::TAKEOFF) {
        return;
    }

    Eigen::Vector3d desired_acc;
  
    if (feedthrough_enable_) {
        desired_acc = targetAcc_;
    } else {
        desired_acc = computeDesAcc(targetPos_, targetVel_, targetAcc_);
    }

     computeControlInputs(desired_acc);
    
}




// do mission execution
// this function is called when the drone is in mission execution state
// it will calculate the control input based on the target position, velocity and acceleration
void geomControlBase::computeControlCmds4Mission() {

    if (mission_state_ != MissionState::MISSION_EXECUTION) {
        return;
    }

    Eigen::Vector3d desired_acc;
  
    if (feedthrough_enable_) {
        desired_acc = targetAcc_;
    } else {
        desired_acc = computeDesAcc(targetPos_, targetVel_, targetAcc_);
    }

    // computeBodyRateCmd(cmdBodyRate_, desired_acc);
    computeControlInputs(desired_acc);
    
}



// computeDesAcc calculates desired acc from reference position, vel and acc.
Eigen::Vector3d geomControlBase::computeDesAcc(const Eigen::Vector3d &target_pos, const Eigen::Vector3d &target_vel,
    const Eigen::Vector3d &target_acc)
{

    /// Compute BodyRate commands using differential flatness
    /// Controller based on Faessler 2017
    const Eigen::Vector3d a_ref = target_acc;
    if (velocity_yaw_) {
        mavYaw_ = computeYawFromVelocity(mavVel_);
    }

    const Eigen::Quaterniond q_ref = acc2quaternion(a_ref - g_, mavYaw_);
    const Eigen::Matrix3d R_ref = q_ref.toRotationMatrix();

    const Eigen::Vector3d pos_error = mavPos_ - target_pos;
    const Eigen::Vector3d vel_error = mavVel_ - target_vel;

    // Position and vel Controller
    const Eigen::Vector3d a_fb = posControllerPID(pos_error, vel_error);

    // Rotor Drag compensation
    const Eigen::Vector3d a_rd = R_ref * D_.asDiagonal() * R_ref.transpose() * target_vel;  // Rotor drag

    // Reference acceleration
    //const Eigen::Vector3d a_des = a_fb + a_ref - a_rd - g_;
    const Eigen::Vector3d a_des = a_fb + a_ref - g_;

    return a_des;

}

Eigen::Vector3d geomControlBase::posControllerPID(const Eigen::Vector3d &pos_error, const Eigen::Vector3d &vel_error) {
    if (!poscontrol_initialized_) {
        poscontrol_now_ = std::chrono::steady_clock::now();
        poscontrol_last_ = poscontrol_now_;
        poscontrol_initialized_ = true;
        // First call: don't integrate, return basic controller
        error_pose_I.setZero();
    } else {
        poscontrol_last_ = poscontrol_now_;
        poscontrol_now_ = std::chrono::steady_clock::now();

        std::chrono::duration<double> duration = poscontrol_now_ - poscontrol_last_;
        double poscontrol_dt = duration.count();  // seconds

        error_pose_I = error_pose_I + poscontrol_dt * pos_error;
    }
  
    // Eigen::Vector3d a_fb =
    //     Kpos_.asDiagonal() * pos_error + Kvel_.asDiagonal() * vel_error;  // feedforward term for trajectory error
  
    // Added I controller
    Eigen::Vector3d a_fb =
        Kpos_.asDiagonal() * pos_error + Kvel_.asDiagonal() * vel_error + KposI_.asDiagonal()* error_pose_I;  // feedforward term for trajectory error
  

    //Eigen::Vector3d coneffort_vel = Kvel_.asDiagonal() * vel_error;
  
    // ROS_DEBUG_STREAM("vel control effort is " << coneffort_vel.transpose());
    if (a_fb.norm() > max_fb_acc_)
      a_fb = (max_fb_acc_ / a_fb.norm()) * a_fb;  // Clip acceleration if reference is too large
  
    return a_fb;
  }



void geomControlBase::computeControlInputs(const Eigen::Vector3d &a_des) {
    // three different contoller are here
    // attitude controller
    // bodyrate controller

    if(control_mode_ == ControlMode::AttitudeControl) 
        {
            // if attitude control is chosen
            // control_input = thrust + attitude 
            control_input_ = computeThrustAttitudeCmd(a_des, mavYaw_);
        }
    else if (control_mode_ == ControlMode::BodyrateControl)
        {
            // if attitude control is chosen
            // first compute reference attitude from desired acc and yaw angle
            Eigen::Quaterniond ref_att = computeRefAttitudeQuaterion(a_des, mavYaw_);
            // then compute control_input: control_input = thrust + bodyrate
            // different control methods are defined by bodyrate_method_ that has a type of enum class BodyrateControlMethod {Geometric, Basic};
            control_input_ = computeThurstBodyrateCmd(ref_att, a_des, mavAtt_);
        }

    if (control_input_.valueless_by_exception()) [[unlikely]] {
        std::cerr << "[ERROR] Control input is valueless due to exception.\n";
        return;
    }

}

// compute thrust and attitude cmd from target linear acceleration
geomControlBase::ThrustAttitude geomControlBase::computeThrustAttitudeCmd(const Eigen::Vector3d &ref_acc, const double &current_yaw)
{
    geomControlBase::ThrustAttitude thrust_attitude_cmd;

    // compute reference attitude
    Eigen::Quaterniond ref_attitude_quat = computeRefAttitudeQuaterion(ref_acc,current_yaw);

    thrust_attitude_cmd.attitude =ref_attitude_quat;

    // compute thrust considering limit
    Eigen::Matrix3d rotmat = quat2RotMatrix(mavAtt_);
    const Eigen::Vector3d zb = rotmat.col(2);
    thrust_attitude_cmd.thrust =
        std::max(0.0, std::min(0.8, norm_thrust_const_ * ref_acc.dot(zb) + norm_thrust_offset_));  // Calculate thrust

    return thrust_attitude_cmd;
}

//compute target attitude from target linear acceleration
Eigen::Quaterniond geomControlBase::computeRefAttitudeQuaterion(const Eigen::Vector3d &ref_acc, const double &current_yaw)
{
    Eigen::Quaterniond ref_attitude_quat;

    ref_attitude_quat = acc2quaternion(ref_acc, current_yaw);

    return ref_attitude_quat;
}

// compute thrust and bodyrate cmd from target linear acceleration and target attitude
geomControlBase::ThrustBodyrate geomControlBase::computeThurstBodyrateCmd(const Eigen::Quaterniond &ref_att, const Eigen::Vector3d &ref_acc, const Eigen::Vector4d &curr_att)
{
    geomControlBase::ThrustBodyrate thrust_bodyrate_cmd;
          if (bodyrate_method_ == BodyrateControlMethod::Geometric) {
            thrust_bodyrate_cmd = geometricAttcontroller(ref_att, ref_acc, curr_att);  // Calculate BodyRate

          } else {
            thrust_bodyrate_cmd = attController(ref_att, ref_acc, curr_att);  // Calculate BodyRate
          }

          return thrust_bodyrate_cmd;
}

geomControlBase::ThrustBodyrate geomControlBase::geometricAttcontroller(const Eigen::Quaterniond &ref_att, const Eigen::Vector3d &ref_acc,
                                                         const Eigen::Vector4d &curr_att) {
    // Geometric attitude controller
    // Attitude error is defined as in Lee, Taeyoung, Melvin Leok, and N. Harris McClamroch. "Geometric tracking control
    // of a quadrotor UAV on SE (3)." 49th IEEE conference on decision and control (CDC). IEEE, 2010.
    // The original paper inputs moment commands, but for offboard control, angular rate commands are sent
    geomControlBase::ThrustBodyrate thrust_bodyrate_cmd;
    // Eigen::Vector4d ratecmd;
    Eigen::Matrix3d rotmat;    // Rotation matrix of current attitude
    Eigen::Matrix3d rotmat_d;  // Rotation matrix of desired attitude
    Eigen::Vector3d error_att;

    rotmat = quat2RotMatrix(curr_att);
    rotmat_d = ref_att.toRotationMatrix();

    error_att = 0.5 * matrix_hat_inv(rotmat_d.transpose() * rotmat - rotmat.transpose() * rotmat_d);
    thrust_bodyrate_cmd.bodyrate = (2.0 / attctrl_tau_) * error_att;
    rotmat = quat2RotMatrix(mavAtt_);
    const Eigen::Vector3d zb = rotmat.col(2);
    thrust_bodyrate_cmd.thrust =
        std::max(0.0, std::min(0.8, norm_thrust_const_ * ref_acc.dot(zb) + norm_thrust_offset_));  // Calculate thrust


    return thrust_bodyrate_cmd;
}

// Eigen::Vector4d geomControlBase::geometric_attcontroller(const Eigen::Vector4d &ref_att, const Eigen::Vector3d &ref_acc,
//     Eigen::Vector4d &curr_att) {
//     // Geometric attitude controller
//     // Attitude error is defined as in Lee, Taeyoung, Melvin Leok, and N. Harris McClamroch. "Geometric tracking control
//     // of a quadrotor UAV on SE (3)." 49th IEEE conference on decision and control (CDC). IEEE, 2010.
//     // The original paper inputs moment commands, but for offboard control, angular rate commands are sent

//     Eigen::Vector4d ratecmd;
//     Eigen::Matrix3d rotmat;    // Rotation matrix of current attitude
//     Eigen::Matrix3d rotmat_d;  // Rotation matrix of desired attitude
//     Eigen::Vector3d error_att;

//     rotmat = quat2RotMatrix(curr_att);
//     rotmat_d = quat2RotMatrix(ref_att);

//     error_att = 0.5 * matrix_hat_inv(rotmat_d.transpose() * rotmat - rotmat.transpose() * rotmat_d);
//     ratecmd.head(3) = (2.0 / attctrl_tau_) * error_att;
//     rotmat = quat2RotMatrix(mavAtt_);
//     const Eigen::Vector3d zb = rotmat.col(2);
//     ratecmd(3) =
//     std::max(0.0, std::min(0.8, norm_thrust_const_ * ref_acc.dot(zb) + norm_thrust_offset_));  // Calculate thrust

//     // ROS_DEBUG_STREAM("ref_acc is " << ref_acc.transpose());
//     // ROS_DEBUG_STREAM("zb is " << zb.transpose());
//     // ROS_DEBUG_STREAM("ref_acc dot zb is " << ref_acc.dot(zb));
//     // ROS_DEBUG_STREAM("thurst is " << ratecmd(3));

//     return ratecmd;
// }

geomControlBase::ThrustBodyrate geomControlBase::attController(const Eigen::Quaterniond &ref_att, const Eigen::Vector3d &ref_acc,
    const Eigen::Vector4d &curr_att) {
    // Geometric attitude controller
    // Attitude error is defined as in Brescianini, Dario, Markus Hehn, and Raffaello D'Andrea. Nonlinear quadrocopter
    // attitude control: Technical report. ETH Zurich, 2013.

    // Eigen::Vector4d ratecmd;
    geomControlBase::ThrustBodyrate thrust_bodyrate_cmd;

    const Eigen::Vector4d inverse(1.0, -1.0, -1.0, -1.0);
    const Eigen::Vector4d q_inv = inverse.asDiagonal() * curr_att;
    const Eigen::Vector4d ref_att_eigen = Eigen::Vector4d(ref_att.w(), ref_att.x(), ref_att.y(), ref_att.z());
    const Eigen::Vector4d qe = quatMultiplication(q_inv, ref_att_eigen);
    thrust_bodyrate_cmd.bodyrate(0) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(1);
    thrust_bodyrate_cmd.bodyrate(1) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(2);
    thrust_bodyrate_cmd.bodyrate(2) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(3);
    const Eigen::Matrix3d rotmat = quat2RotMatrix(mavAtt_);
    const Eigen::Vector3d zb = rotmat.col(2);
    thrust_bodyrate_cmd.thrust =
    std::max(0.0, std::min(0.8, norm_thrust_const_ * ref_acc.dot(zb) + norm_thrust_offset_));  // Calculate thrust

    return thrust_bodyrate_cmd;
}


void geomControlBase::setPostControlPGains(const Eigen::Vector3d &Kpos){Kpos_ = Kpos;};
void geomControlBase::setPostControlDGains(const Eigen::Vector3d &Kvel){Kvel_ = Kvel;};
void geomControlBase::setPostControlIGains(const Eigen::Vector3d &KposI){KposI_ = KposI;};

void geomControlBase::setAttitudeControllerGain(const double &attitude_gain){attctrl_tau_ = attitude_gain;};


Eigen::Vector3d geomControlBase::targetPosition() const {return targetPos_;};
Eigen::Vector3d geomControlBase::targetVelocity() const {return targetVel_;};
Eigen::Vector3d geomControlBase::targetAcceleration() const {return targetAcc_;};


void geomControlBase::setTakeoffHeightAndTime(const double &takeoff_height, const double &takeoff_time)
{
    takeoff_height_ = takeoff_height;
    takeoff_time_ = takeoff_time;
}


Eigen::Quaterniond geomControlBase::acc2quaternion(const Eigen::Vector3d &vector_acc, const double &yaw) {
    Eigen::Quaterniond quat;

    Eigen::Vector4d quat_eigen;
    Eigen::Vector3d zb_des, yb_des, xb_des, proj_xb_des;
    Eigen::Matrix3d rotmat;
  
    proj_xb_des << std::cos(yaw), std::sin(yaw), 0.0;
  
    zb_des = vector_acc / vector_acc.norm();
    yb_des = zb_des.cross(proj_xb_des) / (zb_des.cross(proj_xb_des)).norm();
    xb_des = yb_des.cross(zb_des) / (yb_des.cross(zb_des)).norm();
  
    rotmat << xb_des(0), yb_des(0), zb_des(0), xb_des(1), yb_des(1), zb_des(1), xb_des(2), yb_des(2), zb_des(2);
    quat_eigen = rot2Quaternion(rotmat);

    quat = Eigen::Quaterniond(quat_eigen[0], quat_eigen[1], quat_eigen[2], quat_eigen[3]);

    quat.normalize();

    return quat;
  }
  
/*-----------------------Setting functions-------------------------------------*/
void geomControlBase::setMissionState(const MissionState &state)
{
    mission_state_ = state;
}

void geomControlBase::setMaxFeedbackAcceleration(const double &max_acc)
{
    max_fb_acc_ = max_acc;
}

void geomControlBase::setFlightArmingState(const FlightArmingState &flight_arming_state)
{
    flight_arming_state_ = flight_arming_state;
}

void geomControlBase::setFlightOffboardState(const FlightOffboardState &flight_offboard_state)
{
    flight_offboard_state_ = flight_offboard_state;
}

void geomControlBase::setVelocityYawMode(const bool &enable) 
{ 
    velocity_yaw_ = enable; 
}

void geomControlBase::setThrustParameters(const double &thrust_const, const double &thrust_offset)
{
    norm_thrust_const_ = thrust_const;
    norm_thrust_offset_ = thrust_offset;
}

void geomControlBase::setControlMode(const ControlMode &mode)
{
    control_mode_ = mode;
}

void geomControlBase::setDragCoefficients(const Eigen::Vector3d &drag_coeffs)
{
    D_ = drag_coeffs;
}

/*-----------------------variables returned-------------------------------------*/
Eigen::Vector3d geomControlBase::homePosition() const
{
    return home_position_;
}

Eigen::Vector3d geomControlBase::mavPost() const
{
    return  mavPos_;
}
Eigen::Vector3d geomControlBase::mavVel() const
{
    return mavVel_;
}
Eigen::Vector4d geomControlBase::mavAtt() const
{
    return mavAtt_;
}
Eigen::Vector3d geomControlBase::mavRate() const
{
    return mavRate_;
}

bool geomControlBase::isHomePositionSet() const  
{ 
    return home_position_set_; 
} 

Eigen::Vector3d geomControlBase::takeoffTargetPosition() const 
{ 
    return home_position_ + Eigen::Vector3d(0, 0, 1) * takeoff_height_; 
};