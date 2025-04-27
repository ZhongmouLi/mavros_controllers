#include "geometric_controller/geom_control_base.hpp"




const Eigen::Vector3d geomControlBase::g_{0.0, 0.0, -9.8}; // gravity vector

geomControlBase::~geomControlBase() {
    // Destructor
  }

geomControlBase::geomControlBase():fail_detec_(false), ctrl_enable_(true), landing_commanded_(false), feedthrough_enable_(false)
{
    // Destructor
    node_state_ = WAITING_FOR_HOME_POSE;
    home_position_set_ = false;
}

  


// set home position
void geomControlBase::setHomePosition(const Eigen::Vector3d &home_position) {
    home_position_ = home_position;
    targetPos_ = home_position_; // set home position as initial target position
    home_position_set_ = true;
}



// input target position, velocity and acceleration
void geomControlBase::inputTargetPostionVelAcc(const Eigen::Vector3d &target_pos, const Eigen::Vector3d &target_vel,
    const Eigen::Vector3d &target_acc) {
    targetPos_ = target_pos;
    targetVel_ = target_vel;
    targetAcc_ = target_acc;
}

// input previous target position and velocity
void geomControlBase::updatePreviousTargePostionVelAconst(Eigen::Vector3d &target_pos_previous, const Eigen::Vector3d &target_vel_previous) {
    targetPos_prev_ = target_pos_previous;
    targetVel_prev_ = target_vel_previous;
}

// input targe yaw angle 
void geomControlBase::inputTargeYawAngle(const double &targe_yaw) {
    mavYaw_ = targe_yaw;
}




// update mav with current position and attitude   
void geomControlBase::updateMavPostionAttitude(const Eigen::Vector3d &mav_pos, const Eigen::Vector4d &mav_att) {
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
void geomControlBase::doPreTakeoff(){

    // ROS_INFO_STREAM_THROTTLE(2, "home_position is "<< home_position_);
    if (!home_position_set_) {
        Eigen::Vector3d desired_acc = controlPosition(home_position_, Eigen::MatrixXd::Zero(3, 1), Eigen::MatrixXd::Zero(3, 1));
    
        computeBodyRateCmd(cmdBodyRate_, desired_acc);
    }
    else {
        std::cout << "home position is not set" << std::endl;
    }
        

    // if( (current_state_.mode == "OFFBOARD") && current_state_.armed)
    // {
    //       node_state = MISSION_EXECUTION;
    // }

}

// do mission execution
// this function is called when the drone is in mission execution state
// it will calculate the control input based on the target position, velocity and acceleration
void geomControlBase::doExecteMission() {

    Eigen::Vector3d desired_acc;
  
    if (feedthrough_enable_) {
        desired_acc = targetAcc_;
    } else {
        desired_acc = controlPosition(targetPos_, targetVel_, targetAcc_);
    }

    computeBodyRateCmd(cmdBodyRate_, desired_acc);
    
}



// controlPosition calculates desired acc from reference position, vel and acc.
Eigen::Vector3d geomControlBase::controlPosition(const Eigen::Vector3d &target_pos, const Eigen::Vector3d &target_vel,
    const Eigen::Vector3d &target_acc)
{

    /// Compute BodyRate commands using differential flatness
    /// Controller based on Faessler 2017
    const Eigen::Vector3d a_ref = target_acc;
    if (velocity_yaw_) {
        mavYaw_ = getVelocityYaw(mavVel_);
    }

    const Eigen::Vector4d q_ref = acc2quaternion(a_ref - g_, mavYaw_);
    const Eigen::Matrix3d R_ref = quat2RotMatrix(q_ref);

    const Eigen::Vector3d pos_error = mavPos_ - target_pos;
    const Eigen::Vector3d vel_error = mavVel_ - target_vel;

    // Position and vel Controller
    const Eigen::Vector3d a_fb = poscontroller(pos_error, vel_error);

    // Rotor Drag compensation
    const Eigen::Vector3d a_rd = R_ref * D_.asDiagonal() * R_ref.transpose() * target_vel;  // Rotor drag

    // Reference acceleration
    //const Eigen::Vector3d a_des = a_fb + a_ref - a_rd - g_;
    const Eigen::Vector3d a_des = a_fb + a_ref - g_;

    return a_des;

}

Eigen::Vector3d geomControlBase::poscontroller(const Eigen::Vector3d &pos_error, const Eigen::Vector3d &vel_error) {
    // Added I controller

    // ROS timer 
    // poscontrol_last_ = poscontrol_now_;
    // poscontrol_now_ = ros::Time::now();
    
    // poscontrol_dt = (poscontrol_now_ - poscontrol_last_).toSec();
    // error_pose_I = error_pose_I + poscontrol_dt * pos_error;

    // poscontrol_last_ = poscontrol_now_;
    // poscontrol_now_ = std::chrono::steady_clock::now();

    // std::chrono::duration<double> duration = poscontrol_now_ - poscontrol_last_;
    // double poscontrol_dt = duration.count();  // seconds

    // error_pose_I = error_pose_I + poscontrol_dt * pos_error;


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



    // 
  
    // Eigen::Vector3d a_fb =
    //     Kpos_.asDiagonal() * pos_error + Kvel_.asDiagonal() * vel_error;  // feedforward term for trajectory error
  
    // Added I controller
    Eigen::Vector3d a_fb =
        Kpos_.asDiagonal() * pos_error + Kvel_.asDiagonal() * vel_error + KposI_.asDiagonal()* error_pose_I;  // feedforward term for trajectory error
  
    // ROS_DEBUG_STREAM("poseI error is " << error_pose_I.transpose());
    // ROS_DEBUG_STREAM("poseI control effort is " << KposI_.asDiagonal()* error_pose_I);
  
    // ROS_DEBUG_STREAM("pose error is " << pos_error.transpose());
  
    Eigen::Vector3d coneffort_pose = Kpos_.asDiagonal() * pos_error;
    // ROS_DEBUG_STREAM("pose control effort is " << coneffort_pose.transpose());
    // ROS_DEBUG_STREAM("vel error is " << vel_error.transpose());
  
    Eigen::Vector3d coneffort_vel = Kvel_.asDiagonal() * vel_error;
  
    // ROS_DEBUG_STREAM("vel control effort is " << coneffort_vel.transpose());
    if (a_fb.norm() > max_fb_acc_)
      a_fb = (max_fb_acc_ / a_fb.norm()) * a_fb;  // Clip acceleration if reference is too large
  
    return a_fb;
  }


void geomControlBase::computeBodyRateCmd(Eigen::Vector4d &bodyrate_cmd, const Eigen::Vector3d &a_des) {
    // Reference attitude
    q_des = acc2quaternion(a_des, mavYaw_);
  
    // Choose which kind of attitude controller you are running
      if (ctrl_mode_ == ERROR_GEOMETRIC) {
        bodyrate_cmd = geometric_attcontroller(q_des, a_des, mavAtt_);  // Calculate BodyRate
  
      } else {
        bodyrate_cmd = attcontroller(q_des, a_des, mavAtt_);  // Calculate BodyRate
      }
  }


Eigen::Vector4d geomControlBase::geometric_attcontroller(const Eigen::Vector4d &ref_att, const Eigen::Vector3d &ref_acc,
    Eigen::Vector4d &curr_att) {
// Geometric attitude controller
// Attitude error is defined as in Lee, Taeyoung, Melvin Leok, and N. Harris McClamroch. "Geometric tracking control
// of a quadrotor UAV on SE (3)." 49th IEEE conference on decision and control (CDC). IEEE, 2010.
// The original paper inputs moment commands, but for offboard control, angular rate commands are sent

Eigen::Vector4d ratecmd;
Eigen::Matrix3d rotmat;    // Rotation matrix of current attitude
Eigen::Matrix3d rotmat_d;  // Rotation matrix of desired attitude
Eigen::Vector3d error_att;

rotmat = quat2RotMatrix(curr_att);
rotmat_d = quat2RotMatrix(ref_att);

error_att = 0.5 * matrix_hat_inv(rotmat_d.transpose() * rotmat - rotmat.transpose() * rotmat_d);
ratecmd.head(3) = (2.0 / attctrl_tau_) * error_att;
rotmat = quat2RotMatrix(mavAtt_);
const Eigen::Vector3d zb = rotmat.col(2);
ratecmd(3) =
std::max(0.0, std::min(0.8, norm_thrust_const_ * ref_acc.dot(zb) + norm_thrust_offset_));  // Calculate thrust

// ROS_DEBUG_STREAM("ref_acc is " << ref_acc.transpose());
// ROS_DEBUG_STREAM("zb is " << zb.transpose());
// ROS_DEBUG_STREAM("ref_acc dot zb is " << ref_acc.dot(zb));
// ROS_DEBUG_STREAM("thurst is " << ratecmd(3));

return ratecmd;
}


Eigen::Vector4d geomControlBase::attcontroller(const Eigen::Vector4d &ref_att, const Eigen::Vector3d &ref_acc,
    Eigen::Vector4d &curr_att) {
// Geometric attitude controller
// Attitude error is defined as in Brescianini, Dario, Markus Hehn, and Raffaello D'Andrea. Nonlinear quadrocopter
// attitude control: Technical report. ETH Zurich, 2013.

Eigen::Vector4d ratecmd;

const Eigen::Vector4d inverse(1.0, -1.0, -1.0, -1.0);
const Eigen::Vector4d q_inv = inverse.asDiagonal() * curr_att;
const Eigen::Vector4d qe = quatMultiplication(q_inv, ref_att);
ratecmd(0) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(1);
ratecmd(1) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(2);
ratecmd(2) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(3);
const Eigen::Matrix3d rotmat = quat2RotMatrix(mavAtt_);
const Eigen::Vector3d zb = rotmat.col(2);
ratecmd(3) =
std::max(0.0, std::min(0.8, norm_thrust_const_ * ref_acc.dot(zb) + norm_thrust_offset_));  // Calculate thrust

return ratecmd;
}


Eigen::Vector4d geomControlBase::acc2quaternion(const Eigen::Vector3d &vector_acc, const double &yaw) {
    Eigen::Vector4d quat;
    Eigen::Vector3d zb_des, yb_des, xb_des, proj_xb_des;
    Eigen::Matrix3d rotmat;
  
    proj_xb_des << std::cos(yaw), std::sin(yaw), 0.0;
  
    zb_des = vector_acc / vector_acc.norm();
    yb_des = zb_des.cross(proj_xb_des) / (zb_des.cross(proj_xb_des)).norm();
    xb_des = yb_des.cross(zb_des) / (yb_des.cross(zb_des)).norm();
  
    rotmat << xb_des(0), yb_des(0), zb_des(0), xb_des(1), yb_des(1), zb_des(1), xb_des(2), yb_des(2), zb_des(2);
    quat = rot2Quaternion(rotmat);
    return quat;
  }
  