#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float32.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/TransformStamped.h>

#include "librobot_traj_generate/librobot_traj_generate.hpp"

#define RATE 50

static mavros_msgs::State mavrState;
static bool home_position_received = false;
static Eigen::VectorXd q_home(3);


void viconPoseCallback(const geometry_msgs::TransformStamped::ConstPtr& msg_vicon)
{

  if(!home_position_received)
  {
    q_home(0) = msg_vicon->transform.translation.x;
    q_home(1) = msg_vicon->transform.translation.y;
    q_home(2) = msg_vicon->transform.translation.z;
    home_position_received =  true;
    ROS_INFO_STREAM("home position in planner is set as "<< q_home.transpose());
  }
}
void mavStateCallback(const mavros_msgs::State::ConstPtr& msg)
{
  mavrState = *msg;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "geometric_controller_takeoff");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private_("~");
    ros::Rate loop_rate(RATE);  

    ros::Publisher ref_setpoint_pub = nh.advertise<geometry_msgs::TwistStamped>("reference/setpoint", 1);
    ros::Publisher ref_yaw_setpoint_pub = nh.advertise<std_msgs::Float32>("reference/yaw", 1);
    ros::Subscriber sub = nh.subscribe("mavros/state", 1, mavStateCallback);
    ros::Subscriber vicon_sub = nh.subscribe("vicon/drone", 1, viconPoseCallback);

    // double home_x_;
    // double home_y_;
    // double home_z_;
    double height_;
    double traveling_time;

    // nh_private_.param<double>("home_x", home_x_,0);
    // nh_private_.param<double>("home_y", home_y_,0);
    // nh_private_.param<double>("home_z", home_z_,0);
    nh_private_.param<double>("take_off_height", height_,0.5);
    nh_private_.param<double>("take_off_time", traveling_time,5);


    //
    // Eigen::VectorXd q_home(3);
    // q_home(0) = home_x_;
    // q_home(1) = home_y_;
    // q_home(2) = home_z_;

    // Eigen::VectorXd q_destination(3);
    // q_destination(0) = q_home(0);
    // q_destination(1) = q_home(1);
    // q_destination(2) = q_home(2) + height_;


    // LibTrajGenerate trajectory_generator(q_home,q_destination,traveling_time);


    // geometry_msgs::TwistStamped takeoff_point;
    // std_msgs::Float32 yaw_ref;
    // yaw_ref.data = 0.0;

    // unsigned int seq = 1;

    // takeoff_point.header.seq=seq;
    // takeoff_point.header.frame_id="base_link";
    // takeoff_point.twist.angular.x = q_home(0);
    // takeoff_point.twist.angular.y = q_home(1);
    // takeoff_point.twist.angular.z = q_home(2);

    // takeoff_point.twist.linear.x = 0;
    // takeoff_point.twist.linear.y = 0;
    // takeoff_point.twist.linear.z = 0;



    // waiting for 10s

    geometry_msgs::TwistStamped takeoff_point;
    std_msgs::Float32 yaw_ref;
    yaw_ref.data = 0.0;

    unsigned int seq = 1;


    while( (mavrState.armed != true) || (mavrState.mode != mavrState.MODE_PX4_OFFBOARD))
    {
         if(mavrState.armed != true)
         {
          ROS_INFO_THROTTLE(2,"waiting to be armed");
         }

         if(mavrState.mode != mavrState.MODE_PX4_OFFBOARD)
         {
          ROS_INFO_THROTTLE(2, "waiting to be OFFBOARD");
         }

         takeoff_point.header.seq=seq;
         takeoff_point.header.frame_id="base_link";
         takeoff_point.twist.angular.x = q_home(0);
         takeoff_point.twist.angular.y = q_home(1);
         takeoff_point.twist.angular.z = q_home(2);

         takeoff_point.twist.linear.x = 0;
         takeoff_point.twist.linear.y = 0;
         takeoff_point.twist.linear.z = 0;

         takeoff_point.header.stamp = ros::Time::now();
         takeoff_point.header.seq++;
         
         ref_setpoint_pub.publish(takeoff_point);
         ref_yaw_setpoint_pub.publish(yaw_ref);
         ros::spinOnce();
         loop_rate.sleep();
    }    


    // ADD
    Eigen::VectorXd q_destination(3);
    q_destination(0) = q_home(0);
    q_destination(1) = q_home(1);
    q_destination(2) = q_home(2) + height_;


    LibTrajGenerate trajectory_generator(q_home,q_destination,traveling_time);
    // ADD end


    Eigen::VectorXd q_des(3);
    Eigen::VectorXd dq_des(3);
    Eigen::VectorXd ddq_des(3);

    double t_tracking_begin = ros::Time::now().toSec();
    double time_step_tracking = 0;

    while (ros::ok())
    {
        ROS_INFO_THROTTLE(5, "taking off");
        //
        time_step_tracking = ros::Time::now().toSec()-t_tracking_begin; // 0.1s
        trajectory_generator.doFifthDegreeTrajectory(time_step_tracking);
        trajectory_generator.getTrajectory(q_des, dq_des, ddq_des);
        takeoff_point.twist.angular.x = q_des(0);
        takeoff_point.twist.angular.y = q_des(1);
        takeoff_point.twist.angular.z = q_des(2);
        takeoff_point.twist.linear.x = dq_des(0);
        takeoff_point.twist.linear.y = dq_des(1);
        takeoff_point.twist.linear.z = dq_des(2);        
        //     
        takeoff_point.header.stamp = ros::Time::now();
        takeoff_point.header.seq++;

        ref_setpoint_pub.publish(takeoff_point);
        ref_yaw_setpoint_pub.publish(yaw_ref);
        ros::spinOnce();
        loop_rate.sleep();
        // ROS_INFO_STREAM_THROTTLE(5, "takeoff to x: " << takeoff_point.twist.angular.x << " y: "<<takeoff_point.twist.angular.y << " z: "<<takeoff_point.twist.angular.z);
        ROS_INFO_STREAM_THROTTLE(0.5, "takeoff to x: " << takeoff_point.twist.angular.x << " y: "<<takeoff_point.twist.angular.y << " z: "<<takeoff_point.twist.angular.z);
    }
    
    

}