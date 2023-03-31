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


    double height_;
    double traveling_time;


    nh_private_.param<double>("take_off_height", height_,0.5);
    nh_private_.param<double>("take_off_time", traveling_time,5);

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
    // waypoints
    Eigen::MatrixXd v_waypoints_t = Eigen::MatrixXd::Zero(7,4);

    // home
    v_waypoints_t.row(0) << q_home(0), q_home(1), q_home(2), traveling_time;
    
    // take off in z
    v_waypoints_t.row(1) << q_home(0), q_home(1), q_home(2) + height_, traveling_time;

    // move to (0.5, 0)
    v_waypoints_t.row(2) << q_home(0)+1, q_home(1), q_home(2) + height_, traveling_time;

    // move to (0.5, 0.5)
    v_waypoints_t.row(3) << q_home(0)+1, q_home(1)+2, q_home(2) + height_, traveling_time;

    // move to (0, 0.5)
    v_waypoints_t.row(4) << q_home(0), q_home(1)+2, q_home(2) + height_, traveling_time;

    // move to (0 ,0)
    v_waypoints_t.row(5) << q_home(0), q_home(1)+2, q_home(2) + height_, traveling_time;

    // land
    v_waypoints_t.row(6) << q_home(0), q_home(1)+2, q_home(2), 0;

    double T_begin = ros::Time::now().toSec();
    
    for (size_t ii = 0; ii < v_waypoints_t.rows()-1; ii++)
    {
        // get current destination from waypoints
        Eigen::VectorXd v_position_start = v_waypoints_t.block<1,3>(ii,0);

        ROS_INFO_STREAM("start posision is "<< v_position_start.transpose());

        Eigen::VectorXd v_position_end = v_waypoints_t.block<1,3>(ii+1,0);
        ROS_INFO_STREAM("end posision is "<< v_position_end.transpose());

        double t_tracking_begin = ros::Time::now().toSec();

        double t_track_step_ = 0;       
        double T_traveling = v_waypoints_t(ii,3);
        ROS_INFO_STREAM("travelling time is "<< T_traveling);

        while (ros::ok() && (t_track_step_ <= T_traveling))
        {       
          Eigen::VectorXd q_des(3);
          Eigen::VectorXd dq_des(3);
          Eigen::VectorXd ddq_des(3);
          // ROS_INFO_STREAM("debug p1 ");

          LibTrajGenerate trajectory_generator(v_position_start,v_position_end,T_traveling);
          t_track_step_ = ros::Time::now().toSec()-t_tracking_begin;

          //  ROS_INFO_STREAM("debug p2 ");
          trajectory_generator.doFifthDegreeTrajectory(t_track_step_);

          //  ROS_INFO_STREAM("debug p3 ");
          trajectory_generator.getTrajectory(q_des, dq_des, ddq_des);     

          // ROS_INFO_STREAM("debug p4 ");

          takeoff_point.twist.angular.x = q_des(0);
          takeoff_point.twist.angular.y = q_des(1);
          takeoff_point.twist.angular.z = q_des(2);
          takeoff_point.twist.linear.x = dq_des(0);
          takeoff_point.twist.linear.y = dq_des(1);
          takeoff_point.twist.linear.z = dq_des(2);      

          takeoff_point.header.stamp = ros::Time::now();
          takeoff_point.header.seq++;

          //  ROS_INFO_STREAM("debug p5 ");
          ref_setpoint_pub.publish(takeoff_point);
          ref_yaw_setpoint_pub.publish(yaw_ref);
          ros::spinOnce();
          loop_rate.sleep();      

          ROS_INFO_STREAM_THROTTLE(1, "current destination is "<<v_position_end.transpose());   
          ROS_INFO_STREAM_THROTTLE(1, "traveling time is "<< ros::Time::now().toSec()-T_begin << "/" << v_waypoints_t.rightCols<1>().sum());          
             
        }
        
    };


    while(ros::ok())
    {
      ROS_INFO_STREAM_THROTTLE(5, "drone reaches final desitination: " << v_waypoints_t.bottomRows<1>());

      takeoff_point.twist.angular.x = v_waypoints_t(v_waypoints_t.rows()-1, 0);
      takeoff_point.twist.angular.y = v_waypoints_t(v_waypoints_t.rows()-1, 1);
      takeoff_point.twist.angular.z = v_waypoints_t(v_waypoints_t.rows()-1, 2);
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
    

}

