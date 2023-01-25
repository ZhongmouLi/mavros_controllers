#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float32.h>


int main(int argc, char** argv) {
    ros::init(argc, argv, "geometric_controller_takeoff");
    ros::NodeHandle nh;
    ros::Rate loop_rate(50);  

    ros::Publisher ref_setpoint_pub = nh.advertise<geometry_msgs::TwistStamped>("reference/setpoint", 1);
    ros::Publisher ref_yaw_setpoint_pub = nh.advertise<std_msgs::Float32>("reference/yaw", 1);

    geometry_msgs::TwistStamped takeoff_point;
    std_msgs::Float32 yaw_ref;
    yaw_ref.data = 0.0;

    unsigned int seq = 1;

    takeoff_point.header.seq=seq;
    takeoff_point.header.frame_id="base_link";
    takeoff_point.twist.angular.x = 0.53;
    takeoff_point.twist.angular.y = 0.97;
    takeoff_point.twist.angular.z = 1;

    takeoff_point.twist.linear.x = 0;
    takeoff_point.twist.linear.y = 0;
    takeoff_point.twist.linear.z = 0;


    while (ros::ok())
    {
        takeoff_point.header.stamp = ros::Time::now();
        ref_setpoint_pub.publish(takeoff_point);
        ref_yaw_setpoint_pub.publish(yaw_ref);
        ros::spinOnce();
        loop_rate.sleep();
        ROS_INFO_STREAM_THROTTLE(5, "takeoff and hover at  " << takeoff_point.twist.angular.z);
    }
    
    

}