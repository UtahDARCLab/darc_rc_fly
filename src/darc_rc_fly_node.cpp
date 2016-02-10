#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/RCIn.h>
#include <iostream>

float roll, pitch, yaw , thrust;
void rc_callback(const mavros_msgs::RCIn& rc_msg_in) { 
  roll   = rc_msg_in.channels[0];
  pitch  = rc_msg_in.channels[1];
  yaw    = rc_msg_in.channels[3];
  thrust = rc_msg_in.channels[2];
}  // rc_callback

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "darc_rc_fly_node");
  ros::NodeHandle node;
  ros::Rate loop_rate(500);

  ros::Subscriber rc_sub;
  rc_sub = node.subscribe("/mavros/rc/in",1,rc_callback);

  ros::Publisher desired_u_pub;
  desired_u_pub = node.advertise<geometry_msgs::Twist>("desired_u",1);

  geometry_msgs::Twist desired_u;
  float max_range = 1900.0, min_range = 1100.0;

  while (ros::ok()) {
    ros::spinOnce();

    desired_u.linear.z  = (2.0 * (thrust - min_range) / (max_range - min_range)) - 1;
    if (desired_u.linear.z < -1.0)
      desired_u.linear.z = -1.0;
    else if (desired_u.linear.z > 1.0)
      desired_u.linear.z = 1.0;

    desired_u.angular.x = -1.0* (2.0 * (max_range - roll) / (max_range - min_range) - 1);
    if (desired_u.angular.x < -1.0)
      desired_u.angular.x = -1.0;
    else if (desired_u.angular.x > 1.0)
      desired_u.angular.x = 1.0;

    desired_u.angular.y = -1.0*(1 - 2.0 * (max_range - pitch) / (max_range - min_range));
    if (desired_u.angular.y < -1.0)
      desired_u.angular.y = -1.0;
    else if (desired_u.angular.y > 1.0)
      desired_u.angular.y = 1.0;

    desired_u.angular.z = 2.0*(max_range - yaw) / (max_range - min_range) - 1;
    if (desired_u.angular.z < -1.0)
      desired_u.angular.z = -1.0;
    else if (desired_u.angular.z > 1.0)
      desired_u.angular.z = 1.0;

    desired_u_pub.publish(desired_u);
    loop_rate.sleep();
  }
  return 0;
}

