#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/State.h>

#include <iostream>
#include <unistd.h>

float rx, ry, vz, vw;
void rc_callback(const mavros_msgs::RCIn& rc_msg_in) {
  rx = rc_msg_in.channels[0];
  ry = rc_msg_in.channels[1];
  vz = rc_msg_in.channels[2];
  vw = rc_msg_in.channels[3];
}  // rc_callback

bool offboard_mode = false;
void mode_callback(const mavros_msgs::State& state) {
  offboard_mode = !state.mode.compare("OFFBOARD") && state.armed;
}  // mode_callback

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "darc_rc_fly_node");
  ros::NodeHandle node;
  ros::Rate loop_rate(100);

  ros::Subscriber rc_sub;
  rc_sub = node.subscribe("/mavros/rc/in",1,rc_callback);

  ros::Subscriber mode_sub;
  mode_sub = node.subscribe("/mavros/state", 1, mode_callback);

  ros::Publisher u_pub;
  u_pub = node.advertise<geometry_msgs::Twist>("desired_u",1);
  geometry_msgs::Twist u;

  const float max_range = 1900.0, min_range = 1100.0;
  const float delta_range = max_range - min_range;
  const float dead_zone = 0.35;

  double roll_trim;
  if (node.getParam("/roll_trim", roll_trim)) {;} else {
    ROS_ERROR("Set Roll Trim"); return 0; }

  double pitch_trim;
  if (node.getParam("/pitch_trim", pitch_trim)) {;} else {
    ROS_ERROR("Set Pitch Trim"); return 0; }

  double yaw_trim;
  if (node.getParam("/yaw_trim", yaw_trim)) {;} else {
    ROS_ERROR("Set Yaw Trim"); return 0; }

  double throttle_trim;
  if (node.getParam("/throttle_trim", throttle_trim)) {;} else {
    ROS_ERROR("Set Throttle Trim"); return 0; }

  while (ros::ok()) {
    ros::spinOnce();

    if (offboard_mode) {
      u.angular.x = 2.0 * (max_range - rx) / delta_range - 1.0;
      u.angular.x -= roll_trim;
      u.angular.x = (u.angular.x < -1.0) ? -1.0 : ((u.angular.x > 1.0) ? 1.0 : u.angular.x);

      u.angular.y = 2.0 * (ry - min_range) / delta_range - 1.0;
      u.angular.y -= pitch_trim;
      u.angular.y = (u.angular.y < -1.0) ? -1.0 : ((u.angular.y > 1.0) ? 1.0 : u.angular.y);

      u.angular.z = 2.0 * (max_range - vw) / delta_range - 1.0;
      u.angular.z -= yaw_trim;
      u.angular.z = (u.angular.z < -1.0) ? -1.0 : ((u.angular.z > 1.0) ? 1.0 : u.angular.z);

      u.linear.z = 2.0 * (vz - min_range) / delta_range - 1.0;
      u.linear.z -= throttle_trim;
      u.linear.z = (u.linear.z < -1.0) ? -1.0 : ((u.linear.z > 1.0) ? 1.0 : u.linear.z);

      if (u.linear.z < dead_zone && u.linear.z > -dead_zone) {
        u.linear.z = 0.0;
      } else if (u.linear.z < 0.0) {
        u.linear.z = (u.linear.z + dead_zone) / (1.0 - dead_zone);
      } else {
        u.linear.z = (u.linear.z - dead_zone) / (1.0 - dead_zone);
      }
    } else {
      u.angular.x = 0.0;
      u.angular.y = 0.0;
      u.angular.z = 0.0;
      u.linear.z = 0.0;
    }

    u_pub.publish(u);
    loop_rate.sleep();
  }
  return 0;
}

