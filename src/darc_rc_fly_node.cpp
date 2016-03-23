#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/RCIn.h>
#include <iostream>
#include <unistd.h>

float rx, ry, vz, vw;
void rc_callback(const mavros_msgs::RCIn& rc_msg_in) {
  rx = rc_msg_in.channels[0];
  ry = rc_msg_in.channels[1];
  vz = rc_msg_in.channels[2];
  vw = rc_msg_in.channels[3];
}  // rc_callback

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "darc_rc_fly_node");
  ros::NodeHandle node;
  ros::Rate loop_rate(100);

  ros::Subscriber rc_sub;
  rc_sub = node.subscribe("/mavros/rc/in",1,rc_callback);

  ros::Publisher u_pub;
  u_pub = node.advertise<geometry_msgs::Twist>("desired_u",1);
  geometry_msgs::Twist u;

  float max_range = 1900.0, min_range = 1100.0;
  float delta_range = max_range - min_range;

  while (ros::ok()) {
    ros::spinOnce();

    float ax = 2.0 * (max_range - rx) / delta_range - 1.0;
    ax = (ax < -1.0) ? -1.0 : ((ax > 1.0) ? 1.0 : ax);

    float ay = 2.0 * (ry - min_range) / delta_range - 1.0;
    ay = (ay < -1.0) ? -1.0 : ((ay > 1.0) ? 1.0 : ay);

    float az = 2.0 * (max_range - vw) / delta_range - 1.0;
    az = (az < -1.0) ? -1.0 : ((az > 1.0) ? 1.0 : az);

    float lz = 2.0 * (vz - min_range) / delta_range - 1.0;
    lz = (lz < -1.0) ? -1.0 : ((lz > 1.0) ? 1.0 : lz);

    static const float dead_zone = 0.35;
    if (lz < dead_zone && lz > -dead_zone) {
      lz = 0.0;
    } else if (lz < 0.0) {
      lz = (lz + dead_zone) / (1.0 - dead_zone);
    } else {
      lz = (lz - dead_zone) / (1.0 - dead_zone);
    }
    
    u.angular.x = ax;
    u.angular.y = ay;
    u.angular.z = az;
    u.linear.z  = lz;
    u_pub.publish(u);

    loop_rate.sleep();
  }
  return 0;
}

