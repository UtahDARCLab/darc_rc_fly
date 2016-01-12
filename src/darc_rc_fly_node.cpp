#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <roscopter/RC.h>

roscopter::RC rc_in;
void rc_callback(const roscopter::RC& rc_msg_in) { 
  rc_in = rc_msg_in;
}  // rc_callback

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "darc_rc_fly_node");
  ros::NodeHandle node;
  ros::Rate loop_rate(500);

  ros::Subscriber rc_sub;
  rc_sub = node.subscribe("rc",1,rc_callback);

  ros::Publisher desired_u_pub;
  desired_u_pub = node.advertise<geometry_msgs::Twist>("desired_u",1);

  geometry_msgs::Twist desired_u;
  while (ros::ok()) {
    ros::spinOnce();
    desired_u.linear.z = rc_in.channel[2];
    desired_u.angular.x = rc_in.channel[0];
    desired_u.angular.y = rc_in.channel[1];
    desired_u.angular.z = rc_in.channel[3];
    desired_u_pub.publish(desired_u);
    loop_rate.sleep();
  }
  return 0;
}

