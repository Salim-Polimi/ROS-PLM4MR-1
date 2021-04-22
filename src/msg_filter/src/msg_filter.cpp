#include "ros/ros.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <nav_msgs/Odometry.h>
#include <robotics_hw1/MotorSpeed.h>
#include <msg_filter/SpeedAndOdom.h>

ros::Publisher chatter_pub;

void callback(const robotics_hw1::MotorSpeed::ConstPtr& msg1_speed,
              const robotics_hw1::MotorSpeed::ConstPtr& msg2_speed,
              const robotics_hw1::MotorSpeed::ConstPtr& msg3_speed,
              const robotics_hw1::MotorSpeed::ConstPtr& msg4_speed,
              const nav_msgs::Odometry::ConstPtr& msg5_odom) {
   ROS_INFO ("Received: (%f) and (%f) and (%f) and (%f)",  msg1_speed->rpm,  msg2_speed->rpm,  msg3_speed->rpm,  msg4_speed->rpm);
  msg_filter::SpeedAndOdom SyncMsg;
  SyncMsg.header = msg1_speed->header;
  SyncMsg.rpm_fl = msg1_speed->rpm;
  SyncMsg.rpm_fr = msg2_speed->rpm;
  SyncMsg.rpm_rl = msg3_speed->rpm;
  SyncMsg.rpm_rr = msg4_speed->rpm;
  SyncMsg.odom = *msg5_odom;

  chatter_pub.publish(SyncMsg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "msg_filter_sub");

  ros::NodeHandle n;

  chatter_pub = n.advertise<msg_filter::SpeedAndOdom>("sync_msgs", 1000);

  message_filters::Subscriber<robotics_hw1::MotorSpeed> sub1(n, "motor_speed_fl", 1);
  message_filters::Subscriber<robotics_hw1::MotorSpeed> sub2(n, "motor_speed_fr", 1);
  message_filters::Subscriber<robotics_hw1::MotorSpeed> sub3(n, "motor_speed_rl", 1);
  message_filters::Subscriber<robotics_hw1::MotorSpeed> sub4(n, "motor_speed_rr", 1);
  message_filters::Subscriber<nav_msgs::Odometry> sub_odom(n, "scout_odom", 1);

  typedef message_filters::sync_policies::ApproximateTime<robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed, nav_msgs::Odometry> MySyncPolicy;

  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub1, sub2, sub3, sub4, sub_odom);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4, _5));

  ros::spin();

  return 0;
}
