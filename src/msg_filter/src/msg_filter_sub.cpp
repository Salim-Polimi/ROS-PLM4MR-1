#include "ros/ros.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <msg_filter/MotorSpeed.h>


void callback(const msg_filter::MotorSpeed::ConstPtr& msg1, 
              const msg_filter::MotorSpeed::ConstPtr& msg2,
              const msg_filter::MotorSpeed::ConstPtr& msg3, 
              const msg_filter::MotorSpeed::ConstPtr& msg4) {
  ROS_INFO ("Received two messages: (%f) and (%f) and (%f) and (%f)", 
    msg1->rpm,
    msg2->rpm,
    msg3->rpm,
    msg4->rpm); 
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "msg_filter_sub");

  ros::NodeHandle n;

  message_filters::Subscriber<msg_filter::MotorSpeed> sub1(n, "motor_speed_fl", 1);
  message_filters::Subscriber<msg_filter::MotorSpeed> sub2(n, "motor_speed_fr", 1);
  message_filters::Subscriber<msg_filter::MotorSpeed> sub3(n, "motor_speed_rl", 1);
  message_filters::Subscriber<msg_filter::MotorSpeed> sub4(n, "motor_speed_rr", 1);
  
  //typedef message_filters::sync_policies::ExactTime<geometry_msgs::Vector3Stamped, geometry_msgs::Vector3Stamped> MySyncPolicy;
  typedef message_filters::sync_policies::ApproximateTime<msg_filter::MotorSpeed, msg_filter::MotorSpeed, msg_filter::MotorSpeed, msg_filter::MotorSpeed> MySyncPolicy;
  
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub1, sub2, sub3, sub4);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));

  ros::spin();

  return 0;
}
