#include <msg_filter.h>

#include <nav_msgs/Odometry.h>
#include <robotics_hw1/MotorSpeed.h>
#include <msg_filter/SpeedAndOdom.h>


template<>
void FilterNode<msg_filter::SpeedAndOdom, robotics_hw1::MotorSpeed, nav_msgs::Odometry>::callback(const robotics_hw1::MotorSpeed::ConstPtr& msg1_speed,
                                                                                                  const robotics_hw1::MotorSpeed::ConstPtr& msg2_speed,
                                                                                                  const robotics_hw1::MotorSpeed::ConstPtr& msg3_speed,
                                                                                                  const robotics_hw1::MotorSpeed::ConstPtr& msg4_speed,
                                                                                                  const nav_msgs::Odometry::ConstPtr& msg5_odom) 
{
  // DEBUG
  //ROS_INFO ("Received: (%f) and (%f) and (%f) and (%f)",  msg1_speed->rpm,  msg2_speed->rpm,  msg3_speed->rpm,  msg4_speed->rpm);
  
  msg_filter::SpeedAndOdom SyncMsg;
  SyncMsg.header = msg1_speed->header;
  SyncMsg.rpm_fl = msg1_speed->rpm;
  SyncMsg.rpm_fr = msg2_speed->rpm;
  SyncMsg.rpm_rl = msg3_speed->rpm;
  SyncMsg.rpm_rr = msg4_speed->rpm;
  SyncMsg.odom = *msg5_odom;

  sync_pub.publish(SyncMsg);
}


int main(int argc, char** argv) 
{
  ros::init(argc, argv, "msg_filter_node");
  
  FilterNode<msg_filter::SpeedAndOdom, robotics_hw1::MotorSpeed, nav_msgs::Odometry> Filter("sync_msgs",1000,"motor_speed_fl","motor_speed_fr","motor_speed_rl","motor_speed_rr","scout_odom",1);

  ros::Rate loop_rate(100); //100Hz
  while (ros::ok())
  {
      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;
}
