#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <msg_filter/SpeedAndOdom.h>
#include <PubSubNode.h>
#include <std_msgs/Float64.h>


	

template <>
void PubSubNode<nav_msgs::Odometry, msg_filter::SpeedAndOdom>::subCallback(const msg_filter::SpeedAndOdom::ConstPtr& receivedMsg)
{
	//ROS_INFO ("Received: (%f) and (%f) and (%f) and (%f)",  receivedMsg->rpm_fl,  receivedMsg->rpm_fr,  receivedMsg->rpm_rl,  receivedMsg->rpm_rr);
	//double omega_z;
	//double Vl;
	//double Vr;
	//double y0;
	//omega_z = receivedMsg->odom.twist.twist.angular.z;
	//Vl= (((receivedMsg->rpm_fl + receivedMsg->rpm_rl)*6.28*0.1575))/2;
	//Vr= (((receivedMsg->rpm_fr + receivedMsg->rpm_rr)*6.28*0.1575))/2;

	//y0 = (-Vl + Vr)/(2*omega_z);

	//ROS_INFO ("y0 = (%f)", y0);
}	


int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "Odometry_node");
	PubSubNode<nav_msgs::Odometry, msg_filter::SpeedAndOdom> Odometry("pubtopic","sync_msgs",1);
	ros::spin();
}