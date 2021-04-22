#include "ros/ros.h"
#include "math.h"
#include <nav_msgs/Odometry.h>
#include <msg_filter/SpeedAndOdom.h>
#include <param_estimation/Estimation.h>
#include <odometry.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/TwistStamped.h>
#define M_PI 3.14159265358979323846
#define R 0.1575
#define T_RATIO 0.025974
#define Y0 0.51

template <>
void OdometryNode<geometry_msgs::TwistStamped, msg_filter::SpeedAndOdom>::subCallback(const msg_filter::SpeedAndOdom::ConstPtr& receivedMsg)
{
	// Estimated velocities computation
	rpm_avg_l = - (receivedMsg->rpm_fl + receivedMsg->rpm_rl)/2;  // negato perchè gli assi di rotazione sono opposti
	rpm_avg_r = (receivedMsg->rpm_fr + receivedMsg->rpm_rr)/2;
	Vl_m = (rpm_avg_l*2*M_PI*R)/60;
	Vr_m = (rpm_avg_r*2*M_PI*R)/60;
	Vl_r = Vl_m * T_RATIO;
	Vr_r = Vr_m * T_RATIO;
	// Estimated velocities
	Vx = (Vl_r + Vr_r)/2;
	omega_z = (-Vl_r+Vr_r)/(2*Y0);
	// Estimated velocities message construction
	twist_msg.header = receivedMsg->header;
	twist_msg.twist.linear.x = Vx;
	twist_msg.twist.angular.z = omega_z;
	// Estimated velocities publication
  twist_publisher.publish(twist_msg);

  // integration
	// Ts computation
	receivedTime = receivedMsg->header.stamp.toSec();
	if (FirstExec==true) {
		previousTime = receivedTime;
		Ts = 0.02; // a value of Ts that it's not far from the average
		FirstExec = false;
	}else{
		Ts = receivedTime - previousTime;
		previousTime = receivedTime;
	}
	// Euler integration
	/*
	x = x + Vx*Ts*cos(theta);
	y = y + Vx*Ts*sin(theta);
	theta = theta + omega_z*Ts;
	*/
  // Runge-Kutta integration
	//
	x = x + Vx*Ts*cos(theta+(omega_z*Ts)/2);
	y = y + Vx*Ts*sin(theta+(omega_z*Ts)/2);
	theta = theta + omega_z*Ts;
	//
	// Odometry message construction
	nav_msgs::Odometry odom_msg;
	odom_msg.header = receivedMsg->header;
	odom_msg.pose.pose.position.x = x;
	odom_msg.pose.pose.position.y = y;
	// Orientation is missing! must be added later
	// Odometry publication
	odometry_publisher.publish(odom_msg);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "odometry_node");
	OdometryNode<geometry_msgs::TwistStamped, msg_filter::SpeedAndOdom> odometry("est_velocities","sync_msgs",1);
	ros::spin();
}
