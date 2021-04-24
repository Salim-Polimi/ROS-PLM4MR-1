#include "ros/ros.h"
#include "math.h"
#include <nav_msgs/Odometry.h>
#include <msg_filter/SpeedAndOdom.h>
#include <odometry.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/TwistStamped.h>


#define M_PI 3.14159265358979323846

int int_method;


template <>
void OdometryNode<geometry_msgs::TwistStamped, msg_filter::SpeedAndOdom>::subCallback(const msg_filter::SpeedAndOdom::ConstPtr& receivedMsg)
{
	// Estimated velocities computation
	rpm_avg_l = - (receivedMsg->rpm_fl + receivedMsg->rpm_rl)/2;  // negato perchè gli assi di rotazione sono opposti
	rpm_avg_r = (receivedMsg->rpm_fr + receivedMsg->rpm_rr)/2;
	Vl_m = (rpm_avg_l*2*M_PI*raggio)/60;
	Vr_m = (rpm_avg_r*2*M_PI*raggio)/60;
	Vl_r = Vl_m * t_ratio;
	Vr_r = Vr_m * t_ratio;
	// Estimated velocities
	Vx = (Vl_r + Vr_r)/2;
	omega_z = (-Vl_r+Vr_r)/(2*y0);
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

	switch(int_method)
	{
		case 0:
			// Euler integration
			x = x + Vx*Ts*cos(theta);
			y = y + Vx*Ts*sin(theta);
			theta = theta + omega_z*Ts;
			break;

		case 1:
			// Runge-Kutta integration
	
			x = x + Vx*Ts*cos(theta+(omega_z*Ts)/2);
			y = y + Vx*Ts*sin(theta+(omega_z*Ts)/2);
			theta = theta + omega_z*Ts;
			break;

		default:
			ROS_INFO("invalid integration method");
			break;
	}
	
  
	// Odometry message construction
	nav_msgs::Odometry odom_msg;
	odom_msg.header = receivedMsg->header;
	odom_msg.pose.pose.position.x = x;
	odom_msg.pose.pose.position.y = y;
	// Orientation is missing! must be added later
	// Odometry publication
	odometry_publisher.publish(odom_msg);
}


void responseCallback(odometry::paramConfig &config, uint32_t level) //CHIAMATA quando un parametro cambia?
{
  //printo i valori di tutti i parametri
  
  int_method = config.intMethod;
  ROS_INFO("integration method: %d", int_method);

  //ROS_INFO ("%d",level);
  // LEVEL ti dice il level corrispondente al parametro cambiato
}



template<>
bool OdometryNode<geometry_msgs::TwistStamped, msg_filter::SpeedAndOdom>::resetOdom(odometry::setResetOdom::Request  &req, odometry::setResetOdom::Response &res)
{
 	
 	x=0;
 	y=0;
 	theta=0;


  return true;
}

template<>
bool OdometryNode<geometry_msgs::TwistStamped, msg_filter::SpeedAndOdom>::setOdom(odometry::setResetOdom::Request  &req, odometry::setResetOdom::Response &res)
{
	x=req.x;
 	y=req.y;
 	theta=req.theta;

  return true;
}



int main(int argc, char **argv)
{
ros::init(argc, argv, "odometry_node");

/////////////////METTERE X Y THETA IN UN'UNICA VARIABILE POSE, vedi teams, msg del prof



OdometryNode<geometry_msgs::TwistStamped, msg_filter::SpeedAndOdom> odometry("est_velocities","sync_msgs",1);



//modifiche
dynamic_reconfigure::Server<odometry::paramConfig> server;
dynamic_reconfigure::Server<odometry::paramConfig>::CallbackType f;
f = boost::bind(responseCallback, _1, _2);
server.setCallback(f);




ros::Rate loop_rate(100); //100Hz

	while (ros::ok())
	{

 	//ROS_INFO("%f", y0); 
    	ros::spinOnce();
    	loop_rate.sleep();

  	}


	return 0;
}



