#include <odometry.h>
#include "math.h"
#include <nav_msgs/Odometry.h>
#include <msg_filter/SpeedAndOdom.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_broadcaster.h>

#define M_PI 3.14159265358979323846



template <>
void OdometryNode<geometry_msgs::TwistStamped, odometry::OdometryAndMethod, msg_filter::SpeedAndOdom>::subCallback(const msg_filter::SpeedAndOdom::ConstPtr& receivedMsg)
{
	// Estimated velocities computation
	rpm_avg_l = - (receivedMsg->rpm_fl + receivedMsg->rpm_rl)/2;  // negato (-)perchè gli assi di rotazione sono opposti
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


	// integration
	switch(int_method)
	{
		case 0:
			// Euler integration
			x = x + Vx*Ts*cos(theta);
			y = y + Vx*Ts*sin(theta);
			theta = theta + omega_z*Ts;
			method.data = "euler";
			break;

		case 1:
			// Runge-Kutta integration
			x = x + Vx*Ts*cos(theta+(omega_z*Ts)/2);
			y = y + Vx*Ts*sin(theta+(omega_z*Ts)/2);
			theta = theta + omega_z*Ts;
			method.data = "rk";
			break;

		default:
			ROS_INFO("invalid integration method");
			method.data = "INVALID!";
			break;
	}

	// Odometry message construction
	odometry::OdometryAndMethod odom_msg;
	odom_msg.odom.header = receivedMsg->odom.header;
	odom_msg.odom.header.frame_id = "odom"; //come da rep105 qui devo mettere l'odom fixed frame, che è static transformata risp la world frame
	odom_msg.odom.child_frame_id = "base_link";
	//world-->map-->odom---dynTF--->base_link ( i primi 2 sono static transform, obv)
	
	odom_msg.odom.pose.pose.position.x = x;
	odom_msg.odom.pose.pose.position.y = y;
	quaternion = tf::createQuaternionMsgFromYaw(theta);
	odom_msg.odom.pose.pose.orientation = quaternion;
	odom_msg.method = method;

	// Odometry publication
	odometry_publisher.publish(odom_msg);

	// tf message construction and sending
	transform.setOrigin( tf::Vector3(x, y, 0) );
  	tf::Quaternion q;
  	q.setRPY(0, 0, theta);
  	transform.setRotation(q);
  	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
}


template<>
void OdometryNode<geometry_msgs::TwistStamped, odometry::OdometryAndMethod, msg_filter::SpeedAndOdom>::responseCallback(odometry::paramConfig &config, uint32_t level)
{
  int_method = config.intMethod;
  ROS_INFO("integration method: %d", int_method);
}


template<>
bool OdometryNode<geometry_msgs::TwistStamped, odometry::OdometryAndMethod, msg_filter::SpeedAndOdom>::resetOdom(odometry::resetOdom::Request  &req, odometry::resetOdom::Response &res)
{
 	x=0;
 	y=0;
 	theta=0;

 	res.x=x;
 	res.y=y;
 	res.theta=theta;

    return true;
}


template<>
bool OdometryNode<geometry_msgs::TwistStamped, odometry::OdometryAndMethod, msg_filter::SpeedAndOdom>::setOdom(odometry::setOdom::Request  &req, odometry::setOdom::Response &res)
{
	x=req.x;
 	y=req.y;
 	theta=req.theta;

 	res.x=x;
 	res.y=y;
 	res.theta=theta;

  return true;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "odometry_node");

	OdometryNode<geometry_msgs::TwistStamped, odometry::OdometryAndMethod, msg_filter::SpeedAndOdom> odometry("est_velocities","est_odometry","sync_msgs",1);

	ros::Rate loop_rate(100); //100Hz
	while (ros::ok())
	{
    	ros::spinOnce();
    	loop_rate.sleep();
  	}

	return 0;
}
