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

	//DEBUG
	//ROS_INFO("int_method STA A %i", int_method);

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
	//odom_msg.odom.header = receivedMsg->header; FIXED, prendeva l'header sbagliato: così perdevo il reference al frame parent di scout odom
	odom_msg.odom.header = receivedMsg->odom.header;
	odom_msg.odom.header.frame_id = "est_odom"; //come da rep105 qui devo mettere l'odom fixed frame, che è static transformata risp la world frame
	odom_msg.odom.child_frame_id = "base_link";
/////////////////////////////
//world-->map-->odom---dynTF--->base_link ( i primi 2 sono static transform, obv)
///////////////////////
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
  	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "est_odom", "base_link"));

  	//tf for visual wheel spinning
  	/*tf::Quaternion q_wheels;
  	q_wheels.setRPY(0, rpm_avg_l/60, 0);
  	transform_fl.setOrigin( tf::Vector3(0.249, 0.29153, -0.0702) );
  	transform_fl.setRotation(q_wheels);
  	q_wheels.setRPY(0, rpm_avg_l/60, 0);
  	transform_fr.setOrigin( tf::Vector3(0.249, -0.29153, -0.0702) );
  	transform_fr.setRotation(q_wheels);
  	q_wheels.setRPY(0, rpm_avg_r/60, 0);
  	transform_rl.setOrigin( tf::Vector3(-0.249, 0.29153, -0.0702) );
  	transform_rl.setRotation(q_wheels);
  	q_wheels.setRPY(0, rpm_avg_r/60, 0);
  	transform_rr.setOrigin( tf::Vector3(-0.249, -0.29153, -0.0702) );
  	transform_rr.setRotation(q_wheels);
  	
  	br.sendTransform(tf::StampedTransform(transform_fl, ros::Time::now(), "base_link", "front_left_wheel_link"));
	br.sendTransform(tf::StampedTransform(transform_fr, ros::Time::now(), "base_link", "front_right_wheel_link"));
	br.sendTransform(tf::StampedTransform(transform_rl, ros::Time::now(), "base_link", "rear_left_wheel_link"));
	br.sendTransform(tf::StampedTransform(transform_rr, ros::Time::now(), "base_link", "rear_right_wheel_link"));
*/
}


template<>
void OdometryNode<geometry_msgs::TwistStamped, odometry::OdometryAndMethod, msg_filter::SpeedAndOdom>::responseCallback(odometry::paramConfig &config, uint32_t level) //CHIAMATA quando un parametro cambia?
{
  //printo i valori di tutti i parametri

  int_method = config.intMethod;
  ROS_INFO("integration method: %d", int_method);

  //ROS_INFO ("%d",level);
  // LEVEL ti dice il level corrispondente al parametro cambiato
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

	//modifiche
	
	

	ros::Rate loop_rate(100); //100Hz
	while (ros::ok())
	{
    	ros::spinOnce();
    	loop_rate.sleep();
  	}

	return 0;
}
