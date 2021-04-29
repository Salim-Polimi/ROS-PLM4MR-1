#include <param_estimation.h>
#include "math.h"
#include <nav_msgs/Odometry.h>
#include <msg_filter/SpeedAndOdom.h>
#include <std_msgs/Float64.h>
#define M_PI 3.14159265358979323846

template <>
void EstimationNode<param_estimation::Estimation, msg_filter::SpeedAndOdom>::subCallback(const msg_filter::SpeedAndOdom::ConstPtr& receivedMsg)
{
	omega_z = receivedMsg->odom.twist.twist.angular.z;
  	Vx = receivedMsg->odom.twist.twist.linear.x;
	rpm_avg_l = - (receivedMsg->rpm_fl + receivedMsg->rpm_rl)/2;
	rpm_avg_r = (receivedMsg->rpm_fr + receivedMsg->rpm_rr)/2;
	Vl_m = (rpm_avg_l*2*M_PI*raggio)/60;
	Vr_m = (rpm_avg_r*2*M_PI*raggio)/60;

  	est.t_ratio = (2*Vx*60)/(rpm_avg_l+rpm_avg_r);
 	est.y0 = (-Vl_m*est.t_ratio + Vr_m*est.t_ratio)/(2*omega_z);

  	publisher.publish(est);
	ROS_INFO ("t_ratio = (%f), y0 = (%f)", est.t_ratio,est.y0);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "param_estimation_node");
	EstimationNode<param_estimation::Estimation, msg_filter::SpeedAndOdom> param_estimation("param_estimation","sync_msgs",1);

	ros::Rate loop_rate(100); //100Hz
	while (ros::ok())
	{
    	ros::spinOnce();
    	loop_rate.sleep();
    }
}
