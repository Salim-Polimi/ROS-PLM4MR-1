#include "ros/ros.h"
#include "param_handling/setResetOdom.h"


bool resetOdom(param_handling::setResetOdom::Request  &req, param_handling::setResetOdom::Response &res)
{
  
			//---dal testo non si capisce se è da azzerare anche theta o solo x,y

  return true;
}


bool setOdom(param_handling::setResetOdom::Request  &req, param_handling::setResetOdom::Response &res)
{
  
  
  //res.x=;
  //res.y= req.y;
 // res.theta = ;

  return true;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "set_reset_odom_server");
  ros::NodeHandle n;

  ros::ServiceServer resetService = n.advertiseService("reset_odometry", resetOdom);
  ros::ServiceServer setService = n.advertiseService("set_odometry", setOdom);
  ROS_INFO("Server running");
  ros::spin();

  return 0;
}
