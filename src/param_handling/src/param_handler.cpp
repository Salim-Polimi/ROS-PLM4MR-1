#include "ros/ros.h"

//per dyn reconfigure
#include <param_handling/paramConfig.h>
#include <dynamic_reconfigure/server.h>


#include <sstream>
#include "std_msgs/String.h"
//#include <std_msgs/Float64.h>


void callback(param_handling::paramConfig &config, uint32_t level) //CHIAMATA quando un parametro cambia?
{
  //printo i valori di tutti i parametri
  ROS_INFO("Reconfigure Request: %d", config.intMethod);

  ROS_INFO ("%d",level);
  // LEVEL ti dice il level corrispondente al parametro cambiato
}






int main(int argc, char **argv)
{
	ros::init(argc, argv, "param_handler_node");
	// DEBUG
	//ROS_INFO ("Gentlemen, a short view back to the past. Thirty years ago, Niki Lauda told us ‘take a monkey, place him into the cockpit and he is able to drive the car.’ Thirty years later, Sebastian told us ‘I had to start my car like a computer, it’s very complicated.’ And Nico Rosberg said that during the race – I don’t remember what race - he pressed the wrong button on the wheel. Question for you both: is Formula One driving today too complicated with twenty and more buttons on the wheel, are you too much under effort, under pressure? What are your wishes for the future concerning the technical programme during the race? Less buttons, more? Or less and more communication with your engineers?");
	
  ros::NodeHandle n;


  dynamic_reconfigure::Server<param_handling::paramConfig> server;
  dynamic_reconfigure::Server<param_handling::paramConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);


  double y0;
  n.getParam("/y0", y0);
  ros::Rate loop_rate(10); //Sbinna a 10Hz, prolly
  while (ros::ok())
  {

    //ROS_INFO("%f", y0); 
    
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}
