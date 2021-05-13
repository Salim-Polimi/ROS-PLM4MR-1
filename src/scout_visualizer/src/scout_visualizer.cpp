#include <scout_visualizer.h>



void VisualizerNode::estOdomSubCallback(const odometry::OdometryAndMethod::ConstPtr& receivedMsg) 
{
   nav_msgs::Odometry onlyEstOdom;
   onlyEstOdom = receivedMsg->odom;
   pubEstOdom.publish(onlyEstOdom);
}

void VisualizerNode::scoutOdomSubCallback(const msg_filter::SpeedAndOdom::ConstPtr& receivedMsg) 
{
   nav_msgs::Odometry onlyScoutOdom;
   onlyScoutOdom = receivedMsg->odom;
   pubScoutOdom.publish(onlyScoutOdom);
}

void VisualizerNode::gtPathSubCallback(const geometry_msgs::PoseStamped::ConstPtr& receivedMsg)
{
  nav_msgs::Path path;
  path.header = receivedMsg->header;
  path.header.frame_id = "world";
  path.poses.push_back(*receivedMsg);
  pubGtPath.publish(path);
}


int main(int argc, char** argv) 
{
  ros::init(argc, argv, "scout_visualizer_node");
  
  VisualizerNode Visualizer;


  ros::Rate loop_rate(100); //100Hz
  while (ros::ok())
  {
      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;
}
