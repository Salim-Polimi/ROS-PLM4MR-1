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

void VisualizerNode::WheelSbinSubCallback(const msg_filter::SpeedAndOdom::ConstPtr& receivedMsg)
{
  sensor_msgs::JointState jointmsg;
  jointmsg.header = receivedMsg->header;
  jointmsg.name = {"front_right_wheel","front_left_wheel", "rear_left_wheel", "rear_right_wheel"};

  radS_fl = ((receivedMsg->rpm_fl)/60)*2*M_PI;
  radS_fr = ((receivedMsg->rpm_fr)/60)*2*M_PI;
  radS_rl = ((receivedMsg->rpm_rl)/60)*2*M_PI;
  radS_rr = ((receivedMsg->rpm_rr)/60)*2*M_PI;

  if (FirstExec==true) 
  {
    prevT = receivedMsg->header.stamp.toSec();
    deltaT = 0.02; // a value that it's not far from the average
    FirstExec = false;
  }
  else
  {
    deltaT = receivedMsg->header.stamp.toSec() - prevT;
    prevT = receivedMsg->header.stamp.toSec();
  }

  angle_fl = angle_fl + radS_fl * deltaT;
  angle_fr = angle_fr + radS_fr * deltaT;
  angle_rl = angle_rl + radS_rl * deltaT;
  angle_rr = angle_rr + radS_rr * deltaT;

  jointmsg.position =  {angle_fr, angle_fl, angle_rl, angle_rr};

  pubWheelSbin.publish(jointmsg);
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
