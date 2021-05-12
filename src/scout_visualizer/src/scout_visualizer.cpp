#include <scout_visualizer.h>




#define M_PI 3.14159265358979323846


/*
void VisualizerNode::subCallback(const msg_filter::SpeedAndOdom::ConstPtr& receivedMsg)
{
	// Estimated velocities computation
	rpm_avg_l = - (receivedMsg->rpm_fl + receivedMsg->rpm_rl)/2;  // negato (-)perchè gli assi di rotazione sono opposti
	rpm_avg_r = (receivedMsg->rpm_fr + receivedMsg->rpm_rr)/2;

	//Prepare and send messages
	/*sensor_msgs::JointState jointmsg;
	jointmsg.name = "front_left_wheel";
	jointmsg.position = rpm_avg_l;
	odometry_publisher.publish(odom_msg);

	jointmsg.name = "front_right_wheel";
	jointmsg.position = rpm_avg_r;
	odometry_publisher.publish(odom_msg);

	jointmsg.name = "rear_left_wheel";
	jointmsg.position = rpm_avg_l;
	odometry_publisher.publish(odom_msg);

	jointmsg.name = "rear_right_wheel";
	jointmsg.position = rpm_avg_r;
	odometry_publisher.publish(odom_msg);

	sensor_msgs::JointState jointmsg;
	jointmsg.header = receivedMsg->header;
	jointmsg.name = {"front_right_wheel","front_left_wheel", "rear_left_wheel", "rear_right_wheel"};


	//jointmsg.position =  {rpm_avg_r, rpm_avg_l, rpm_avg_l, rpm_avg_r} ;
	jointmsg.position =  {0.0, 0.0, 0.0, 0.0} ;
	//jointmsg.velocity =  {rpm_avg_r, rpm_avg_l, rpm_avg_l, rpm_avg_r} ;
	joint_publisher.publish(jointmsg);

}
*/
void VisualizerNode::estPathCallback(const typename odometry::OdometryAndMethod::ConstPtr& receivedMsg)
{
	nav_msgs::Path path;
	geometry_msgs::PoseStamped temp;
	temp.pose = receivedMsg->odom.pose.pose;
	path.header = receivedMsg->odom.header;
	path.poses.push_back(temp);
	path_publisher.publish(path);
}

/*
void VisualizerNode::scoutPathCallback(const typename nav_msgs::Odometry::ConstPtr& receivedMsg)
{

}


void VisualizerNode::gtPathCallback(const typename geometry_msgs::PoseStamped::ConstPtr& receivedMsg)
{

}

*/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "scout_visualizer_node");
	//ros::Time::init(); //altrimenti crashava!
	VisualizerNode visualizer();


	ros::Rate loop_rate(10); //100Hz
	while (ros::ok())
	{
    	ros::spinOnce();
    	loop_rate.sleep();
  	}

	return 0;
}

//subbo a est odometry, scout odometry, gt pose. inizio facendo est odometry
//come mai non compare il topic paths?????