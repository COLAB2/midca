/*********************************************************************
 * 
 * 
 * For example, consider using PointCloudLab
 * (https://github.com/COLAB2/baxter_pcl/blob/master/src/detect_objects.cpp)
 * 
 * 
 *********************************************************************/

#include <ros/ros.h>


std::string detect_block()
{
	//TODO: create block_attr string with the following attributes:
	//['color', 'z', 'y', 'x']

	return block_attr;
}


int main(int argc, char** argv)
{
	ROS_INFO_STREAM("Starting ../domains/coloredBlocksworld/ros/coloredBlocksworld Entity Detectors node");

	ros::init(argc, argv, "../domains/coloredBlocksworld/ros/coloredBlocksworld_entity_detectors");
	ros::NodeHandle n;

	ros::Publisher block_pub = n.advertise<std_msgs::String>("block_attr", 1);
  
	while (ros::ok())
	{
		block_pub.publish(block_attr);
	
		ros::spinOnce();
	}

	ros::spin();
	return 0;
}