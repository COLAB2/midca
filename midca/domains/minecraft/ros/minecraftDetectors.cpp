/*********************************************************************
 * 
 * 
 * For example, consider using PointCloudLab
 * (https://github.com/COLAB2/baxter_pcl/blob/master/src/detect_objects.cpp)
 * 
 * 
 *********************************************************************/

#include <ros/ros.h>


std::string detect_stone()
{
	//TODO: create stone_attr string with the following attributes:
	//['craftable', 'breakable']

	return stone_attr;
}

std::string detect_skeleton()
{
	//TODO: create skeleton_attr string with the following attributes:
	//['health', 'armor']

	return skeleton_attr;
}

std::string detect_creeper()
{
	//TODO: create creeper_attr string with the following attributes:
	//['health']

	return creeper_attr;
}

std::string detect_pig()
{
	//TODO: create pig_attr string with the following attributes:
	//['health', 'age']

	return pig_attr;
}

std::string detect_player()
{
	//TODO: create player_attr string with the following attributes:
	//['health', 'armor']

	return player_attr;
}

std::string detect_pickaxe()
{
	//TODO: create pickaxe_attr string with the following attributes:
	//['craftable', 'speed']

	return pickaxe_attr;
}

std::string detect_sword()
{
	//TODO: create sword_attr string with the following attributes:
	//['craftable', 'damage']

	return sword_attr;
}

std::string detect_chicken()
{
	//TODO: create chicken_attr string with the following attributes:
	//['health', 'age']

	return chicken_attr;
}

std::string detect_grass()
{
	//TODO: create grass_attr string with the following attributes:
	//['craftable', 'breakable']

	return grass_attr;
}


int main(int argc, char** argv)
{
	ROS_INFO_STREAM("Starting minecraft/ros/minecraft Entity Detectors node");

	ros::init(argc, argv, "minecraft/ros/minecraft_entity_detectors");
	ros::NodeHandle n;

	ros::Publisher stone_pub = n.advertise<std_msgs::String>("stone_attr", 1);
	ros::Publisher skeleton_pub = n.advertise<std_msgs::String>("skeleton_attr", 1);
	ros::Publisher creeper_pub = n.advertise<std_msgs::String>("creeper_attr", 1);
	ros::Publisher pig_pub = n.advertise<std_msgs::String>("pig_attr", 1);
	ros::Publisher player_pub = n.advertise<std_msgs::String>("player_attr", 1);
	ros::Publisher pickaxe_pub = n.advertise<std_msgs::String>("pickaxe_attr", 1);
	ros::Publisher sword_pub = n.advertise<std_msgs::String>("sword_attr", 1);
	ros::Publisher chicken_pub = n.advertise<std_msgs::String>("chicken_attr", 1);
	ros::Publisher grass_pub = n.advertise<std_msgs::String>("grass_attr", 1);
  
	while (ros::ok())
	{
		stone_pub.publish(stone_attr);
		skeleton_pub.publish(skeleton_attr);
		creeper_pub.publish(creeper_attr);
		pig_pub.publish(pig_attr);
		player_pub.publish(player_attr);
		pickaxe_pub.publish(pickaxe_attr);
		sword_pub.publish(sword_attr);
		chicken_pub.publish(chicken_attr);
		grass_pub.publish(grass_attr);
	
		ros::spinOnce();
	}

	ros::spin();
	return 0;
}