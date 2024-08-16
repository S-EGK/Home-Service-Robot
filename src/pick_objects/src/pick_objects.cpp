#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
 
// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
	// Initialize the pick_objects node
	ros::init(argc, argv, "pick_objects");

	//tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);

	// Wait 5 sec for move_base action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
	ROS_INFO("Waiting for the move_base action server to come up");
	}

	// Pick up
	move_base_msgs::MoveBaseGoal pick_up;

	// set up the frame parameters
	pick_up.target_pose.header.frame_id = "map";
	pick_up.target_pose.header.stamp = ros::Time::now();

	// Define a position and orientation of the pick up zone for the robot to reach
	pick_up.target_pose.pose.position.x = 0.0;
	pick_up.target_pose.pose.position.y = 2.0;
	pick_up.target_pose.pose.orientation.w = 1.0;

	// Send the pick_up position and orientation for the robot to reach
	ROS_INFO("Sending pick up location");
	ac.sendGoal(pick_up);

	// Wait an infinite time for the results
	ac.waitForResult();

	// Check if the robot reached the pick_up location
	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("Hooray, the robot has reached the pick up location");
	else
		ROS_INFO("The robot failed to move to the pick up location for some reason");

	ROS_INFO("Picking the object...");
	ros::Duration(5).sleep(); // sleep for 5 seconds
	ROS_INFO("... Pickup finished");

	// Drop off
	move_base_msgs::MoveBaseGoal drop_off;

	// set up the frame parameters
	drop_off.target_pose.header.frame_id = "map";
	drop_off.target_pose.header.stamp = ros::Time::now();

	// Define a position and orientation of the drop_off zone for the robot to reach
	drop_off.target_pose.pose.position.x = -6.0;
	drop_off.target_pose.pose.position.y = 2.0;
	drop_off.target_pose.pose.orientation.w = 1.0;

	// Send the pick_up position and orientation for the robot to reach
	ROS_INFO("Sending drop_off location");
	ac.sendGoal(drop_off);

	// Wait an infinite time for the results
	ac.waitForResult();

	// Check if the robot reached the pick_up location
	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("Hooray, the robot has reached the drop_off location");
	else
		ROS_INFO("The robot failed to move to the drop_off location for some reason");

	// Return to home
	move_base_msgs::MoveBaseGoal return_home;

	// set up the frame parameters
	return_home.target_pose.header.frame_id = "map";
	return_home.target_pose.header.stamp = ros::Time::now();

	// Define a position and orientation of the drop_off zone for the robot to reach
	return_home.target_pose.pose.position.x = 0.0;
	return_home.target_pose.pose.position.y = 0.0;
	return_home.target_pose.pose.orientation.w = 1.0;

	// Send the pick_up position and orientation for the robot to reach
	ROS_INFO("Sending base location");
	ac.sendGoal(return_home);

	// Wait an infinite time for the results
	ac.waitForResult();

	// Check if the robot reached the pick_up location
	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("Hooray, the robot has reached the base location");
	else
		ROS_INFO("The robot failed to move to the base location for some reason");


	return 0;
}
