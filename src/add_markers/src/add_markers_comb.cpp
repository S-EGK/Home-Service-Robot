#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

// Define pick_up and drop_off goals
float pickup_x = 0.0;
float pickup_y = 2.0;
float dropoff_x = -6.0;
float dropoff_y = 2.0;

bool pick_up = false;
bool drop_off = false;

float dist_error = 0.3;

// Define callback function for odometry
void odom_callback(const nav_msgs::Odometry::ConstPtr& msg) 
{
	// Retrieve the robot's current position from the odometry message
	float current_x = msg->pose.pose.position.x;
	float current_y = msg->pose.pose.position.y;
	
	// Calculate distance from the pickup and dropoff location
	float pickup_dist = sqrt(pow((pickup_x - current_x), 2) + pow((pickup_y - current_y), 2));
	float dropoff_dist = sqrt(pow((dropoff_x - current_x), 2) + pow((dropoff_y - current_y), 2));
	
	// If the robot is at the pickup location and hasn't picked up the object yet
	if(!pick_up && pickup_dist <= dist_error) {
		pick_up = true;
		("Arrived at the pick up zone");
	}

	// If the robot has picked up the object and is at the drop-off location
	if(pick_up && !drop_off && dropoff_dist <= dist_error)
	{
		drop_off = true;
		ROS_INFO("Arrived at the drop off zone");
	}
}

int main( int argc, char** argv )
{
	ros::init(argc, argv, "add_markers");
	ros::NodeHandle n;
	ros::Rate r(1);
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

	// Subscribe to odometry values
	ros::Subscriber odometry_sub = n.subscribe("/odom", 1000, odom_callback);

	// Set our initial shape type to be a cube
	uint32_t shape = visualization_msgs::Marker::CUBE;

	while (ros::ok())
	{
		visualization_msgs::Marker marker;
		// Set the frame ID and timestamp. See the TF tutorials for information on these.
		marker.header.frame_id = "map";
		marker.header.stamp = ros::Time::now();

		// Set the namespace and id for this marker.  This serves to create a unique ID
		// Any marker sent with the same namespace and id will overwrite the old one
		marker.ns = "add_markers";
		marker.id = 0;

		// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
		marker.type = shape;

		// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
		marker.pose.position.x = pickup_x;
		marker.pose.position.y = pickup_y;
		marker.pose.position.z = 0.0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;

		// Set the scale of the marker -- 1x1x1 here means 1m on a side
		marker.scale.x = 0.1;
		marker.scale.y = 0.1;
		marker.scale.z = 0.1;

		// Set the color -- be sure to set alpha to something non-zero!
		marker.color.r = 0.0f;
		marker.color.g = 0.0f;
		marker.color.b = 1.0f;
		marker.color.a = 1.0;

		marker.lifetime = ros::Duration();

		// Publish the marker
		while (marker_pub.getNumSubscribers() < 1)
		{
			if (!ros::ok())
			{
			return 0;
			}
			ROS_WARN_ONCE("Please create a subscriber to the marker");
			sleep(1);
		}

		// Set the marker action: ADD
		marker.action = visualization_msgs::Marker::ADD;
		if(!pick_up) {
			ROS_INFO("Object ready to be picked up");
		}
		
		if(pick_up) {
			// Set the marker action: DELETE
			marker.action = visualization_msgs::Marker::DELETE;
			ROS_INFO("Object picked up and is being transported");
		}
	
		if(drop_off) {
			// Set the pose of the marker
			marker.pose.position.x = dropoff_x;
			marker.pose.position.y = dropoff_y;
			marker.pose.orientation.w = 1.0;
			// Set the marker action: ADD
			marker.action = visualization_msgs::Marker::ADD;
			ROS_INFO("Object droped off");
		}

		marker_pub.publish(marker);
		
		if(drop_off) {
			return 0;
		}
		ros::spinOnce();
	}
	
	return 0;
}
