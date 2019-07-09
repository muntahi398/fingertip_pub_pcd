#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pcl/point_cloud.h>
#include <visualization_msgs/Marker.h>
#include <cmath>

int main(int argc, char** argv){
	int counter = 0;
	float circleStep = 2 * M_PI / 100;
	ros::init(argc, argv, "markertest");

	ros::NodeHandle node;

	ros::Publisher vis_pub = node.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
	//ros::Publisher turtle_vel = node.advertise<turtlesim::Velocity>("turtle2/command_velocity", 10);

	ros::Rate loop_rate(10);
	while (node.ok()){

		visualization_msgs::Marker marker;
		marker.header.frame_id = "/fingertip_laser_1";
		marker.header.stamp = ros::Time::now();
		marker.ns = "markertestspace";
		marker.id = counter;
		marker.type = visualization_msgs::Marker::CUBE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = 0.001 * counter;
		marker.pose.position.y = cos(counter * circleStep) * (0.30 - 0.001 * counter);
		marker.pose.position.z = sin(counter * circleStep) * (0.30 - 0.001 * counter);
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = 0.01;
		marker.scale.y = 0.01;
		marker.scale.z = 0.01;
		marker.color.a = 1.0;
		marker.color.r = 1.0;
		marker.color.g = 0.0;
		marker.color.b = 0.0;
		marker.lifetime = ros::Duration();
		vis_pub.publish( marker );

		counter = (counter + 1) % 300;

		loop_rate.sleep();
	}
	return 0;
};

