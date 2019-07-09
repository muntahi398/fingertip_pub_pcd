#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include "fingercloud_pub.h"



void publish_point_cloud(ros::Publisher &pub_pointcloud, const std::vector<point> &pointVector, ros::Time time) {

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_msg (new pcl::PointCloud<pcl::PointXYZ>() );
	cloud_msg->header.stamp = time;

	cloud_msg->width = pointVector.size();
	cloud_msg->height = 1;
	cloud_msg->is_dense     = false;
	cloud_msg->points.resize (cloud_msg->height * cloud_msg->width);
	cloud_msg->header.frame_id = "/map";

	pcl::PointCloud<pcl::PointXYZ>::iterator pt_iter = cloud_msg->begin ();


	for (unsigned int counter=0 ; counter < cloud_msg-> width ; ++pt_iter) {
		pcl::PointXYZ& pt = *pt_iter;
		// Fill in XYZ
		pt.x = pointVector[counter].x;
		pt.y = pointVector[counter].y;
		pt.z = pointVector[counter].z;
		++counter;
	}

	pub_pointcloud.publish(cloud_msg);
}
