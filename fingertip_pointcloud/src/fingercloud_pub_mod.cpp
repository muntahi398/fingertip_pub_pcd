/* fingertip sensor ROS - Publisher

    Copyright (c) 2011 Alexis Maldonado  <maldonado@tum.edu>, Humberto Alvarez <humalvarez@mytum.de>
    Technische Universitaet Muenchen - Intelligent Autonomous Systems (Prof. Beetz)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
//#include <fingertip_msg/adns.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Range.h>

//#include <fingertip_msg/adns.h>
#include <geometry_msgs/PointStamped.h>

#include "fingercloud_pub.h"
#include <iostream>
#include <bitset>
std::vector<point> pointVector[MAX_FINGER_LASER];
boost::mutex m_mutex;
bool savePoints = false;

//uint32_t sensor_flag =0b0011111000111111; ideal
//uint8_t finger_a=0b11111111,finger_b=0b11111110,finger_a_vert=0b00000011,finger_b_vert=0b00000011;
// change in this place for sensor that are good to have point cloud
uint8_t finger_a=0b11111111,finger_b=0b11111110,finger_a_vert=0b00000011,finger_b_vert=0b00000011;



uint32_t sensor_flag = finger_a | (finger_a_vert  << 8) | (finger_b  << 16) | (finger_b_vert << 24);

//cout << "Hello World" << std::bitset<32>(i32)<< endl;
//void transformPoint(point& p, sensor_msgs::Range&  msg, const tf::TransformListener* listener){
void transformPoint(point& p, boost::shared_ptr<const sensor_msgs::Range> msg, const tf::TransformListener* listener){

  geometry_msgs::PointStamped fingertiplaser_point;
  fingertiplaser_point.header.frame_id = msg->header.frame_id;

  //we'll just use the most recent transform available for our simple example
  fingertiplaser_point.header.stamp = msg->header.stamp;

  //just an arbitrary point in space
  fingertiplaser_point.point.x = p.x;
  fingertiplaser_point.point.y = 0.00+p.y;
  fingertiplaser_point.point.z = 0.0005 + p.z;

  try{
    geometry_msgs::PointStamped base_point;
    listener->waitForTransform("/world", msg->header.frame_id, msg->header.stamp, ros::Duration(0.2));
    listener->transformPoint("/world", fingertiplaser_point, base_point);

	p.x = base_point.point.x;
	p.y = base_point.point.y;
	p.z = base_point.point.z;

    /*ROS_INFO("fingertip_laser: (%.2f, %.2f. %.2f) -----> map: (%.2f, %.2f, %.2f) at time %.2f",
        fingertiplaser_point.point.x, fingertiplaser_point.point.y, fingertiplaser_point.point.z,
        base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());*/
  }
  catch(tf::TransformException& ex){
    ROS_ERROR("Received an exception trying to transform a point from \"%s\" to \"/map\": %s", msg->header.frame_id.c_str(), ex.what());
  }
}

void fingertip_callback(const boost::shared_ptr<const sensor_msgs::Range> msg, const tf::TransformListener* listener)

//void fingertip_callback(const  sensor_msgs::Range&  msg, const tf::TransformListener* listener,const int sens_no)
{
	int sens_no=1;
	if (msg->header.frame_id =="left_fingertip_sensor_s0")
		sens_no=0;
	else if (msg->header.frame_id =="left_fingertip_sensor_s1")
		sens_no=1;
	else if (msg->header.frame_id =="left_fingertip_sensor_s2")
		sens_no=2;
	else if (msg->header.frame_id =="left_fingertip_sensor_s3")
		sens_no=3;
	else if (msg->header.frame_id =="left_fingertip_sensor_s4")
		sens_no=4;
	else if (msg->header.frame_id =="left_fingertip_sensor_s5")
		sens_no=5;

	else if (msg->header.frame_id =="right_fingertip_sensor_s0")
		sens_no=10;
	else if (msg->header.frame_id =="right_fingertip_sensor_s1")
		sens_no=11;
	else if (msg->header.frame_id =="right_fingertip_sensor_s2")
		sens_no=12;
	else if (msg->header.frame_id =="right_fingertip_sensor_s3")
		sens_no=13;
	else if (msg->header.frame_id =="right_fingertip_sensor_s4")
		sens_no=14;
	else if (msg->header.frame_id =="right_fingertip_sensor_s5")
		sens_no=15;
	if (msg->range != 0 && savePoints) {
    	boost::mutex::scoped_lock l(m_mutex);
		//ROS_INFO("Sensor Id: %d\tDist: %d\n", msg->sensor_id, msg->dist_mm);
		point p;
		p.z = (float)msg->range; //can be set to 100 to see bigger change
		p.y = 0.0;
		p.x = 0.0;
		transformPoint(p, msg, listener);
		if (sens_no < MAX_FINGER_LASER)  //msg->sensor_id is uint8, so it must be positive
		{
			pointVector[sens_no].push_back(p);
		}
	}
}

//int sensor_frame_to_no()
//{
//	fingertiplaser_point.header.frame_id = msg->header.frame_id;
//
//}


//rostopic pub -1 /fingertip/command std_msgs/String reset
//rostopic pub -1 /fingertip/command std_msgs/String stop
//rostopic pub -1 /fingertip/command std_msgs/String start
void commandCallback(const std_msgs::String::ConstPtr& msg)
{
	boost::mutex::scoped_lock l(m_mutex);
	if (msg->data.compare("reset") == 0)
	{
		printf("reset command received\n");
		for (int i = 0; i < MAX_FINGER_LASER; i++)
		{
			pointVector[i].clear();
		}
	}
	else if (msg->data.compare("stop") == 0)
	{
		savePoints = false;
		printf("stop command received\n");
	}
	else if (msg->data.compare("start") == 0)
	{
		savePoints = true;
		printf("start command received\n");
	}
}



int main(int argc, char** argv){

	ros::init(argc, argv, "fingercloudpub");
	ros::NodeHandle node;
	
	tf::TransformListener listener(ros::Duration(10.0));
	bool array[32];
//	for (int i = 0; i < 32; ++i) {
//		array[i] = (sensor_flag >> i) & 1;
//		if (array[i] ==1)
//				if (i==0)
					ros::Subscriber sub0 = node.subscribe<sensor_msgs::Range>("/finger0/s0", 1000, boost::bind(fingertip_callback, _1, &listener));
//		        else if (i==1)
					ros::Subscriber sub1 = node.subscribe<sensor_msgs::Range>("/finger0/s1", 1000, boost::bind(fingertip_callback, _1, &listener));
//				else if (i==2)
					ros::Subscriber sub2 = node.subscribe<sensor_msgs::Range>("/finger0/s2", 1000, boost::bind(fingertip_callback, _1, &listener));
//				else if (i==3)
					ros::Subscriber sub3 = node.subscribe<sensor_msgs::Range>("/finger0/s3", 1000, boost::bind(fingertip_callback, _1, &listener));
//				else if (i==4)
					ros::Subscriber sub4 = node.subscribe<sensor_msgs::Range>("/finger0/s4", 1000, boost::bind(fingertip_callback, _1, &listener));
//				else if (i==5)
					ros::Subscriber sub5 = node.subscribe<sensor_msgs::Range>("/finger0/s5", 1000, boost::bind(fingertip_callback, _1, &listener));

//				else if (i==16)
					ros::Subscriber sub10 = node.subscribe<sensor_msgs::Range>("/finger1/s0", 1000, boost::bind(fingertip_callback, _1, &listener));
//				else if (i==17)
					ros::Subscriber sub11 = node.subscribe<sensor_msgs::Range>("/finger1/s1", 1000, boost::bind(fingertip_callback, _1, &listener));
//				else if (i==18)
					ros::Subscriber sub12 = node.subscribe<sensor_msgs::Range>("/finger1/s2", 1000, boost::bind(fingertip_callback, _1, &listener));
//				else if (i==19)
					ros::Subscriber sub13 = node.subscribe<sensor_msgs::Range>("/finger1/s3", 1000, boost::bind(fingertip_callback, _1, &listener));
//				else if (i==20)
					ros::Subscriber sub14 = node.subscribe<sensor_msgs::Range>("/finger1/s4", 1000, boost::bind(fingertip_callback, _1, &listener));
//				else if (i==21)
					ros::Subscriber sub15 = node.subscribe<sensor_msgs::Range>("/finger1/s5", 1000, boost::bind(fingertip_callback, _1, &listener));

//				}


	ros::Subscriber sub_command = node.subscribe<std_msgs::String>("fingertip/command", 5, commandCallback);
	
	ros::Publisher pub_pointcloud[MAX_FINGER_LASER];
	
	for (int i = 0; i < MAX_FINGER_LASER; i++)
	{
		std::stringstream s;
		s << "fingertip_pointcloud_" << i;
		pub_pointcloud[i] = node.advertise< pcl::PointCloud<pcl::PointXYZ> >( s.str(), 5 );
	}

	ros::Rate loop_rate(10);
	while (node.ok()){

		for (int i = 0; i < MAX_FINGER_LASER; i++)
		{
			publish_point_cloud(pub_pointcloud[i], pointVector[i], ros::Time::now());
		}
		
		ros::spinOnce();
		
		loop_rate.sleep();
	}
	
	return 0;
};

