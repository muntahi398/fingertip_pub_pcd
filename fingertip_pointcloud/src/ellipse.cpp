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

/*	This is just a dummy to simulate that "/right_hand_fore_finger_distal_link"
	and "/right_hand_thumb_distal_link" are moving in a vertical plane forming
	an ellipse.
*/

#include <tf/transform_broadcaster.h>

#include <math.h>
#include "ros/ros.h"
#include<string.h>    //strlen
#include <ros/ros.h>



#define CIRCLE_STEPS 1000
#define PI 3.14159265

int main(int argc, char** argv){

	float circleStep = 2 * PI / CIRCLE_STEPS;
	int stepCounter = 0;
	
	ros::init(argc, argv, "ellipse");
	
	tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setRotation( tf::createQuaternionFromRPY(0.0, 0.0, 0.0) );
    
    ros::Rate loop_rate(100);
    
    ros::Time now;

    while (ros::ok())
    {
	    now = ros::Time::now();
	
		transform.setOrigin( tf::Vector3(0.0, cos(stepCounter * circleStep) * 0.20, sin(stepCounter * circleStep) * 0.30 + 0.50) );
		br.sendTransform(tf::StampedTransform(transform, now, "/base_link", "/right_hand_fore_finger_distal_link"));
		
		transform.setOrigin( tf::Vector3(0.0, cos(stepCounter * circleStep) * 0.20, sin(stepCounter * circleStep) * 0.30 + 0.55) );
		br.sendTransform(tf::StampedTransform(transform, now, "/base_link", "/right_hand_thumb_distal_link"));
		
		stepCounter = (stepCounter+1) % CIRCLE_STEPS;
		
    	ros::spinOnce();
	    loop_rate.sleep();
    }
}
