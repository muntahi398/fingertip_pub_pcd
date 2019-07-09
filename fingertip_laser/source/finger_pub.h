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

#ifndef finger_pub_h
#define finger_pub_h

//#include "AdnsRegisters.h"
#include "AdnsMotionBurstReport.h"
#include "FrameReader.h"
#include "ros/ros.h"
#include "fingertip_msg/adns.h"
#include "fingertip_laser/ChangeActiveSensor.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>


using namespace std;

const int NORM_MODE = 10;
FtdiUsbDevice* device;
AdnsReader* adnsReader;
FrameReader* frameReader;
int modenum = NORM_MODE; //normal mode
int lastModenum = NORM_MODE;
uint8_t original_sens_active_mask;

image_transport::Publisher imgpub[4];

void SetRegisters(bool);
void SetMotionBurst(bool);

void UpdateRegisters();
void UpdateMotionBurst(ros::Publisher &pub, int modenum);
void switch_sensor(uint8_t sensor_id);
void commandCallback(const std_msgs::String::ConstPtr& msg);
void changeSensActiveMask(int modenum);
void UpdateFrame(unsigned char* framedata, int modenum);
bool changeSensor(fingertip_laser::ChangeActiveSensor::Request  &req,
         		  fingertip_laser::ChangeActiveSensor::Response &res );
uint8_t getSensActiveMask();

#endif
