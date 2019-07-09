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

#include "finger_pub.h"

//#include <stdio.h>


int main(int argc, char **argv){

    ROS_INFO("Starting fingertip publisher.");
    ros::init(argc, argv, "fingerpub");
    ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("fingertip/change_active_sensor", changeSensor);

	ros::Subscriber sub_command = n.subscribe<std_msgs::String>("fingertip/sensor_command", 5, commandCallback);

    device = new FtdiUsbDevice(INTERFACE_B, 0x0403, 0x6010);

    if (!device->DeviceFound())
    {
        ROS_FATAL("Did not find the FTDI interface. Exiting.");
        exit(EXIT_FAILURE);
    } else
    {
        ROS_INFO("Found the FTDI interface.");
    }
    device->SetBaudrate(3000000);
    device->SetLatencyTimer(1);

    adnsReader = new AdnsReader(device);

    frameReader = new FrameReader(adnsReader);

    ros::Publisher pub = n.advertise<fingertip_msg::adns>("fingertip/adns", 1000);
	
	image_transport::ImageTransport it(n);
	
	imgpub[0] = it.advertise("fingertip/image0", 1);
	imgpub[1] = it.advertise("fingertip/image1", 1);
	imgpub[2] = it.advertise("fingertip/image2", 1);
	imgpub[3] = it.advertise("fingertip/image3", 1);

    ros::Rate loop_rate(-1);  //For now publish as fast as the sensor can deliver data
    
    original_sens_active_mask = getSensActiveMask();

	switch_sensor(0);
	UpdateRegisters();
	UpdateRegisters();

	switch_sensor(1);
	UpdateRegisters();
	UpdateRegisters();

	switch_sensor(2);
	UpdateRegisters();
	UpdateRegisters();

	switch_sensor(3);
	UpdateRegisters();
	UpdateRegisters();

	changeSensActiveMask(NORM_MODE);
	
    ROS_INFO("Entering streaming loop.");
    while (ros::ok())
    {
    	//normal mode
    	if (modenum == NORM_MODE)
    	{
    		for (int i = 0; i < 4; i++)
    		{
				if (original_sens_active_mask & (0x01<<i))
				{
					switch_sensor(i);
					UpdateMotionBurst(pub, modenum);	
				}
    		}
    	}
    	//frame mode, modenum indicates the sensor
    	else if (modenum >= 0 && modenum < 4)
    	{	
    		if (modenum != lastModenum)
    		{	
    			changeSensActiveMask(NORM_MODE);	//to make sens_active_mask = 0
    			changeSensActiveMask(modenum);		//to switch on sensor for image reading
    			lastModenum = modenum;
    		}
   			UpdateMotionBurst(pub, modenum);
    	}

		ros::spinOnce();

		loop_rate.sleep();
    }
}


//to get the original sens_active_mask at the beginning
//to know which sensors are really connected
uint8_t getSensActiveMask()
{
	unsigned char* buffer = adnsReader->Read("O",1)->GetData();
    return buffer[0];
}


void changeSensActiveMask(int modenum)
{
	if (modenum == 0) {
		adnsReader->Read("A", 0);
	} else if (modenum == 1) {
		adnsReader->Read("B", 0);
	} else if (modenum == 2) {
		adnsReader->Read("C", 0);
	} else if (modenum == 3) {
		adnsReader->Read("D", 0);
	} else if (modenum == NORM_MODE) {
		adnsReader->Read("n", 0);
	}
}	


void switch_sensor(uint8_t sensor_id)
{
    
    char ascii[2];
    if (sensor_id == 0) {
	ascii[0] = '0';
    } else if (sensor_id == 1) {
	ascii[0] = '1';
    } else if (sensor_id == 2) {
	ascii[0] = '2';
    } else if (sensor_id == 3) {
	ascii[0] = '3';
    } else {
	ascii[0] = '0';
	printf("Unsupported sensor_id in switch_sensor %d\n", sensor_id);
    
    }
    ascii[1]= 0;
	
	
    Buffer* data = adnsReader->Read(ascii, 1);
    if (data != NULL)
    {
        //printf("switch_sensor: %c\n", data->GetData()[0]);
	delete data;
    }
}


void UpdateRegisters()
{
    Buffer* data = adnsReader->Read("r", 37);
    if (data != NULL)
    {
        delete data;
    }
}


void UpdateMotionBurst(ros::Publisher &pub, int modenum)
{
    ros::Time now = ros::Time::now();
    Buffer* data;
	bool includeFrame = false;
			
	if (modenum >= 0 && modenum < 4)
	{
		//f	switch to motionburst, does not load firmware
		data = adnsReader->Read("f", 15);
		includeFrame = true;
	}
	else
	{
		//m	switch to motionburst, loads firmware
		data = adnsReader->Read("m", 15);
	}

    if (data != NULL)
    {
        AdnsMotionBurstReport report;
        data->CopyTo(&report);
        delete data;
        report.Initialize();

		fingertip_msg::adns msg;
		msg.header.stamp = now;
		msg.sensor_id = report.SensorID;
		msg.shutter = report.Shutter > 0x4e20 ? 0x4e20 : report.Shutter;
		msg.frame_period = report.FramePeriod > 0x61a8 ? 0x61a8 : report.FramePeriod;
		msg.frame_period = msg.frame_period < 4000 ? 4000 : msg.frame_period;
		msg.squal = report.SurfaceQuality;
		msg.observation = report.Observation;
		msg.motion = report.Motion;
		msg.deltax = report.DeltaX;
		msg.deltay = report.DeltaY;
		msg.pixel_sum = report.PixelSum;
		msg.max_pixel = report.PixelMaximum;
		msg.min_pixel = report.PixelMinimum;
		
		if (includeFrame)
		{
			frameReader->UpdateAll();
			if (frameReader->IsFrameAvailable())
			{
				unsigned char* framedata = frameReader->GetFrameBuffer()->GetData();
				for (int i = 0; i < 900; i++)
				{
					msg.frame.push_back(framedata[i+1]);
				}

	   			UpdateFrame(framedata, modenum);
			}
		}

		if (msg.sensor_id == 1)
		{
		    msg.header.frame_id = "/right_hand_fore_finger_distal_link";
		} else if (msg.sensor_id == 3)
		{
		    msg.header.frame_id = "/right_hand_ring_finger_distal_link";
		} else if (msg.sensor_id == 0)
		{
		    msg.header.frame_id = "/right_hand_thumb_distal_link";
		}

		if ((report.FramePeriod < 5000) or (report.Shutter < 3000)) //old values 5000 1500
		{
			if (report.Shutter < 100) //old value 100
		    {
		        msg.dist_mm = 1;
		    } else if (report.Shutter < 1000) //old value 1000
		    {
		        msg.dist_mm = 5;
		    } else
		    {
		        msg.dist_mm = 10;
		    }
		}
		pub.publish(msg);

    }
}


void UpdateFrame(unsigned char* framedata, int modenum)
{
	if (imgpub[0].getNumSubscribers() || imgpub[1].getNumSubscribers() || imgpub[2].getNumSubscribers() || imgpub[3].getNumSubscribers())
	{
		sensor_msgs::Image img;
		img.header.stamp = ros::Time::now();
		img.width = 30; 
		img.height = 30;
		img.encoding = "mono8";
		img.step = sizeof(framedata[0]) * 30;
	
		for (int i = 0; i < 900; i++)
		{
			img.data.push_back(framedata[i+1] << 1);
		}
	
		if (modenum == 0)
		{
			img.header.frame_id = "/right_hand_thumb_distal_link";
		} 
		else if (modenum == 1)
		{
			img.header.frame_id = "/right_hand_fore_finger_distal_link";
		} 
		else if (modenum == 3)
		{
			img.header.frame_id = "/right_hand_ring_finger_distal_link";
		}
		imgpub[modenum].publish(img);
	}
}


bool changeSensor(fingertip_laser::ChangeActiveSensor::Request  &req, fingertip_laser::ChangeActiveSensor::Response &res )
{
	if (req.sensor == NORM_MODE)
	{
		modenum = NORM_MODE;
		printf("Current mode is normal\n");
		res.success = 1;
	}
	else if (original_sens_active_mask & (0x01<<req.sensor))
	{
		switch_sensor(req.sensor);
		modenum = req.sensor;
		printf("Current mode is frame%d\n", modenum);
		res.success = 1;
	}
	else
	{
		printf("Sensor %d is not connected\n", req.sensor);
		res.success = 0;
	}
  	return true;
}


//rostopic pub -1 /fingertip/sensor_command std_msgs/String normal
//rostopic pub -1 /fingertip/sensor_command std_msgs/String frame0, 1 or 3
void commandCallback(const std_msgs::String::ConstPtr& msg)
{
	//boost::mutex::scoped_lock l(m_mutex);
	if (msg->data.compare("normal") == 0)
	{
		modenum = NORM_MODE;
		printf("Current mode is normal\n");
	}
	else if (msg->data.compare("frame0") == 0 || msg->data.compare("frame1") == 0 || msg->data.compare("frame2") == 0 || msg->data.compare("frame3") == 0)
	{
		int tmpmode;
		string tmpstr;
		tmpstr = msg->data;
		sscanf(tmpstr.substr(5).c_str(), "%d", &tmpmode);
		if (original_sens_active_mask & (0x01<<tmpmode))
		{
			modenum = tmpmode;
    		switch_sensor(modenum);
			printf("Current mode is frame%d\n", modenum);
		}
		else
		{
			printf("Sensor %d is not connected\n", tmpmode);
		}
	}
}
