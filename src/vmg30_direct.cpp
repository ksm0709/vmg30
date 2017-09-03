#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"


#include "serial.h"

#define SAMPLE_HZ	80

typedef union
{
    struct
    {
    int8_t cmd, pkglen, pkgtype; 
    int16_t id, clk;
    int32_t quat_w[4], quat_h[4];
    int16_t finger[10], palmarch, thumbcr, noval;
    int16_t press[5];
    int16_t abduction[4];
    int16_t time;
    int8_t crc,endcar;
    }s;

    uint8_t bytes[87];
} GLOVE_PACKET;

int main(int argc, char **argv)
{
	ros::init(argc,argv,"vmg30_direct_node");
	ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::Float32MultiArray>("vmg30_fingers",1); 
	std_msgs::Float32MultiArray pub_msg;

	ros::Rate loop_rate(SAMPLE_HZ);

	GLOVE_DATA dataRead;

	char port_name[30] = "/dev/ttyUSB0";
	int count=0;

	if( argc > 1 )
		strcpy(port_name, argv[1]);

	char HEADER[3] = "$$";
	Serial comm( 2 , (char*)&HEADER ); 
	comm.setCheckSum(GLOVE_DATA_SIZE-1);

	if( comm.Open(port_name, 460800, 10, 1) )
		ROS_INFO("VMG30 Device Connected! (%s)",port_name);
	else
	{
		ROS_INFO("VMG30 Device Connection Failed! (%s)",port_name);
		return 0;
	}

	while( ros::ok() )
	{
		if( comm.readPacket(dataRead.bytes, GLOVE_DATA_SIZE ) )
		{
			pub_msg.data.clear();

			for( int i = 0; i < 5 ; i++ )
				pub_msg.data.push_back(dataRead.s.finger[i].pip);

			pub.publish( pub_msg );
			
		}
		
		if( ++count > SAMPLE_HZ-1 )			
		{
			ROS_INFO("Data receving freqency : %.0ld Hz", comm.getFreq());
			count = 0;
		}

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

