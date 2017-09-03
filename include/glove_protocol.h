#pragma once
#include <string>
using namespace std;

#define USE_LITE_PROTOCOL 1

#if USE_LITE_PROTOCOL == 0

#define PACKET_SIZE			152
#define GLOVE_DATA_SIZE		145 // byte

typedef struct
{
	double pip, mp, press;
} FINGER;

typedef union
{
	struct{ 
		FINGER finger[5];
		double roll;
		double pitch;
		double yaw;
		char csum;
	}s;

	char bytes[PACKET_SIZE];
	
} GLOVE_DATA;

#else

#define PACKET_SIZE			
#define GLOVE_DATA_SIZE		21 // byte

typedef union
{
	struct{
		float pip[5];
		char csum;
	}s;

	char bytes[21];
} GLOVE_DATA;


#endif