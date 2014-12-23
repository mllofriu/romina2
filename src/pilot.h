#include "geometry_msgs/Twist.h"

#define STRAIGHT_CORRECTION 1.15
#define TO_METERS_PER_SEC_FWD 3942.39
#define TO_RADS_PER_SEC_ROT 512

class Pilot {
public:
	Pilot();
	~Pilot();

	void velCallback(const geometry_msgs::Twist::ConstPtr& vel);
private:
	char chksum(unsigned char* data, int length);
	void sendVels(int id1, int vel1, int dir1, int id2, int vel2, int dir2);

	SerialPort serialPort;
};