#include "geometry_msgs/Twist.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#define STRAIGHT_CORRECTION 1 //1.15
#define TO_METERS_PER_SEC_FWD 3942.39
#define TO_RADS_PER_SEC_ROT 750

class Pilot {
public:
	Pilot();
	~Pilot();

	void velCallback(const geometry_msgs::Twist::ConstPtr& vel);
  void publishOdom();
private:
	char chksum(unsigned char* data, int length);
	void sendVels(int id1, int vel1, int dir1, int id2, int vel2, int dir2);


	SerialPort serialPort;
  geometry_msgs::Twist::ConstPtr vel;
  ros::Time lastcheck;
  tf::TransformBroadcaster tfbr;
  tf::Transform odomT;
  ros::Subscriber sub;
};
