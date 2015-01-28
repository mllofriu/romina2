#include "geometry_msgs/Twist.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "ros/ros.h"
#include "romina2/Move.h"
#include <semaphore.h>
#include <boost/thread.hpp>

#define STRAIGHT_CORRECTION 1 //1.15
#define TO_METERS_PER_SEC_FWD 3942.39
#define TO_RADS_PER_SEC_ROT 900

class Pilot {
public:
	Pilot(ros::NodeHandle& n);
	~Pilot();

	void velCallback(const geometry_msgs::Twist::ConstPtr& vel);
	void publishOdom();
	bool move(romina2::Move::Request &req, romina2::Move::Response &res);
private:
	char chksum(unsigned char* data, int length);
	void sendVels(int id1, int vel1, int dir1, int id2, int vel2, int dir2);

	SerialPort serialPort;
	geometry_msgs::Twist vel;
	// Odom publisher variables
	ros::Time lastcheck;
	tf::TransformBroadcaster tfbr;
	tf::Transform odomT;
	// Service variables
	tf::Transform movedOdomT;
	tf::Transform toMoveOdomT;
	bool movingByService;
	double pTranslation;
	double pRotation;

	ros::Subscriber sub;
	sem_t mtx;
	boost::thread * publisherThread;
	int toRadPerSec;
};
