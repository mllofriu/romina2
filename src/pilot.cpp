#include "serialport.h"
#include "pilot.h"

#include <boost/thread.hpp>
#include <tf/tf.h>


#define MOTOR1 16
#define MOTOR2 18

Pilot::Pilot(ros::NodeHandle & n){
  if (serialPort.connect("/dev/ttyUSB0")!=0) {
    ROS_INFO ("Serial port opened\n");

  } else {
    ROS_INFO ("Can't open serial port");
  }

  sem_init(&mtx, 0, 1);
  vel = geometry_msgs::Twist();

  n.param("toRadPerSec", toRadPerSec, TO_RADS_PER_SEC_ROT);

  // Send an initial transform with odometry 0
  odomT = tf::Transform (tf::Quaternion(tf::Vector3(0,0,1), 0));
  lastcheck = ros::Time::now();
  tfbr.sendTransform(tf::StampedTransform(odomT, lastcheck, "map", "odom"));

  //http://answers.ros.org/question/108551/using-subscribercallback-function-inside-of-a-class-c/
  sub = n.subscribe("/cmd_vel", 10, &Pilot::velCallback, this);

  publisherThread = new boost::thread(boost::bind( &Pilot::publishOdom, this ));
  
  movingByService = false;
  n.getParam("pTranslation", pTranslation);
  n.getParam("pRotation", pRotation);
  ros::ServiceServer service = n.advertiseService("move", &Pilot::move, this);
  ROS_INFO("Service ready.");
}

Pilot::~Pilot(){
  sendVels(MOTOR1,0,0, MOTOR2,0,0);
  serialPort.disconnect();
}

bool Pilot::move(romina2::Move::Request &req, romina2::Move::Response &res){
	sem_wait(&mtx);
	tf::transformMsgToTF(req.movement,toMoveOdomT);
	movedOdomT = tf::Transform();
	movingByService = true;
	sem_post(&mtx);

	return true;
}

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void Pilot::velCallback(const geometry_msgs::Twist::ConstPtr& velmsg)
{
  ROS_DEBUG("Received vel");

  sem_wait(&mtx);
  vel = *velmsg;
  // Cancels moving by service
  movingByService = false;
  sem_post(&mtx);  

  // 1m, 512 -> 7.7s => 512 -> 1/7.7 m/s => tics = 512 / .12987 
  int forwardvel = -round(vel.linear.x * TO_METERS_PER_SEC_FWD);

  int rotvel = vel.angular.z * toRadPerSec;

  int vel1 = forwardvel + rotvel;
  int vel2 = (forwardvel - rotvel) * STRAIGHT_CORRECTION;

  if (vel1 > 1023)
    vel1 = 1023;
  if (vel1 < -1023)
    vel1 = -1023;
  if (vel2 > 1023)
    vel2 = 1023;
  if (vel2 < -1023)
    vel2 = -1023;

  bool dir1 = vel1 > 0;
  bool dir2 = vel2 < 0;
  
  sendVels(MOTOR1,abs(vel1),dir1, MOTOR2,(int) abs(vel2),dir2);
}

char Pilot::chksum(unsigned char* data, int length)
{
    int cs = 0;
    for (int i = 2; i < length; i++)
    {
      cs += data[i];
    }
    cs = ~cs;
    return (char)(cs & 0x0FF);
}

void Pilot::sendVels(int id1, int vel1, int dir1, int id2, int vel2, int dir2){
  int vel1low = vel1 & 0xFF;
  int vel1hi = (vel1 & 0xFF00) >> 8;
  if (dir1)
    vel1hi = vel1hi | 4;
  int vel2low = vel2 & 0xFF;
  int vel2hi = (vel2 & 0xFF00) >> 8;
  if (dir2)
    vel2hi = vel2hi | 4;
  // Sync Write: 0xFF 0xFF 0xFE(broadcast) [(L+1) * N + 4] 0x83(instruction)  0x20(location) 0x02(lenght of the data) 0x01(id1) lowvel1 hivel1 0x02(id2) lowvel2 hivel2 chk
  unsigned char pkt[] = {0xFF, 0xFF, 0xFE, 0x0A, 0x83, 0x20, 0x02, id1,vel1low, vel1hi, id2, vel2low, vel2hi, 0x00};
  pkt[13] = chksum(pkt, 13);
  int n = serialPort.sendArray(pkt, 14);
}

void Pilot::publishOdom(){  

  ros::Rate r(10);
  while (ros::ok())
  {
    //ros::spinOnce();
    ros::Time tNow = ros::Time::now();

    double interval = (tNow - lastcheck).toSec();
    // Get the relative change
    // TODO: mutuoexclusion of variable vel
    tf::Transform change;
    tf::Vector3 o;

    sem_wait(&mtx);
    o.setX(vel.linear.x * interval);
    // Added 2 factor to the angle - maybe bug when building the quaternion
    tf::Quaternion rot(tf::Vector3(0,0,1), 2 * vel.angular.z * interval);
    sem_post(&mtx);  

    change.setOrigin(o);
    change.setRotation(rot);
    // Modify the odometry transform
    odomT.mult(odomT, change);

    // Modify the movement transform
    if (movingByService){
    	//movedOdomT.mult(movedOdomT, change);
    	// Integrate x movement and rotation separately
    	tf::Vector3 movedO = movedOdomT.getOrigin();
    	movedO.setX(movedO.getX() + change.getOrigin().getX());
    	tf::Quaternion movedRot = movedOdomT.getRotation();
    	movedRot = movedRot * change.getRotation();
    	movedOdomT.setOrigin(movedO);
    	movedOdomT.setRotation(movedRot);
    	// Find difference
    	tf::Transform remaining = movedOdomT.inverse();
    	remaining.mult(toMoveOdomT, remaining);
    	sem_wait(&mtx);
    	vel.linear.x = pTranslation * remaining.getOrigin().getX();
    	vel.angular.z = pRotation * remaining.getRotation().getAngle();
    	sem_post(&mtx);
    }
    //Republish the transform
    lastcheck = tNow;
    tfbr.sendTransform(tf::StampedTransform(odomT, tNow, "map", "odom"));

    r.sleep();
  }  
  
}

/*int main(int argc, char **argv)
{
  ros::init(argc, argv, "pilot");

  Pilot pilot (ros::NodeHandle()); 
  
  ros::Rate r(5);
  while (ros::ok())
  {
    ros::spinOnce();

    pilot.publishOdom();

    r.sleep();
  }  

  return 0;
}*/

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

class PilotNodelet : public nodelet::Nodelet {
public:
	PilotNodelet() {};
	virtual ~PilotNodelet(){};

	virtual void onInit()
	  {
	    ros::NodeHandle nh = this->getPrivateNodeHandle();

	    // resolve node(let) name
	    std::string name = nh.getUnresolvedNamespace();
	    //NODELET_INFO_STREAM("Namespace " << name);
	    int pos = name.find_last_of('/');
	    name = name.substr(pos + 1);

	    NODELET_INFO_STREAM("Initialising nodelet... [" << name << "]");
	    controller_.reset(new Pilot(nh));

	  }
	private:
	  boost::shared_ptr<Pilot> controller_;
};

PLUGINLIB_EXPORT_CLASS(PilotNodelet,
                       nodelet::Nodelet);


