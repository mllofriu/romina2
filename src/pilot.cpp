#include "ros/ros.h"
#include "serialport.h"
#include "pilot.h"


#define MOTOR1 16
#define MOTOR2 18

Pilot::Pilot(){
  if (serialPort.connect("/dev/ttyUSB0")!=0) {
    printf ("Serial port opened\n");

  } else {
    printf ("Can't open serial port");
  }

  ros::NodeHandle n;

  vel = geometry_msgs::Twist::ConstPtr(new geometry_msgs::Twist());

  // Send an initial transform with odometry 0
  odomT = tf::Transform (tf::Quaternion(tf::Vector3(0,0,1), 0));
  lastcheck = ros::Time::now();
  tfbr.sendTransform(tf::StampedTransform(odomT, lastcheck, "map", "odom"));

  //http://answers.ros.org/question/108551/using-subscribercallback-function-inside-of-a-class-c/
  sub = n.subscribe("/cmd_vel", 10, &Pilot::velCallback, this);

  
}

Pilot::~Pilot(){
  sendVels(MOTOR1,0,0, MOTOR2,0,0);
  serialPort.disconnect();
}
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void Pilot::velCallback(const geometry_msgs::Twist::ConstPtr& velmsg)
{
  ROS_INFO("Received vel");

  vel = velmsg;

  // 1m, 512 -> 7.7s => 512 -> 1/7.7 m/s => tics = 512 / .12987 
  int forwardvel = -round(velmsg->linear.x * TO_METERS_PER_SEC_FWD);

  int rotvel = velmsg->angular.z * TO_RADS_PER_SEC_ROT;

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
  ros::Time tNow = ros::Time::now();

  double interval = (tNow - lastcheck).toSec();
  // Get the relative change
  // TODO: mutuoexclusion of variable vel
  tf::Transform change;
  tf::Vector3 o;
  o.setX(vel->linear.x * interval);
  change.setOrigin(o);
  // Added 2 factor to the angle - maybe bug when building the quaternion
  tf::Quaternion rot(tf::Vector3(0,0,1), 2 * vel->angular.z * interval);
  change.setRotation(rot);
  // Modify the transform
  odomT.mult(odomT, change);
  
  //Republish the transform
  lastcheck = tNow;
  tfbr.sendTransform(tf::StampedTransform(odomT, tNow, "map", "odom"));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pilot");

  Pilot pilot; 
  
  ros::Rate r(5);
  while (ros::ok())
  {
    ros::spinOnce();

    pilot.publishOdom();

    r.sleep();
  }  

  return 0;
}
