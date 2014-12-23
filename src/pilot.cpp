#include "ros/ros.h"
#include "serialport.h"
#include "pilot.h"

Pilot::Pilot(){
  if (serialPort.connect("/dev/ttyUSB0")!=0) {
    printf ("Serial port opened\n");

  } else {
    printf ("Can't open serial port");
  }

  ros::NodeHandle n;

  //http://answers.ros.org/question/108551/using-subscribercallback-function-inside-of-a-class-c/
  ros::Subscriber sub = n.subscribe("/cmd_vel", 1, &Pilot::velCallback, this);

  ros::spin();
}

Pilot::~Pilot(){
  sendVels(1,0,0, 2,0,0);
  serialPort.disconnect();
}
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void Pilot::velCallback(const geometry_msgs::Twist::ConstPtr& velmsg)
{
  ROS_INFO("I heard: x=[%f]", velmsg->linear.x);

  // 1m, 512 -> 7.7s => 512 -> 1/7.7 m/s => tics = 512 / .12987 
  int forwardvel = round(velmsg->linear.x * TO_METERS_PER_SEC_FWD);
  
  ROS_INFO("Forward velocity: x=[%d]", forwardvel);

  int rotvel = velmsg->angular.z * TO_RADS_PER_SEC_ROT;

  int vel1 = forwardvel + rotvel;
  int vel2 = (forwardvel - rotvel) * STRAIGHT_CORRECTION;

  if (vel1 > 4095)
    vel1 = 4095;
  if (vel1 < -4095)
    vel1 = -4095;
  if (vel2 > 4095)
    vel2 = 4095;
  if (vel2 < -4095)
    vel2 = -4095;

  bool dir1 = vel1 > 0;
  bool dir2 = vel2 < 0;
  
  sendVels(1,abs(vel1),dir1, 2,(int) abs(vel2),dir2);
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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pilot");

  Pilot pilot; 

  return 0;
}