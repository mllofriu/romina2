#include "ros/ros.h"
#include "serialport.h"

char chksum(unsigned char* data, int length)
{
    int cs = 0;
    for (int i = 2; i < length; i++)
    {
      cs += data[i];
    }
    cs = ~cs;
    return (char)(cs & 0x0FF);
}

void configMotor(SerialPort * serialPort, int id){
  // Sync Write: 0xFF 0xFF 0xFE(broadcast) [(L+1) * N + 4] 0x83(instruction)  0x20(location) 0x02(lenght of the data) 0x01(id1) lowvel1 hivel1 0x02(id2) lowvel2 hivel2 chk
  //unsigned char pkt[] = {0xFF, 0xFF, 0xFE, 0x04, 0x03, 0x04, 0x01, 0x00};
  //pkt[7] = chksum(pkt, 7);
  //int n = serialPort->sendArray(pkt, 8);

  // Set the new id
  //unsigned char idpkt[] = {0xFF, 0xFF, 0xFE, 0x04, 0x03, 0x03, id, 0x00};
  //idpkt[7] = chksum(idpkt, 7);
  //n = serialPort->sendArray(idpkt, 8);

  unsigned char ledpkt[] = {0xFF, 0xFF, 0xFE, 0x04, 0x03, 0x19, 0x01, 0x00};
  ledpkt[7] = chksum(ledpkt, 7);
  int n = serialPort->sendArray(ledpkt, 8);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motor_configurer");
  
  if (argc < 2){
    ROS_ERROR("Usage: configmotors newId");  
    exit(1);
  }

  SerialPort serialPort;
  if (serialPort.connect("/dev/ttyUSB0")!=0) {
    printf ("Serial port opened\n");

  } else {
    printf ("Can't open serial port");
  }

  configMotor(&serialPort, atoi(argv[1]));
  return 0;
}
