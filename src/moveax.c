#include "serialport.h"
#include <stdio.h>
#include <iostream> 
#include <unistd.h>

using namespace std;

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

void sendVels(int id1, int vel1, int dir1, int id2, int vel2, int dir2, SerialPort * serialPort){
	int vel1low = vel1 & 0xFF;
	int vel1hi = (vel1 & 0xFF00) >> 8;
	if (dir1)
		vel1hi = vel1hi | 4;
	int vel2low = vel2 & 0xFF;
	int vel2hi = (vel2 & 0xFF00) >> 8;
	if (dir2)
		vel2hi = vel2hi | 4;
	unsigned char pkt[] = {0xFF, 0xFF, 0xFE, 0x0A, 0x83, 0x20, 0x02, id1,vel1low, vel1hi, id2, vel2low, vel2hi, 0x00};
	pkt[13] = chksum(pkt, 13);
	int n = serialPort->sendArray(pkt, 14);
}

int main() {
	cout << "AX Control starts" << endl; // prints AX Control 
	int error=0; 
	int idAX12=0; 
	SerialPort serialPort; 
	if (serialPort.connect("/dev/ttyUSB0")!=0) {
		printf ("Serial port opened\n");
		// Sync Write: 0xFF 0xFF 0xFE(broadcast) [(L+1) * N + 4] 0x83(instruction)  0x20(location) 0x02(lenght of the data) 0x01(id1) lowvel1 hivel1 0x02(id2) lowvel2 hivel2 chk
		// This one sends vels to both motors with orientation
		sendVels(1,512,1, 2,512,0, &serialPort);
		serialPort.disconnect();
	} else {
		printf ("nCan't open serial port");
		error=-1;
	}
	 
	cout << endl << "AX Control ends" << endl; // prints AX Control
	return error;
}

