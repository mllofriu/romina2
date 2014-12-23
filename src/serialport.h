// Taken from http://softwaresouls.com/softwaresouls/2012/03/05/linux-c-dynamixel-reading-and-writing-example/
#ifndef SERIALPORT_H_
#define SERIALPORT_H_

 
class SerialPort {
private:
int fileDescriptor;
 
public:
int connect ();
int connect (char * device);
void disconnect(void);
 
int sendArray(unsigned char *buffer, int len);
int getArray (unsigned char *buffer, int len);
 
int bytesToRead();
void clear();
};
 
#endif /* SERIALPORT_H_ */