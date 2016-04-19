#include "../../../hexapede/hexapete_ctrl_motor/src/Motorcontroller.h"

#include <unistd.h>

#include <termios.h>
#include <errno.h>
#include <stdio.h>
#include <fcntl.h>

#include <iostream>
#include <string.h>
#include <stdlib.h>

#include <string>
#include <ros/ros.h>
#include "../../../hexapede/hexapete_ctrl_motor/src/params.h"

using namespace std;

Motorcontroller::Motorcontroller()
{
	_rpmMax = RPMMAX;
	_cmdMax = CMDMAX;

	_maxCmd = CMDMAX;
	_minCmd = -CMDMAX;

	std::string comPort;
	ros::NodeHandle prvNh("~");
	prvNh.param<std::string>("com_port", comPort, "/dev/ttyACM0");

	//const char comPort[] = comPortAdress.c_str();//"/dev/ttyACM3";
	const speed_t baud = B115200;
  _com = new SerialPort(comPort.c_str(), baud);
}

Motorcontroller::~Motorcontroller()
{
  stop();
  delete _com;
}

int Motorcontroller::getRPMMax()
{
  return (int)_rpmMax;
}

void Motorcontroller::setRPM(double rpm[6])
{
  

  // voltage control vector [0;255], i.e., [-u_max;u_max]
  unsigned char voltage[14];
  memset(voltage, 0, 14*sizeof(char));
  
  //Projektgruppe!
  cout << "rpm(soll):" << endl;
  for(int i = 0; i < 6; i++){  
    cout << rpm[i] << "  " ;
  }
  cout << endl;

  // normalized control vector in range [_minCmd; _maxCmd]
  double x[6];
  for(int i=0; i<6; i++)
  {
    x[i] = rpm[i]  / _rpmMax * _maxCmd;
    if(x[i]>_maxCmd) x[i] = _maxCmd;
    if(x[i]<_minCmd) x[i] = _minCmd;
  }

  int checksum = 0;

  //Projektgruppe!
  cout << "x(soll):" << endl;
  for(int i = 0; i < 6; i++){  
    cout << x[i] << "  " ;
  }
  cout << endl;


  //Projektgruppe!
  cout << "x(left): " << x[0] << ", x(right): " << x[3] << endl;

  for(int i=0; i<6; i++)
  {
    short sx = x[i];
     //Projektgruppe!
    cout << "sx(" << i << "): " << sx << "  ";

    voltage[2*i] = (unsigned char)(0x00FF & sx);
    voltage[2*i+1] = (unsigned char)((0xFF00 & sx) >>8);

    // calculate 16 Bit checksum
    // -> lower 16 Bit of sum over all channels
    short c = (voltage[2*i+1] << 8) | voltage[2*i];
    checksum += abs(c);
  }
  
  //Projekt
  cout << endl;
  
  cout << "checksum: " << (0xFFFF & checksum) << endl;
  voltage[12] = (unsigned char)(0x00FF & checksum);
  voltage[13] = (unsigned char)((0xFF00 & checksum) >> 8);
  
  //Projektgruppe!!
  for(int i=0; i<14; i++){
    cout << "voltage[" << i << "]: " << (unsigned int)voltage[i] << ", ";
  }
  cout << endl;
  
  _com->send((char*)voltage, 14);

  short events[6];
  bool retval = _com->receive(events, 6);

  if(retval)
  {
    cout << "rpm: " << endl;
    for(int i=0; i<6; i++)
    {
      _rpm[i] = (double)events[i];
      printf("%05.0lf ", _rpm[i]);
    }
    cout << endl;
  }else{
    cout << "No Reply from the TOWER!" << endl << endl;;
  }
  cout << "------------------------------------------------------------" << endl;
}

void Motorcontroller::getRPM(double rpm[6])
{
  for(int i=0; i<6; i++)
    rpm[i] = _rpm[i];
}

void Motorcontroller::stop()
{
  double rpm[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  setRPM(rpm);
}
