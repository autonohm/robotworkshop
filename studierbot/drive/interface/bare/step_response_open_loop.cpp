/**
 * @author Stefan May
 * @date 10.10.2015
 * @brief Step response recorder for open-loop interface of motor shield
 */

#include <iostream>
#include <math.h>
#include <sys/time.h>
#include <cstdlib>
#include <fstream>
#include <ostream>
#include <string>
#include <sstream>
#include <vector>
#include <iomanip>

#include "SerialPort.h"
#include "protocol.h"

using namespace std;

const char _comPort[] = "/dev/frdm_dc_shield";
const speed_t _baud = B115200;

/**
 * Two types of gear motors are supported, e.g.
 * Pololu 131:1 Metal Gearmotor 37Dx57L mm with 64 CPR Encoder,        Pololu item#: 1447
 * Pololu  99:1 Metal Gearmotor 25Dx54L mm MP 12V with 48 CPR Encoder, Pololu item#: 3243
 */
#define POLOLU_GEARMOTOR_37D 1
#define FAULHABER_16002 1

#if POLOLU_GEARMOTOR_37D
#define GEARRATIO 131.f
#define ENCODERTICKSPERREV 64.f
#elif FAULHABER_16002
#define GEARRATIO 64.f
#define ENCODERTICKSPERREV 48.f
#else
#define GEARRATIO 99.f
#define ENCODERTICKSPERREV 48.f
#endif

int main(int argc, char* argv[])
{
 
  if(argc<2)
  {
    cout << "usage: " << argv[0] << " <pwm ratio> [pwm ratio 2] [pwm ratio 3] [pwm ratio 4] [pwm ratio 5] [pwm ratio 6]" << endl;
    return 0;
  }

  SerialPort* com = new SerialPort(_comPort, _baud);
  char bufCmd[14];
  char bufResponse[13];

  bool  retval = false;
  float responseF;
  float gearRatio = GEARRATIO;
  while(!retval)
  {
    bufCmd[0] = 0x16;
    bufCmd[13] = 'F';
    convertTo12ByteArray(gearRatio, &bufCmd[1]);
    int  sent   = com->send(bufCmd, 14);
    retval = com->receive(bufResponse, 13);
    convertFromByteArray(bufResponse, responseF);
    retval = (gearRatio==responseF);
  }
  cout << "Gear ratio: " << gearRatio << endl;

  float ticksPerRev = ENCODERTICKSPERREV;
  bufCmd[0] = 0x17;
  convertTo12ByteArray(ticksPerRev, &bufCmd[1]);
  int  sent   = com->send(bufCmd, 14);
  retval = com->receive(bufResponse, 13);
  convertFromByteArray(bufResponse, responseF);
  retval = (ticksPerRev==responseF);
  cout << "Ticks per Revolution: " << ticksPerRev << endl;

  vector<float> vTimestamp;
  vector<float> vU;  // set value
  vector<float> vV;  // response Motor1
  vector<float> vV2; // response Motor2
  vector<float> vV3; // response Motor3
  vector<float> vV4; // response Motor4
  vector<float> vV5; // response Motor4
  vector<float> vV6; // response Motor4

  bufCmd[0] = 0x00;
  bufCmd[13] = 'S';

  short inc = 1;
  short val = 0;

  timeval clk;
  ::gettimeofday(&clk, 0);
  double t_start = static_cast<double>(clk.tv_sec) + static_cast<double>(clk.tv_usec) * 1.0e-6;

  short samples = 15000;
  short u[6]    = {0, 0, 0, 0, 0, 0};
  u[0] = atoi(argv[1]);
  if(u[0]>100)  u[0] =  100;
  if(u[0]<-100) u[0] = -100;

  if(argc>2)
    u[1] = atoi(argv[2]);
  if(u[1]>100)  u[1] =  100;
  if(u[1]<-100) u[1] = -100;

  if(argc>3)
    u[2] = atoi(argv[3]);
  if(u[2]>100)  u[2] =  100;
  if(u[2]<-100) u[2] = -100;

  if(argc>4)
    u[3] = atoi(argv[4]);
  if(u[3]>100)  u[3] =  100;
  if(u[3]<-100) u[3] = -100;

  if(argc>5)
    u[4] = atoi(argv[5]);
  if(u[4]>100)  u[4] =  100;
  if(u[4]<-100) u[4] = -100;

  if(argc>6)
    u[5] = atoi(argv[6]);
  if(u[5]>100)  u[5] =  100;
  if(u[5]<-100) u[5] = -100;

  for(short i=0; i<samples; i++)
  {
    if(i>0.9*samples)
    {
      u[0] = 0;
      u[1] = 0;
      u[2] = 0;
      u[3] = 0;
      u[4] = 0;
      u[5] = 0;
    }

    convertTo12ByteArray(u, &bufCmd[1]);

    int sent = com->send(bufCmd, 14);

    bool retval = com->receive(bufResponse, 13);

    if(retval & (bufResponse[12]=='S'))
    {
      short rpm1 = ((bufResponse[0]  << 8) & 0xFF00) | (bufResponse[1]  & 0x00FF);
      short rpm2 = ((bufResponse[2]  << 8) & 0xFF00) | (bufResponse[3]  & 0x00FF);
      short rpm3 = ((bufResponse[4]  << 8) & 0xFF00) | (bufResponse[5]  & 0x00FF);
      short rpm4 = ((bufResponse[6]  << 8) & 0xFF00) | (bufResponse[7]  & 0x00FF);
      short rpm5 = ((bufResponse[8]  << 8) & 0xFF00) | (bufResponse[9]  & 0x00FF);
      short rpm6 = ((bufResponse[10] << 8) & 0xFF00) | (bufResponse[11] & 0x00FF);

      ::gettimeofday(&clk, 0);
      double t_now = static_cast<double>(clk.tv_sec) + static_cast<double>(clk.tv_usec) * 1.0e-6;

      vTimestamp.push_back(t_now-t_start);
      vU.push_back(u[0]);
      vV.push_back(((float)rpm1)/VALUESCALE);
      vV2.push_back(((float)rpm2)/VALUESCALE);
      vV3.push_back(((float)rpm3)/VALUESCALE);
      vV4.push_back(((float)rpm4)/VALUESCALE);
      vV5.push_back(((float)rpm5)/VALUESCALE);
      vV6.push_back(((float)rpm6)/VALUESCALE);
    }
    else
      cout << "failed to receive" << endl;
  }


  // Generate trace files
  string filenameInput = "Eingang.sim";
  string filenameOutput = "Ausgang.sim";
  ofstream outInput;
  ofstream outOutput;

  ostringstream oss;
  oss << filenameInput;

  outInput.open(oss.str().c_str(), std::ios::out);

  ostringstream oss2;
  oss2 << filenameOutput;
  outOutput.open(oss2.str().c_str(), std::ios::out);

  double t = 0.f;

  // Replace time stamp with accumulated mean value in order to have equal time distance of measurements
  double deltaT = ((vTimestamp[vTimestamp.size()-1] - vTimestamp[0])) / ((double)vTimestamp.size());

  // Round seconds to 4 digits
  int ndeltaT = (int)(deltaT * 10000.0 + 0.5);
  deltaT = ((double)ndeltaT) / 10000.0;

  outInput << "0 0" << endl;
  outOutput << "0 0" << endl;
  for(int i=0; i<vTimestamp.size(); i++)
  {
    t += deltaT;
    outInput << t << " " << vU[i] << endl;
    outOutput << t << " " << vV[i];
    if(argc>2) outOutput << " " << vV2[i];
    if(argc>3) outOutput << " " << vV3[i];
    if(argc>4) outOutput << " " << vV4[i];
    if(argc>5) outOutput << " " << vV5[i];
    if(argc>6) outOutput << " " << vV6[i];
    outOutput << endl;
  }
  outInput.close();
  outOutput.close();

  cout << "files written to: " << filenameInput << " and " << filenameOutput << endl;

  delete com;
}
