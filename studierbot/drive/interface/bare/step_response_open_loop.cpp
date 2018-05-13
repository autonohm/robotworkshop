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
#define POLOLU_GEARMOTOR_37D 0
#define POLOLU_GEARMOTOR_25D 0
#define FAULHABER_16002 0

#if POLOLU_GEARMOTOR_37D
#define GEARRATIO 131.f
#define ENCODERTICKSPERREV 64.f
#elif POLOLU_GEARMOTOR_25D
#define GEARRATIO 99.f
#define ENCODERTICKSPERREV 48.f
#elif FAULHABER_16002
#define GEARRATIO 64.f
#define ENCODERTICKSPERREV 48.f
#else
#define GEARRATIO 14.f
#define ENCODERTICKSPERREV 1024.f
#endif

SerialPort* _com;
char _bufCmd[14];
char _bufIn[13];

/**
 * Send float commands to motor shield
 * @param cmd command byte
 * @param param parameter
 * @param response response of motor controller
 * @param echo verbosity of function, true provides command line output
 */
template<typename T>
bool sendToMotorshield(char cmd, T param, T* response, bool echo)
{
  _bufCmd[0] = cmd;
  convertTo12ByteArray(param, &_bufCmd[1]);

  _com->send(_bufCmd, 14);
  bool retval = _com->receive(_bufIn, 13);
  convertFromByteArray(_bufIn, *response);

  if(echo)
  {
    cout << "Sent " << param << ", echo: " << *response << endl;
  }

  return retval;
}

bool sendToMotorshieldF(char cmd, float param, float* response, bool echo)
{
  _bufCmd[13] = 'F';
  return sendToMotorshield<float>(cmd, param, response, echo);
}

bool sendToMotorshieldI(char cmd, int param, int* response, bool echo)
{
  _bufCmd[13] = 'I';
  return sendToMotorshield<int>(cmd, param, response, echo);
}

bool sendToMotorshieldS(char cmd, short param[6], short (*response)[6], bool echo)
{
  _bufCmd[13] = 'S';
  bool retval = sendToMotorshield<short[6]>(cmd, param, response, false);
  if(echo)
  {
    short check[6];
    convertFromByteArray(_bufIn, check);
    cout << "Sent " << param[0] << " / " << param[1] << " / " << param[2] << " / " << param[3] << " / " << param[4] << " / " << param[5]
         << ", echo: " << check[0] << " / " << check[1] << " / " << check[2] << " / " << check[3] << " / " << check[4] << " / " << check[5] << endl;
  }
  return retval;
}

int main(int argc, char* argv[])
{
 
  if(argc<2)
  {
    cout << "usage: " << argv[0] << " <pwm ratio> [pwm ratio 2] [pwm ratio 3] [pwm ratio 4] [pwm ratio 5] [pwm ratio 6] [samples]" << endl;
    return 0;
  }

  _com = new SerialPort(_comPort, _baud);

  bool  retval = false;
  float responseF;
  int   responseI;
  short responseS[6];

  // Enable Motorcontroller
  while(!retval)
  {
    sendToMotorshieldI(0x18, 1, &responseI, true);
    retval = (responseI==1);
  }

  float t1 = 0.5f;
  sendToMotorshieldF(0x19, t1, &responseF, true);

  float gearRatio = GEARRATIO;
  sendToMotorshieldF(0x16, gearRatio, &responseF, true);

  float ticksPerRev = ENCODERTICKSPERREV;
  sendToMotorshieldF(0x17, ticksPerRev, &responseF, true);

  vector<float> vTimestamp;
  vector<float> vU;  // set value
  vector<float> vV;  // response Motor1
  vector<float> vV2; // response Motor2
  vector<float> vV3; // response Motor3
  vector<float> vV4; // response Motor4
  vector<float> vV5; // response Motor5
  vector<float> vV6; // response Motor6

  timeval clk;
  ::gettimeofday(&clk, 0);
  double t_start = static_cast<double>(clk.tv_sec) + static_cast<double>(clk.tv_usec) * 1.0e-6;

  long samples = 1500;
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

  if(argc>7)
    samples = atol(argv[7]);

  for(long i=0; i<samples; i++)
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

    bool retval = sendToMotorshieldS(0x00, u, &responseS, true);

    if(retval)
    {
      ::gettimeofday(&clk, 0);
      double t_now = static_cast<double>(clk.tv_sec) + static_cast<double>(clk.tv_usec) * 1.0e-6;

      vTimestamp.push_back(t_now-t_start);
      vU.push_back(u[0]);
      vV.push_back(((float)responseS[0])/VALUESCALE);
      vV2.push_back(((float)responseS[1])/VALUESCALE);
      vV3.push_back(((float)responseS[2])/VALUESCALE);
      vV4.push_back(((float)responseS[3])/VALUESCALE);
      vV5.push_back(((float)responseS[4])/VALUESCALE);
      vV6.push_back(((float)responseS[5])/VALUESCALE);
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
  for(unsigned int i=0; i<vTimestamp.size(); i++)
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

  delete _com;
}
