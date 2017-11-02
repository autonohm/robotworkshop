/**
 * @author Stefan May
 * @date 10.10.2015
 * @brief Step response recorder for closed-loop interface of motor shield
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

#include "SerialPort.h"
#include "protocol.h"
#include "control.h"

using namespace std;

const char _comPort[] = "/dev/ttyACM0";
const speed_t _baud = B115200;

/**
 * Two types of gear motors are supported, e.g.
 * Pololu 131:1 Metal Gearmotor 37Dx57L mm with 64 CPR Encoder,        Pololu item#: 1447
 * Pololu  99:1 Metal Gearmotor 25Dx54L mm MP 12V with 48 CPR Encoder, Pololu item#: 3243
 */
#define POLOLU_GEARMOTOR_37D 1

#if POLOLU_GEARMOTOR_37D
#define GEARRATIO 131.f
#define ENCODERTICKSPERREV 16.f
#define RPMMAX 80
#else
#define GEARRATIO 99.f
#define ENCODERTICKSPERREV 48.f
#define RPMMAX 76
#endif

/**
 * PID controller parameter
 */
#define _KP 0.0f
#define _KI 1.f
#define _KD 0.f
#define ANTIWINDUP 0
#define EULER 0 //Use of Euler method, instead of classic PID controller

#define _INPUTSINE 0

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

  int  sent   = _com->send(_bufCmd, 14);
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
    cout << "usage: " << argv[0] << " <rpm> [rpm 2] [rpm 3] [rpm 4] [rpm 5] [rpm 6]" << endl;
    return 0;
  }

  vector<float> vTimestamp;
  vector<float> vU, vU2, vU3, vU4, vU5, vU6;  // Input Motor 1-6
  vector<float> vV, vV2, vV3, vV4, vV5, vV6;  // Response Motor 1-6

  float w[6] = {0, 0, 0, 0, 0, 0};

  w[0] = atof(argv[1]);

  if(argc>2)
    w[1] = atof(argv[2]);

  if(argc>3)
    w[2] = atof(argv[3]);

  if(argc>4)
    w[3] = atof(argv[4]);

  if(argc>5)
    w[4] = atof(argv[5]);

  if(argc>6)
    w[5] = atof(argv[6]);

  _com = new SerialPort(_comPort, _baud);

  char  bufCmd[14];
  char  bufIn[13];
  int   sent;
  float responseF;
  int   responseI;
  short responseS[6];
  bool  retval = false;

  float gearRatio = GEARRATIO;
  while(!retval)
  {
    sendToMotorshieldF(0x16, gearRatio, &responseF, true);
    retval = (gearRatio==responseF);
  }

  float ticksPerRev = ENCODERTICKSPERREV;
  sendToMotorshieldF(0x17, ticksPerRev, &responseF, true);

  float kp   = _KP;
  float ki   = _KI;
  float kd   = _KD;

  // parasitic time constant of closed-loop controller implemented in motor shield
  float tPar = 0.01;
  if(EULER)
  {
    float aTf[4];
    float bTf[4];

    pidToTransferFunction(kp, ki, kd, tPar, bTf, aTf, true);

    float A[9];
    float b[3];
    float c[3];
    float d;

    transferFunctionToStateControl(bTf, aTf, second, A, b, c, d, true);

    for(int i=0; i<9; i++)
      sendToMotorshieldF(0x05 + i, A[i], &responseF, true);

    for(int i=0; i<3; i++)
      sendToMotorshieldF(0x0E + i, b[i], &responseF, true);

    for(int i=0; i<3; i++)
      sendToMotorshieldF(0x11 + i, c[i], &responseF, true);

    sendToMotorshieldF(0x14, d, &responseF, true);
  }
  else
  {
    sendToMotorshieldF(0x02, kp, &responseF, true);
    sendToMotorshieldF(0x03, ki, &responseF, true);
    sendToMotorshieldF(0x04, kd, &responseF, true);
    sendToMotorshieldI(0x15, ANTIWINDUP, &responseI, true);
  }

  short val = 0;

  timeval clk;
  ::gettimeofday(&clk, 0);
  double t_start = static_cast<double>(clk.tv_sec) + static_cast<double>(clk.tv_usec) * 1.0e-6;

  int samples = 1500;
  short wsetBase[6];
  short wset[6];
  double wsetOffset[6];
  wsetBase[0] = w[0] * VALUESCALE;
  wsetBase[1] = w[1] * VALUESCALE;
  wsetBase[2] = w[2] * VALUESCALE;
  wsetBase[3] = w[3] * VALUESCALE;
  wsetBase[4] = w[4] * VALUESCALE;
  wsetBase[5] = w[5] * VALUESCALE;

  wsetOffset[0] = 0.0;
  wsetOffset[1] = M_PI/6.0;
  wsetOffset[2] = 2.0*M_PI/6.0;
  wsetOffset[3] = 3.0*M_PI/6.0;
  wsetOffset[4] = 4.0*M_PI/6.0;
  wsetOffset[5] = 5.0*M_PI/6.0;

  for(int i=0; i<samples; i++)
  {

    if(i>samples-50)
    {
      wsetBase[0] = 0;
      wsetBase[1] = 0;
      wsetBase[2] = 0;
      wsetBase[3] = 0;
      wsetBase[4] = 0;
      wsetBase[5] = 0;
    }

#if _INPUTSINE
    for(int j=0; j<6; j++)
      wset[j] = wsetBase[j] * sin(((double)i)/180.0*(M_PI/3.0) + wsetOffset[j]);

    bool retval = sendToMotorshieldS(0x01, wset, &responseS, true);
#else
    for(int j=0; j<6; j++)
      wset[j] = wsetBase[j];

    bool retval = sendToMotorshieldS(0x01, wset, &responseS, true);
#endif

    if(retval)
    {
      short rpm1 = ((_bufIn[0]  << 8) & 0xFF00) | (_bufIn[1]  & 0x00FF);
      short rpm2 = ((_bufIn[2]  << 8) & 0xFF00) | (_bufIn[3]  & 0x00FF);
      short rpm3 = ((_bufIn[4]  << 8) & 0xFF00) | (_bufIn[5]  & 0x00FF);
      short rpm4 = ((_bufIn[6]  << 8) & 0xFF00) | (_bufIn[7]  & 0x00FF);
      short rpm5 = ((_bufIn[8]  << 8) & 0xFF00) | (_bufIn[9]  & 0x00FF);
      short rpm6 = ((_bufIn[10] << 8) & 0xFF00) | (_bufIn[11] & 0x00FF);

      ::gettimeofday(&clk, 0);
      double t_now = static_cast<double>(clk.tv_sec) + static_cast<double>(clk.tv_usec) * 1.0e-6;

      vTimestamp.push_back(t_now-t_start);
      vU.push_back(((float)wset[0])/VALUESCALE);
      vU2.push_back(((float)wset[1])/VALUESCALE);
      vU3.push_back(((float)wset[2])/VALUESCALE);
      vU4.push_back(((float)wset[3])/VALUESCALE);
      vU5.push_back(((float)wset[4])/VALUESCALE);
      vU6.push_back(((float)wset[5])/VALUESCALE);
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
    outInput << t << " " << vU[i];
    if(argc>2) outInput << " " << vU2[i];
    if(argc>3) outInput << " " << vU3[i];
    if(argc>4) outInput << " " << vU4[i];
    if(argc>5) outInput << " " << vU5[i];
    if(argc>6) outInput << " " << vU6[i];
    outInput << endl;
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
