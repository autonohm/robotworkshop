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
 * Use of Euler method, instead of classic PID controller
 */
#define EULER 1

SerialPort* _com;

/**
 * Send float commands to motor shield
 * @param cmd command byte
 * @param param float parameter
 * @param echo verbosity of function, true provides command line output
 */
void sendToMotorshield(char cmd, float param, bool echo)
{
  char bufCmd[6];
  char bufIn[5];
  float check;

  bufCmd[0] = cmd;
  floatTo4ByteArray(param, &bufCmd[1]);
  bufCmd[5] = 'F';
  int sent = _com->send(bufCmd, 6);
  bool retval = _com->receive(bufIn, 5);
  check = byteArrayToFloat(bufIn);

  if(echo)
    cout << "Sent " << param << ", echo: " << check << endl;
}

int main(int argc, char* argv[])
{

  if(argc<2)
  {
    cout << "usage: " << argv[0] << " <rpm>" << endl;
    return 0;
  }

  vector<float> vTimestamp;
  vector<float> vU;
  vector<float> vV;

  float w = atof(argv[1]);

  _com = new SerialPort(_comPort, _baud);

  char bufCmd[6];
  char bufIn[5];
  bool retval;
  int sent;

  bufCmd[5] = 'F';

  float kp   = 5.1f;
  float ki   = 50.0f;
  float kd   = 0.06f;

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
      sendToMotorshield(0x05 + i, A[i], true);

    for(int i=0; i<3; i++)
      sendToMotorshield(0x0E + i, b[i], true);

    for(int i=0; i<3; i++)
      sendToMotorshield(0x11 + i, c[i], true);

    sendToMotorshield(0x14, d, true);
  }
  else
  {
    sendToMotorshield(0x02, kp, true);
    sendToMotorshield(0x03, ki, true);
    sendToMotorshield(0x04, kd, true);
  }

  bufCmd[0] = 0x01;

  short val = 0;

  timeval clk;
  ::gettimeofday(&clk, 0);
  double t_start = static_cast<double>(clk.tv_sec) + static_cast<double>(clk.tv_usec) * 1.0e-6;

  short samples = 1500;
  short wset = w * VALUESCALE;
  for(short i=0; i<samples; i++)
  {
    shortValuesTo4ByteArray(wset, 0, &bufCmd[1]);

    int sent = _com->send(bufCmd, 6);

    bool retval = _com->receive(bufIn, 5);

    if(retval & (bufIn[4]=='F'))
    {
      short rpm1 = ((bufIn[0] << 8) & 0xFF00) | (bufIn[1]  & 0x00FF);
      short rpm2 = ((bufIn[3] << 8) & 0xFF00) | (bufIn[2]  & 0x00FF);

      ::gettimeofday(&clk, 0);
      double t_now = static_cast<double>(clk.tv_sec) + static_cast<double>(clk.tv_usec) * 1.0e-6;

      vTimestamp.push_back(t_now-t_start);
      vU.push_back(((float)wset)/VALUESCALE);
      vV.push_back(((float)rpm1)/VALUESCALE);

      //cout << (t_now-t_start) << " " << vU.back() << " " << vV.back() << endl;
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
    outOutput << t << " " << vV[i] << endl;
  }
  outInput.close();
  outOutput.close();

  cout << "files written to: " << filenameInput << " and " << filenameOutput << endl;

  delete _com;
}
