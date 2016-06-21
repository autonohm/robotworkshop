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
#define EULER 0
#define ANTIWINDUP 0

SerialPort* _com;

char _bufCmd[6];
char _bufIn[5];

/**
 * Send float commands to motor shield
 * @param cmd command byte
 * @param param float parameter
 * @param echo verbosity of function, true provides command line output
 */
template<typename T>
bool sendToMotorshield(char cmd, T param, bool echo)
{
  _bufCmd[0] = cmd;
  convertTo4ByteArray(param, &_bufCmd[1]);
  int sent = _com->send(_bufCmd, 6);
  bool retval = _com->receive(_bufIn, 5);

  if(echo)
  {
    T check;
    convertFromByteArray(_bufIn, check);
    cout << "Sent " << param << ", echo: " << check << endl;
  }

  return retval;
}

bool sendToMotorshieldF(char cmd, float param, bool echo)
{
  _bufCmd[5] = 'F';
  return sendToMotorshield<float>(cmd, param, echo);
}

bool sendToMotorshieldI(char cmd, int param, bool echo)
{
  _bufCmd[5] = 'I';
  return sendToMotorshield<int>(cmd, param, echo);
}

bool sendToMotorshieldS(char cmd, short param[2], bool echo)
{
  _bufCmd[5] = 'S';
  bool retval = sendToMotorshield<short[2]>(cmd, param, false);
  if(echo)
  {
    short check[2];
    convertFromByteArray(_bufIn, check);
    cout << "Sent " << param[0] << " / " << param[1] << ", echo: " << check[0] << " / " << check[1] << endl;
  }
  return retval;
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

  float kp   = 7.1f;
  float ki   = 50.0f;
  float kd   = 0.0f;

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
      sendToMotorshieldF(0x05 + i, A[i], true);

    for(int i=0; i<3; i++)
      sendToMotorshieldF(0x0E + i, b[i], true);

    for(int i=0; i<3; i++)
      sendToMotorshieldF(0x11 + i, c[i], true);

    sendToMotorshieldF(0x14, d, true);
  }
  else
  {
    sendToMotorshieldF(0x02, kp, true);
    sendToMotorshieldF(0x03, ki, true);
    sendToMotorshieldF(0x04, kd, true);
    sendToMotorshieldI(0x15, ANTIWINDUP, true);
  }

  short val = 0;

  timeval clk;
  ::gettimeofday(&clk, 0);
  double t_start = static_cast<double>(clk.tv_sec) + static_cast<double>(clk.tv_usec) * 1.0e-6;

  short samples = 1500;
  short wset[2];
  wset[0] = w * VALUESCALE;
  wset[1] = 0;
  for(short i=0; i<samples; i++)
  {

    if(i>0.7*samples) wset[0] = 0;

    bool retval = sendToMotorshieldS(0x01, wset, true);

    if(retval)
    {
      short rpm1 = ((_bufIn[0] << 8) & 0xFF00) | (_bufIn[1]  & 0x00FF);
      short rpm2 = ((_bufIn[3] << 8) & 0xFF00) | (_bufIn[2]  & 0x00FF);

      ::gettimeofday(&clk, 0);
      double t_now = static_cast<double>(clk.tv_sec) + static_cast<double>(clk.tv_usec) * 1.0e-6;

      vTimestamp.push_back(t_now-t_start);
      vU.push_back(((float)wset[0])/VALUESCALE);
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
