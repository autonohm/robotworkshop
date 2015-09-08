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

using namespace std;

const char _comPort[] = "/dev/ttyACM0";
const speed_t _baud = B115200;

int main(int argc, char* argv[])
{

  if(argc<2)
  {
    cout << "usage: " << argv[0] << " <rpm>" << endl;
    return 0;
  }

  vector<float> vt;
  vector<float> u;
  vector<float> v;

  float w = atof(argv[1]);

  SerialPort* com = new SerialPort(_comPort, _baud);

  char bufCmd[6];
  char bufIn[5];
  bool retval;
  int sent;

  float kp = 1.0f;
  float ki = 10.0f;
  float kd = 0.0f;

  int ikp = kp * PARAMSCALE;
  int iki = ki * PARAMSCALE;
  int ikd = kd * PARAMSCALE;
  int check;

  bufCmd[0] = 0x02;
  intTo4ByteArray(ikp, &bufCmd[1]);
  bufCmd[5] = 'F';
  sent = com->send(bufCmd, 6);
  retval = com->receive(bufIn, 5);
  check = byteArrayToInt(bufIn);
  cout << "Sent Kp: " << ikp << ", echo: " << check << endl;

  bufCmd[0] = 0x03;
  intTo4ByteArray(iki, &bufCmd[1]);
  sent = com->send(bufCmd, 6);
  retval = com->receive(bufIn, 5);
  check = byteArrayToInt(bufIn);
  cout << "Sent Ki: " << iki << ", echo: " << check << endl;

  bufCmd[0] = 0x04;
  intTo4ByteArray(ikd, &bufCmd[1]);
  sent = com->send(bufCmd, 6);
  retval = com->receive(bufIn, 5);
  check = byteArrayToInt(bufIn);
  cout << "Sent Kd: " << ikd << ", echo: " << check << endl;

  bufCmd[0] = 0x01;

  short val = 0;

  timeval clk;
  ::gettimeofday(&clk, 0);
  double t_start = static_cast<double>(clk.tv_sec) + static_cast<double>(clk.tv_usec) * 1.0e-6;

  short samples = 1500;
  int wset = w * VALUESCALE;
  for(short i=0; i<samples; i++)
  {
    intTo4ByteArray(wset, &bufCmd[1]);

    int sent = com->send(bufCmd, 6);

    bool retval = com->receive(bufIn, 5);

    if(retval & (bufIn[4]=='F'))
    {
      short rpm1 = ((bufIn[0] << 8) & 0xFF00) | (bufIn[1]  & 0x00FF);
      short rpm2 = ((bufIn[3] << 8) & 0xFF00) | (bufIn[2]  & 0x00FF);

      ::gettimeofday(&clk, 0);
      double t_now = static_cast<double>(clk.tv_sec) + static_cast<double>(clk.tv_usec) * 1.0e-6;

      vt.push_back(t_now-t_start);
      u.push_back(((float)wset)/VALUESCALE);
      v.push_back(((float)rpm1)/VALUESCALE);

      cout << (t_now-t_start) << " " << u.back() << " " << v.back() << endl;
    }
    else
      cout << "failed to receive" << endl;
  }


  // Generate output files
  std::ofstream outInput;
  std::ofstream outOutput;

  std::ostringstream oss;
  oss << "Eingang.sim";

  outInput.open(oss.str().c_str(), std::ios::out);

  std::ostringstream oss2;
  oss2 << "Ausgang.sim";
  outOutput.open(oss2.str().c_str(), std::ios::out);
  double t = 0.f;
  double deltaT = ((vt[vt.size()-1] - vt[0])) / ((double)vt.size());

  // Round seconds to 4 digits
  int ndeltaT = deltaT * 10000;
  deltaT = ((double)ndeltaT) / 10000.0;

  cout << "DeltaT: " << deltaT << " " << ndeltaT << endl;

  // Write files
  outInput << "0 0" << endl;
  outOutput << "0 0" << endl;
  for(int i=0; i<vt.size(); i++)
  {
    t += deltaT;
    outInput << t << " " << u[i] << endl;
    outOutput << t << " " << v[i] << endl;
  }
  outInput.close();
  outOutput.close();

  delete com;
}
