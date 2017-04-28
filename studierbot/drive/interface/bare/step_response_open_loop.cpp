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

const char _comPort[] = "/dev/ttyACM0";
const speed_t _baud = B115200;

int main(int argc, char* argv[])
{
 
  if(argc<2)
  {
    cout << "usage: " << argv[0] << " <voltage>" << endl;
    return 0;
  }
 
  vector<float> vTimestamp;
  vector<float> vU;  // set value
  vector<float> vV;  // response Motor1
  vector<float> vV2; // response Motor2
  vector<float> vV3; // response Motor3
  vector<float> vV4; // response Motor4

  SerialPort* com = new SerialPort(_comPort, _baud);

  char bufCmd[10];
  bufCmd[0] = 0x00;
  bufCmd[9] = 'S';

  short inc = 1;
  short val = 0;

  timeval clk;
  ::gettimeofday(&clk, 0);
  double t_start = static_cast<double>(clk.tv_sec) + static_cast<double>(clk.tv_usec) * 1.0e-6;

  short samples = 1500;
  int umax = 100;
  short u[2];
  u[0] = atoi(argv[1]);
  u[1] = 0;

  for(short i=0; i<samples; i++)
  {
    if(i>0.9*samples) u[0] = 0;

    u[1] = u[0];
    u[2] = u[0];
    u[3] = u[0];

    convertTo8ByteArray(u, &bufCmd[1]);

    int sent = com->send(bufCmd, 10);

    char response[9];
    bool retval = com->receive(response, 9);

    if(retval & (response[8]=='S'))
    {
      short rpm1 = ((response[0] << 8) & 0xFF00) | (response[1]  & 0x00FF);
      short rpm2 = ((response[2] << 8) & 0xFF00) | (response[3]  & 0x00FF);
      short rpm3 = ((response[4] << 8) & 0xFF00) | (response[5]  & 0x00FF);
      short rpm4 = ((response[6] << 8) & 0xFF00) | (response[7]  & 0x00FF);

      ::gettimeofday(&clk, 0);
      double t_now = static_cast<double>(clk.tv_sec) + static_cast<double>(clk.tv_usec) * 1.0e-6;

      vTimestamp.push_back(t_now-t_start);
      vU.push_back(u[0]);
      vV.push_back(((float)rpm1)/VALUESCALE);
      vV2.push_back(((float)rpm2)/VALUESCALE);
      vV3.push_back(((float)rpm3)/VALUESCALE);
      vV4.push_back(((float)rpm4)/VALUESCALE);
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
    outOutput << t << " " << vV[i] << " " << vV2[i] << " " << vV3[i] << " " << vV4[i] <<  endl;
  }
  outInput.close();
  outOutput.close();

  cout << "files written to: " << filenameInput << " and " << filenameOutput << endl;

  delete com;
}
