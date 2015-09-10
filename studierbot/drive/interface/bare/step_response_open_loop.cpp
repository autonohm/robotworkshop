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
    cout << "usage: " << argv[0] << " <voltage>" << endl;
    return 0;
  }
 
  vector<float> vt;
  vector<float> vu;
  vector<float> vv;

  SerialPort* com = new SerialPort(_comPort, _baud);

  char cmd[6];
  cmd[0] = 0x00;
  cmd[5] = 'F';

  short inc = 1;
  short val = 0;

  timeval clk;
  ::gettimeofday(&clk, 0);
  double t_start = static_cast<double>(clk.tv_sec) + static_cast<double>(clk.tv_usec) * 1.0e-6;

  short samples = 1500;
  int umax = 100;
  short u = atoi(argv[1]);
  for(short i=0; i<samples; i++)
  {
    // sine wave test
    //int imax = 1000;
    //int vmax  = 40;
    //u = sin(((double)i)/((double)imax)*M_PI*2.0)*vmax;

    shortValuesTo4ByteArray(u, 0, &cmd[1]);

    int sent = com->send(cmd, 6);

    char output[5];
    bool retval = com->receive(output, 5);

    if(retval & (output[4]=='F'))
    {
      short rpm1 = ((output[0] << 8) & 0xFF00) | (output[1]  & 0x00FF);
      short rpm2 = ((output[3] << 8) & 0xFF00) | (output[2]  & 0x00FF);


      ::gettimeofday(&clk, 0);
      double t_now = static_cast<double>(clk.tv_sec) + static_cast<double>(clk.tv_usec) * 1.0e-6;

      vt.push_back(t_now-t_start);
      vu.push_back(u);
      vv.push_back(((float)rpm1)/VALUESCALE);

      cout << (t_now-t_start) << " " << u << " " << vv.back() << endl;
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
  int ndeltaT = deltaT * 10000;
  deltaT = ((double)ndeltaT) / 10000.0;
  cout << "DeltaT: " << deltaT << " " << ndeltaT << endl;
  outInput << "0 0" << endl;
  outOutput << "0 0" << endl;
  for(int i=0; i<vt.size(); i++)
  {
    t += deltaT;
    outInput << t << " " << vu[i] << endl;
    outOutput << t << " " << vv[i] << endl;
  }
  outInput.close();
  outOutput.close();

  delete com;
}
