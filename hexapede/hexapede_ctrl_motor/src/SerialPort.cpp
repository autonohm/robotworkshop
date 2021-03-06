#include "../../../hexapede/hexapede_ctrl_motor/src/SerialPort.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/wait.h>

#include <iostream>


SerialPort::SerialPort(const char *comPort, const speed_t baud)
{
  struct termios tty;
  char err[80];

  _fd = open(comPort, O_RDWR | O_NOCTTY | O_SYNC | O_NONBLOCK);
  if(_fd == -1)
  {
    strcpy(err, "open_port: Unable to open ");
    strcat(err, comPort);

    perror(err);
    exit(1);
  }
  fprintf(stderr, "%s has been successfully opened\n", comPort);

  memset(&tty, 0, sizeof(tty));  //tty to 0
  if(tcgetattr(_fd, &tty) != 0)
  {
    perror("error from tcgetattr");
  }

  cfsetispeed(&tty, baud);  //set baud input
  cfsetospeed(&tty, baud);  //set baud output

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
  // disable IGNBRK for mismatched speed tests; otherwise receive break
  // as \000 chars
  tty.c_iflag &= ~IGNBRK;         // ignore break signal
  tty.c_lflag = 0;                // no signaling chars, no echo,
  // no canonical processing
  tty.c_oflag = 0;                // no remapping, no delays
  /*MIN  ==  0;  TIME  >  0: TIME specifies the limit for a timer in tenths of a second.  The
   timer is started when read(2) is called.  read(2) returns either when at least  one  byte
   of  data is available, or when the timer expires.  If the timer expires without any input
   becoming available, read(2) returns 0*/
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 10;  //1 second read-timeout

  tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // shut off xon/xoff ctrl

  tty.c_cflag |= (CLOCAL | CREAD);  // ignore modem controls,
  // enable reading
  tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  if(tcsetattr(_fd, TCSANOW, &tty) != 0)
  {
    perror("error calling tcsetattr");
    exit(1);
  }
}

SerialPort::~SerialPort()
{

  close(_fd);

  fprintf(stderr, "serial port closed \n");
}

void SerialPort::send(char* msg, unsigned int len)
{
  write(_fd, msg, len);
}

bool SerialPort::receive(short* msg, unsigned int len)
{
  if(len<6)
  {
    perror("buffer passed to receive method to small (min. 6)");
    return false;
  }
  unsigned char buf[256];
  ssize_t readBytes = read(_fd, buf, 12);
  if(readBytes > 0)
  {
    for(int i=0; i<6; i++)
      msg[i] = (buf[2*i+1] << 8) | buf[2*i];
    return true;
  }
  else
  {
    return false;
  }
}

