#ifndef _SERIAL_PORT_H_
#define _SERIAL_PORT_H_

#include <sys/types.h>
#include <termios.h>

class SerialPort
{
public:

	SerialPort(const char *comPort, const speed_t baud);

	~SerialPort();

	void send(char* msg, unsigned int len);

	bool receive(short* msg, unsigned int len);

private:
	int _fd;

};

#endif /* _SERIAL_PORT_H_ */
