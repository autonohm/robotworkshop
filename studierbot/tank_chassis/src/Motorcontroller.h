#ifndef MOTORCONTROLLER_H_
#define MOTORCONTROLLER_H_

//#include <PCA9685.h>

#include <string>

//#include "../../studierbot_base/src/Encoder.h"
#include "../../drive/interface/SerialPort.h"
#include "../../drive/interface/protocol.h"
#include "../../drive/interface/control.h"

#define EULER 0
#define ANTIWINDUP 0

/**
 * @class Motorcontroller
 * @author Stefan May
 * @date 31.05.2014
 */
class Motorcontroller
{

public:

  /**
   * Default constructor
   */
	Motorcontroller();

	/**
	 * Destructor
	 */
	virtual ~Motorcontroller();

	//void setVelocity(double linear, double angular, double speed);

	/**
	 * Get maximum revolutions per minute
	 * @return maximum rpm
	 */
	int getRPMMax();

	/**
	 * Set revolutions per minute
	 * @param rpmLeft rpm for left track
	 * @param rpmRight rpm for right track
	 */
	void setRPM(double rpmLeft, double rpmRight);

	/**
	 * Get revolutions per minute of left track
	 * @return rpm
	 */
	double getRPMLeft(double* dt);

	/**
   * Get revolutions per minute of right track
   * @return rpm
   */
	double getRPMRight(double* dt);

	/**
	 * Stop motors
	 */
  void stop();

private:
  void init();

  template<typename T>
  bool sendToMotorshield(char cmd, T param, bool echo);

  bool sendToMotorshieldS(char cmd, short param[2], bool echo);
  bool sendToMotorshieldI(char cmd, int param, bool echo);
  bool sendToMotorshieldF(char cmd, float param, bool echo);



  std::string _comPort;
  speed_t _baud;
  char _bufCmd[6];
  char _bufIn[5];
  SerialPort* _com;

	int _stopState;
	double _cmdMax;
	double _rpmMax;

	int _maxCmd;
	int _minCmd;



  float _kp;
  float _ki;
  float _kd;

	//Encoder _encoder;
};

#endif /* MOTORCONTROLLER_H_ */
