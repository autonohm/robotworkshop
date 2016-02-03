#ifndef MOTORCONTROLLER_H_
#define MOTORCONTROLLER_H_

#include "Encoder.h"

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

	/**
	 * Get maximum revolutions per minute
	 * @return maximum rpm
	 */
	int getRPMMax();

	/**
   * Set revolutions per minute
   * @param rpmLeft target rpm for left track
   * @param rpmRight target rpm for right track
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

  int _tty_fd;

	double _cmdMax;
	double _rpmMax;

	int _maxCmd;
	int _minCmd;

	Encoder _encoder;
};

#endif /* MOTORCONTROLLER_H_ */
