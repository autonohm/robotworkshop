#ifndef SERVOCONTROLLER_H
#define SERVOCONTROLLER_H

#include <PCA9685.h>

/**
 * @class Servocontroller
 * @author Markus Runkel
 * @date 11.03.2016
 */
class Servocontroller
{

public:

 	/**
	 * Default constructor
	 */
	Servocontroller();

	/**
	 * Destructor
	 */
	virtual ~Servocontroller();

	/**
	 * Controllfunction for the servos
	 * @param yaw change of the z-axis
	 * @param pitch change of the y-axis
	 */
	void setServo(double yaw, double pitch);

private:

	PCA9685 _servoControl;
	int _valuePWMServoYaw, _valuePWMServoPitch;
	static const int _servoYawBeg = 400, _servoYawEnd = 2200, _servoYawMid = 1200; // Motor Batan S1213
	static const int _servoPitchBeg = 700, _servoPitchEnd = 1800, _servoPitchMid = 1100; // Motor Modelcraft MC-410 Standard

};

#endif
