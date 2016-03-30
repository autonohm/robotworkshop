#include "Servocontroller.h"

#define SERVO_PITCH 15
#define SERVO_YAW 14

Servocontroller::Servocontroller()
{
	_servoControl.init(1,0x40);
	_servoControl.setPWMFreq(200); 
	_valuePWMServoYaw = _servoYawMid;
	_valuePWMServoPitch = _servoPitchMid;
	_servoControl.setPWM(SERVO_YAW,0,_valuePWMServoYaw);
	_servoControl.setPWM(SERVO_PITCH,0,_valuePWMServoPitch);
}

Servocontroller::~Servocontroller()
{
	_servoControl.reset();
}

void Servocontroller::setServo(double yaw, double pitch)
{
	if(yaw > 0) // Camera to the left
	{
		_valuePWMServoYaw -= 10;
	}
	else if(yaw < 0) // Camera to the right
	{
		_valuePWMServoYaw += 10;
	}
	if(pitch > 0) // Camera to the top
	{
		_valuePWMServoPitch += 10;
	}
	else if(pitch < 0) // Camera to the buttom
	{
		_valuePWMServoPitch -= 10;
	}
	
	if(_valuePWMServoYaw > _servoYawEnd) // Limiting to the upper limit parameter range
	{
		_valuePWMServoYaw = _servoYawEnd;
	}
	if(_valuePWMServoYaw < _servoYawBeg) // Limiting to the lower limit parameter range
	{
		_valuePWMServoYaw = _servoYawBeg;
	}

	if(_valuePWMServoPitch > _servoPitchEnd) // Limiting to the upper limit parameter range
	{
		_valuePWMServoPitch = _servoPitchEnd;
	}
	if(_valuePWMServoPitch < _servoPitchBeg) // Limiting to the lower limit parameter range
	{
		_valuePWMServoPitch = _servoPitchBeg;
	}

	_servoControl.setPWM(SERVO_YAW,0,_valuePWMServoYaw);
	_servoControl.setPWM(SERVO_PITCH,0,_valuePWMServoPitch);
}	
