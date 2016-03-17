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
	if(yaw > 0) // Joystic-Knopf nach links; Kamera nach links
	{
		_valuePWMServoYaw -= 10;
	}
	else if(yaw < 0) // Joystic-Knopf nach rechts; Kamera nach rechts
	{
		_valuePWMServoYaw += 10;
	}
	if(pitch > 0) // Joystic-Knopf nach oben; Kamera nach oben
	{
		_valuePWMServoPitch += 10;
	}
	else if(pitch < 0) // Joystic-Knopf nach unten; Kamera nach unten
	{
		_valuePWMServoPitch -= 10;
	}
	
	if(_valuePWMServoYaw > _servoYawEnd) // Wenn das obere Ende des Kennwertbereichs überschritten wird,
					     // wird der Zähler auf den höchsten Wert gesetzt
	{
		_valuePWMServoYaw = _servoYawEnd;
	}
	if(_valuePWMServoYaw < _servoYawBeg) // Wenn das unter Ende des Kennwertbereichs unterschritten wird,
					     // wird der Zähler auf den niedrigsten Wert gesetzt
	{
		_valuePWMServoYaw = _servoYawBeg;
	}

	if(_valuePWMServoPitch > _servoPitchEnd) // Wenn das obere Ende des Kennwertbereichs überschritten wird,
					     // wird der Zähler auf den höchsten Wert gesetzt
	{
		_valuePWMServoPitch = _servoPitchEnd;
	}
	if(_valuePWMServoPitch < _servoPitchBeg) // Wenn das unter Ende des Kennwertbereichs unterschritten wird,
					     // wird der Zähler auf den niedrigsten Wert gesetzt
	{
		_valuePWMServoPitch = _servoPitchBeg;
	}

	_servoControl.setPWM(SERVO_YAW,0,_valuePWMServoYaw);
	_servoControl.setPWM(SERVO_PITCH,0,_valuePWMServoPitch);
}	
