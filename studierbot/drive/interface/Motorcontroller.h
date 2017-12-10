#ifndef MOTORCONTROLLER_H_
#define MOTORCONTROLLER_H_

#include <string>

#include "SerialPort.h"
#include "protocol.h"
#include "control.h"

struct ChassisParams
{
  float track;
  float wheelBase;
  float wheelDiameter;
  int   frontLeft;
  int   frontRight;
  int   centerLeft;
  int   centerRight;
  int   rearLeft;
  int   rearRight;
  int   direction;

  ChassisParams()
  {
    track         = 0.f;
    wheelBase     = 0.f;
    wheelDiameter = 0.f;
    frontLeft     = 0;
    frontRight    = 0;
    centerLeft    = 0;
    centerRight   = 0;
    rearLeft      = 0;
    rearRight     = 0;
    direction     = 0;
  }
};

enum MotorType {Pololu_Gearmotor_25D, Pololu_Gearmotor_37D, Faulhaber_16002};
struct MotorParams
{
  float     gearRatio;
  float     encoderRatio;
  float     rpmMax;
  float     kp;
  float     ki;
  float     kd;
  int       antiWindup;
  MotorType type;

  /**
   * Standard constructor assigns default parameters
   * @param[in] t motor type
   */
  MotorParams(MotorType t)
  {
    type = t;
    switch(type)
    {
    case Pololu_Gearmotor_25D:
      gearRatio    = 99.f;
      encoderRatio = 48.f;
      rpmMax       = 76.f;
      break;
    case Pololu_Gearmotor_37D:
      gearRatio    = 131.f;
      encoderRatio = 64.f;
      rpmMax       = 80.f;
      break;
    case Faulhaber_16002:
      gearRatio    = 64.f;
      encoderRatio = 48.f;
      rpmMax       = 120.f;
      break;
    }
    kp         = 1.f;
    ki         = 0.f;
    kd         = 0.f;
    antiWindup = 1;
  }

  /**
   * Copy constructor
   * @param[in] p parameter instance to be copied
   */
  MotorParams(const MotorParams &p)
  {
    type = p.type;
    gearRatio = p.gearRatio;
    encoderRatio = p.encoderRatio;
    rpmMax = p.rpmMax;
    kp = p.kp;
    ki = p.ki;
    kd = p.kd;
    antiWindup = p.antiWindup;
  }
};

/**
 * @class Motorcontroller
 * @author Stefan May
 * @date 31.05.2014
 */
class Motorcontroller
{

public:

  /**
   * Constructor
   * @param[in] params gear motor parameters, PID controller parameters
   */
  Motorcontroller(MotorParams params);

  /**
   * Destructor
   */
  virtual ~Motorcontroller();

  /**
   * Get maximum revolutions per minute
   * @return maximum rpm
   */
  float getRPMMax();

  /**
   * Get ratio of gearbox
   * @return gear ratio
   */
  float getGearRatio() const;

  /**
   * Set revolutions per minute
   * @param[in] rpm 6-channel rpm value
   */
  void setRPM(float rpm[6]);

  /**
   * Get revolutions per minute
   * param[in] idx motor index
   * @return rpm
   */
  float getRPM(unsigned int idx);

  /**
   * Stop motors
   */
  void stop();

private:

  void init();

  /**
   * Send commands to motor shield
   * @param[in] cmd command byte
   * @param[in] param parameter
   * @param[in] echo verbosity of function, true provides command line output
   */
  template<typename T>
  bool sendToMotorshield(char cmd, T param, bool echo);
  bool sendToMotorshieldS(char cmd, short param[2], bool echo);
  bool sendToMotorshieldI(char cmd, int param, bool echo);
  bool sendToMotorshieldF(char cmd, float param, bool echo);


  std::string _comPort;
  speed_t _baud;
  char _bufCmd[14];
  char _bufResponse[13];
  SerialPort* _com;

  int _stopState;
  float _rpmMax;
  float _gearRatio;
  float _encoderRatio;
  short _rpm[6];

  float _kp;
  float _ki;
  float _kd;
  int   _antiWindup;

};

#endif /* MOTORCONTROLLER_H_ */
