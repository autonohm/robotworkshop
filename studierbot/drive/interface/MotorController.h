#ifndef MOTORCONTROLLER_H_
#define MOTORCONTROLLER_H_

#include <string>
#include <map>

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

struct MotorParams
{
  std::string port;         // Device file link to either serial or can interface
  float       gearRatio;    // Ratio between motor and wheel turns
  float       encoderRatio; // ticks per motor revolution
  float       rpmMax;       // Maximum wheel revolutions per minute
  float       kp;           // Proportional factor variable of PID controller
  float       ki;           // Integration factor of PID controller
  float       kd;           // Differential factor of PID controller
  int         antiWindup;   // Avoid integration, if maximum control value is reached

  MotorParams()
  {
    port         = std::string("");
    gearRatio    = 0.f;
    encoderRatio = 0.f;
    rpmMax       = 0.f;
    kp           = 1.f;
    ki           = 0.f;
    kd           = 0.f;
    antiWindup   = 1;
  }

  /**
   * Copy constructor
   * @param[in] p parameter instance to be copied
   */
  MotorParams(const MotorParams &p)
  {
    gearRatio    = p.gearRatio;
    encoderRatio = p.encoderRatio;
    rpmMax       = p.rpmMax;
    kp           = p.kp;
    ki           = p.ki;
    kd           = p.kd;
    antiWindup   = p.antiWindup;
  }
};

enum MotorControllerChannel {CH0, CH1, CH2, CH3, CH4, CH5, CH6, CH7, CH8, CH9, CH10, CH11, CH12, CH13, CH14, CH15};

/**
 * @class MotorController
 * @author Stefan May
 * @date 31.05.2014
 */
class MotorController
{

public:

  /**
   * Constructor
   * @param[in] params gear motor parameters, PID controller parameters
   */
  MotorController(MotorParams &params);

  /**
   * Destructor
   */
  virtual ~MotorController();

  /**
   * Enable device
   * @return enable state
   */
  virtual bool enable() = 0;

  /**
   * Get maximum revolutions per minute
   * @return maximum rpm
   */
  float getRPMMax() const;

  /**
   * Get ratio of gearbox
   * @return gear ratio
   */
  float getGearRatio() const;

  /**
   * Set motor revolutions per minute
   * @param[in] rpm set point value, see concrete device for supported number of channels
   */
  virtual void setRPM(std::map<MotorControllerChannel, float> rpm) = 0;

  /**
   * Get revolutions per minute
   * param[in] idx motor index
   * @return rpm
   */
  virtual float getRPM(unsigned int idx) = 0;

  /**
   * Stop motors
   */
  virtual void stop() = 0;

protected:

  MotorParams _params;
};

#endif /* MOTORCONTROLLER_H_ */
