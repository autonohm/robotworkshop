#ifndef MOTORCONTROLLER_H_
#define MOTORCONTROLLER_H_

#include <string>

#include "../../drive/interface/SerialPort.h"
#include "../../drive/interface/protocol.h"
#include "../../drive/interface/control.h"

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
   * Get ratio of gearbox
   * @return gear ratio
   */
  double getGearRatio() const;

  /**
   * Set revolutions per minute
   * @param rpmLeft rpm for left track
   * @param rpmRight rpm for right track
   */
  void setRPM(double rpmLeft, double rpmRight);

  /**
   * Get revolutions per minute
   * param idx motor index
   * @return rpm
   */
  double getRPM(unsigned int idx);

  /**
   * Stop motors
   */
  void stop();

private:

  void init();

  /**
   * Send commands to motor shield
   * @param cmd command byte
   * @param param parameter
   * @param echo verbosity of function, true provides command line output
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
  double _rpmMax;
  float _gearRatio;
  float _encoderRatio;
  short _rpm[6];

  float _kp;
  float _ki;
  float _kd;

};

#endif /* MOTORCONTROLLER_H_ */
