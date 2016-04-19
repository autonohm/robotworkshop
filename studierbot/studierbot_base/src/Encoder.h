#ifndef ENCODER_H
#define ENCODER_H

#include <wiringPi.h>
#include <ros/ros.h>

/**
 * @class Encoder
 * @author Stefan May
 * @date 31.05.2014
 */
class Encoder
{
public:

  /**
   * Default constructor
   */
  Encoder();

  /**
   * Destructor
   */
  ~Encoder();

  /**
   * Set direction of left motors (internally the encoders cannot determine this)
   * @param forward true=forward, false=backward
   */
/*  void setForwardLeft(bool forward);

  bool getForwardRight();
	bool getForwardLeft();

  bool isStopped();

  /**
   * Set direction of right motors (internally the encoders cannot determine this)
   * @param forward true=forward, false=backward
   */
//  void setForwardRight(bool forward);

  /**
   * Get encoder ticks per revolution
   * @return encoder ticks
   */
  double getTicksPerTurn();

  /**
   * Get encoder ticks since last call
   * @param ticks encoder ticks counted for left motors
   * @param dt time elapsed since last call
   */
  void getTicksLeft(double* ticks, double* dt);

  /**
   * Get encoder ticks since last call
   * @param ticks encoder ticks counted for right motors
   * @param dt time elapsed since last call
   */
   void getTicksRight(double* ticks, double* dt);

private:

  /**
   * number of ticks per motor revolution
   */
  double _encoderRatio;

  /**
   * Timestamp of last call for getting left encoder ticks
   */
  ros::Time _timestampLeft;

  /**
   * Timestamp of last call for getting right encoder ticks
   */
  ros::Time _timestampRight;

  long _ticksRightOld;
	long _ticksLeftOld;


  ros::Time _timestampStop;

  long _ticksStopped;
};

#endif
