#include "Encoder.h"

#include <stdio.h>
#include <errno.h>
#include <string.h>

#include "params.h"

long _incLeft  = 1;
long _incRight = 1;

long _ticks1   = 0;
long _ticks2   = 0;
long _ticks3   = 0;
long _ticks4   = 0;

int _key3 = 0;

void interrupt1()
{
  _ticks1+=_incLeft;
}

void interrupt2()
{
  _ticks2+=_incLeft;
}

void interrupt3()
{
  _ticks3+=_incRight;
}

void interrupt4()
{
  _ticks4+=_incRight;
}

// GPIO PIN MAPPING
#define BUTTON_PIN1 18
#define BUTTON_PIN2 23
#define BUTTON_PIN3 24
#define BUTTON_PIN4 25

Encoder::Encoder()
{
  _encoderRatio       = ENCODERRATIO;

  _timestampLeft      = ros::Time::now();
  _timestampRight     = ros::Time::now();

  if (wiringPiSetupSys() < 0)
      fprintf (stderr, "Unable to setup wiringPi: %s\n", strerror (errno));

  //pinMode(BUTTON_PIN1, INPUT);
  //pinMode(BUTTON_PIN2, INPUT);
  pinMode(BUTTON_PIN3, INPUT);
  //pinMode(BUTTON_PIN4, INPUT);

  //if (wiringPiISR (BUTTON_PIN1, INT_EDGE_RISING, interrupt1) < 0 )
  //  fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno));

  //if (wiringPiISR (BUTTON_PIN2, INT_EDGE_RISING, interrupt2) < 0 )
  //  fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno));

  if (wiringPiISR (BUTTON_PIN3, INT_EDGE_RISING, interrupt3) < 0 )
    fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno));

  //if (wiringPiISR (BUTTON_PIN4, INT_EDGE_RISING, interrupt4) < 0 )
  //  fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno));
}

Encoder::~Encoder()
{

}

double Encoder::getTicksPerTurn()
{
  return _encoderRatio;
}

/*
void Encoder::setForwardLeft(bool forward)
{
  forward == true ? _incLeft = 1 : _incLeft = -1;
}

void Encoder::setForwardRight(bool forward)
{
  forward == true ? _incRight = 1 : _incRight = -1;
}

bool Encoder::getForwardRight()
{
  return _incRight==1;
}

bool Encoder::getForwardLeft()
{
  return _incLeft==1;
}

bool Encoder::isStopped()
{
  ros::Time tNow = ros::Time::now();
  if(_ticksStopped != _ticks3)
  {
    _ticksStopped = _ticks3;
    _timestampStop = tNow;
  }

  ros::Duration dur = tNow - _timestampStop;
  double dt = dur.toSec();

  return dt>1.0;
}*/

void Encoder::getTicksLeft(double* ticks, double* dt)
{
  // take smaller integral of both encoders
  long t = _ticks1;
	*ticks = (double)(t-_ticksLeftOld);
	_ticksLeftOld = t;
  //if(t>_ticks2) t = _ticks2;

  //_ticks1 = 0;
  //_ticks2 = 0;

  ros::Time tNow = ros::Time::now();
  //*ticks = (double)t;
  ros::Duration dur = tNow - _timestampLeft;
  *dt = dur.toSec();
  _timestampLeft = tNow;
}

void Encoder::getTicksRight(double* ticks, double* dt)
{
  // take smaller integral of both encoders
  long t = _ticks3;
  *ticks = (double)(t-_ticksRightOld);
  _ticksRightOld = t;
  //if(t>_ticks4) t = _ticks4;

  //_ticks3 = 0;
  //_ticks4 = 0;

  ros::Time tNow = ros::Time::now();
  ros::Duration dur = tNow - _timestampRight;
  *dt = dur.toSec();
  _timestampRight = tNow;
}
