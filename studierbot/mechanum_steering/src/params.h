#ifndef PARAMS_H_
#define PARAMS_H_

/**
 * Motor-based parameters
 */
#define POLOLU_GEARMOTOR_37D 0

#if POLOLU_GEARMOTOR_37D
#define GEARRATIO 131.f
#define ENCODERRATIO 64.f  // Encoder ticks per motor revolution
#define RPMMAX 80          // maximum revolution per minute of motor
#else
#define GEARRATIO 99.f
#define ENCODERRATIO 48.f  // Encoder ticks per motor revolution
#define RPMMAX 76          // maximum revolution per minute of motor
#endif

/**
 * PID controller parameters
 */
#define PID_KP 3.1f
#define PID_KI 50.f
#define PID_KD 0.f

#define EULER 0
#define ANTIWINDUP 0

/**
 * Chassis-based parameters
 */
#define TRACK 0.28

#define PINIONCIRCUMFERENCE (0.06 * M_PI)

#endif
