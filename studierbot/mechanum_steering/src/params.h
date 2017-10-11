#ifndef PARAMS_H_
#define PARAMS_H_

/**
 * Motor-based parameters
 */
#define POLOLU_GEARMOTOR_37D 1

#if POLOLU_GEARMOTOR_37D
#define GEARRATIO 131.f
// Different versions of the motor controller provide different resolution
// The first versions use just a quarter of all possible events
//#define ENCODERRATIO 64.f  // Encoder ticks per motor revolution
#define ENCODERRATIO 16.f  // Encoder ticks per motor revolution
#define RPMMAX 80.f        // maximum revolution per minute of motor
#else
#define GEARRATIO 99.f
// Different versions of the motor controller provide different resolution
// The first versions use just a quarter of all possible events
//#define ENCODERRATIO 48.f  // Encoder ticks per motor revolution
#define ENCODERRATIO 12.f  // Encoder ticks per motor revolution
#define RPMMAX 76.f        // maximum revolution per minute of motor
#endif

/**
 * PID controller parameters
 */
#define PID_KP 3.1f
#define PID_KI 50.f
#define PID_KD 0.f

#define EULER 0
#define ANTIWINDUP 1

/**
 * Chassis-based parameters
 */
#define TRACK 0.3
#define WHEELBASE 0.19
#define WHEELDIAMETER 0.1
#endif
