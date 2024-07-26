#pragma once

const short converter[]={1, -1, -1, -1, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1};

// classic static and dynamic friction
//#define FRICTION_TYPE_STATIC_DYNAMIC

// friction as described in paper "humanoid realistic
// simulator"
#define FRICTION_TYPE_PAPER

//#define SIMPLE_TOLERANCE_GEARS
#define FLEXIBLE_TOLERANCE_GEARS

// Simulate the impedance of the motor
#define IMPEDANCE

// Simulate the maximum battery voltage
#define MAX_VOLT

// Use the torque control instead of sending the desired velocity to the joints
#define TORQUE_CONTROL

// Interpolate the angles if the simulation time step length is shorter than the one of the read angles.
#define ANGLE_INTERPOLATION

// Simulation ends when robot has fallen down
#define DO_NOT_FALL

// Time step length of the source angles.
#define SOURCE_DT	0.02

// Turn on log files
//#define LOGGING
//#define TRACING

// Some testing stuff
//#define FLEXIBLE_TEST
//#define FLEXBOX
//#define EA_TEST

// No window
//#define SOFT_HEADLESS

#ifndef FLEXIBLE_TEST
// Number of Naos to simulate. Important when used for evaluation of the fitness function (EA).
// Note, they will not run in parallel on multi-core systems
#define NUM_OF_NAOS	1
#else
#define NUM_OF_NAOS	0
#endif

// EA stuff
#define QUIET
#define CLEANUP


// Insert moveable joints for the arms (maybe more unstable and slower)
//#define MOVEABLE_ARMS

// Insert arms
#define ARMS

// Insert the battery
//#define AKKU

// Use ODE slipping
#define SLIPPING

// Number of cluster worker nodes (for fitness function evaluation)
#define NUM_OF_NODES	180

// Number of different gear types in robot.
#define NUM_OF_GEARTYPES	2

//#define EA_MODE

//#define CART_MODEL

class AngleFromCSV;
class DortmundWalkingEngineAdapter;

// Set the source for the angles

// Hard coded angles
// LEGACY, now virtual base class is used
// typedef AngleFromCSV AngleAdapter;

// Angles from dynamic walking engine
//typedef DortmundWalkingEngineAdapter AngleAdapter;