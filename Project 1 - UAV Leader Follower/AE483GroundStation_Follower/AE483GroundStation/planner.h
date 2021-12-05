#ifndef PLANNER_H
#define PLANNER_H
//
// AE483GroundStation
// David Hanley
//
// planner.h
// This file contains all functions for planning. The most basic example
// would be to hold position at some desired (x,y,z,yaw).
//

/*----------------------------------------------------------------------*/
/*------------------------------ Preamble ------------------------------*/
/*----------------------------------------------------------------------*/

/*--------------- Includes ---------------*/
/*------------- End Includes -------------*/


/*--------------- Pragmas ---------------*/
/*------------- End Pragmas -------------*/


/*--------------- Defines ---------------*/

#define PI_f 3.14159265f
#define PI   3.14159265

/*------------- End Defines -------------*/


/*--------------- Globals ---------------*/

/*----- Structs -----*/
//Position
struct Pos_f
{
	float x;
	float y;
	float z;
};
typedef struct Pos_f Pos_f;

//Orientation (Euler)
struct Ori_f
{
	float roll;
	float pitch;
	float yaw;
};
typedef struct Ori_f Ori_f;
typedef struct Ori_f Euler_f;

//Quaternion
struct Quat_f
{
	float qx;
	float qy;
	float qz;
	float qw;
};
typedef struct Quat_f Quat_f;

//6DOF State
struct State_6_f
{
	Pos_f Pos;
	Ori_f Ori;
};
typedef struct State_6_f State6_f;

//Quadrotor XYZ Yaw
struct PosYaw_f
{
	Pos_f Pos;
	float yaw;
};
typedef struct PosYaw_f PosYaw_f;
/*--- End Structs ---*/
/*------------- End Globals -------------*/

/*--------------- Function Prototypes ---------------*/
// NOTE: The following two functions are used to convert MOCAP
// sensor data to rigid body pose. The MOCAP reference frame is
// a bit different from the z-down reference frame we normally
// call the "room". It is obvious from reading the code how this
// affects position. How it affects orientation (in particular,
// what Euler Angle convention is used in Planner_Quat2Euler_f,
// and how this gets mapped to the YPR convention that we use
// when describing the orientation of the body with respect to
// the room) is lost to history. The bottom line is that this
// conversion is hard-coded in the following two functions, and
// so one shouldn't use them for any other purpose.

void planner(State6_f QuadCurrent, State6_f ObstCurrent, PosYaw_f*QuadDesired, PosYaw_f QuadGoal);


/*----- Quat2Euler/Euler2Quat -----*/
//converts between quaternions and euler angles
//
// input:
//    *quat_param - quaternion struct to be converted from/converted to
//    *euler_param - euler struct to be converted from/converted to
void Planner_Quat2Euler_f(Quat_f *quat_param, Euler_f *euler_param);
/*--- End Quat2Euler/Euler2Quat ---*/

/*----- MakeState -----*/
// Create a state from parts such as Pos, Ori
//
// input:
//    *state_param - ptr to state struct that will be populated
//    *pos_param - ptr to position struct that will become part of the state
//    *ori_param - ptr to orientation struct that will become part of the state
void Planner_MakeState6f_PosQuat(State6_f *state_param, Pos_f *pos_param, Quat_f *quat_param);
/*--- End MakeState ---*/
/*------------- End Function Prototypes -------------*/
/*----------------------------------------------------------------------*/
/*------------------------------ Preamble ------------------------------*/
/*----------------------------------------------------------------------*/
#endif