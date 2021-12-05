//
// AE483GroundStation
// David Hanley
//
// planner.c
// This file contains all functions for planning. The most basic example
// would be to hold position at some desired (x,y,z,yaw).
//





/*----------------------------------------------------------------------*/
/*------------------------------ Preamble ------------------------------*/
/*----------------------------------------------------------------------*/

/*--------------- Includes ---------------*/

//system
#include <math.h>

//planner
#include "planner.h"

/*------------- End Includes -------------*/

/*----------------------------------------------------------------------*/
/*------------------------------ Preamble ------------------------------*/
/*----------------------------------------------------------------------*/

/*----------------------------------------------------------------------*/
/*----------------------------- Functions ------------------------------*/
/*----------------------------------------------------------------------*/

void planner(State6_f QuadCurrent, State6_f ObstCurrent, PosYaw_f *QuadDesired, PosYaw_f
QuadGoal)
{
	QuadDesired->Pos.x = ObstCurrent.Pos.x - 1.00;
	QuadDesired->Pos.y = ObstCurrent.Pos.y;
	QuadDesired->Pos.z = ObstCurrent.Pos.z;
}


/*--------------- Quat2Euler/Euler2Quat ---------------*/
void Planner_Quat2Euler_f(Quat_f *quat_param, Euler_f *euler_param)
{
	//variables
	float sqw = quat_param->qw*quat_param->qw;
	float sqx = quat_param->qx*quat_param->qx;
	float sqy = quat_param->qy*quat_param->qy;
	float sqz = quat_param->qz*quat_param->qz;
	float unit = sqx + sqy + sqz + sqw; // if normalised is one, otherwise is correction factor
	float tx, ty, tz;

	//singularity tests
	float test = quat_param->qx*quat_param->qy + quat_param->qz*quat_param->qw;
	if (test > 0.499*unit) { // singularity at north pole
		ty = 2 * atan2f(quat_param->qx, quat_param->qw);
		tz = PI_f / 2;
		tx = 0;
	} else if (test < -0.499*unit) { // singularity at south pole
		ty = -2 * atan2f(quat_param->qx, quat_param->qw);
		tz = -PI_f / 2;
		tx = 0;
	} else { //no singularity
		ty = atan2f(2 * quat_param->qy*quat_param->qw - 2 * quat_param->qx*quat_param->qz, sqx - sqy - sqz + sqw);
		tz = asinf(2 * test / unit);
		tx = atan2f(2 * quat_param->qx*quat_param->qw - 2 * quat_param->qy*quat_param->qz, -sqx + sqy - sqz + sqw);
	}

	// change of reference
	euler_param->roll = tz;
	euler_param->pitch = -tx;
	euler_param->yaw = -ty;
}
/*------------- End Quat2Euler/Euler2Quat -------------*/

/*----- MakeState -----*/
void Planner_MakeState6f_PosQuat(State6_f *state_param, Pos_f *pos_param, Quat_f *quat_param)
{
	state_param->Pos.x = pos_param->z;
	state_param->Pos.y = -pos_param->x;
	state_param->Pos.z = -pos_param->y;
	Planner_Quat2Euler_f(quat_param, &(state_param->Ori));
}
/*--- End MakeState ---*/
/*----------------------------------------------------------------------*/
/*--------------------------- End Functions ----------------------------*/
/*----------------------------------------------------------------------*/