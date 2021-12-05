/*
//
// AE483GroundStation
// David Hanley
// Rachit Singhvi
// Chris Andres
// Joseph Grigus
// Elijah Chen
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

//Parameter (change all of these as appropriate)

float katt = 1.5;
float batt = 0.5;
float krep = 1;
float brep = 0.5;
float kdes = 0.1;
float bdes = 0.01;
float r = 0.500; // drone radius
float s = 0.250; // obstacle radius

/*----------------------------------------------------------------------*/
/*----------------------------- Functions ------------------------------*/
/*----------------------------------------------------------------------*/

void planner(State6_f QuadCurrent, State6_f ObstCurrent, PosYaw_f *QuadDesired, PosYaw_f
QuadGoal)
{
	// Replace these lines with our method of collision avoidance
	float d; // dist to obst
	float gradd[3]; // grad of dist to obst
	float f_grad[3]; //grad of potential function
	float v[3];
	float vnorm;
	float q[3];
	float p[3];
	//int i; // loop counter

	// Do not change this line
	QuadDesired->yaw = QuadGoal.yaw;
	

	//init gradient
	f_grad[0] = 0;
	f_grad[1] = 0;
	f_grad[2] = 0;

	v[0] = QuadDesired->Pos.x - QuadGoal.Pos.x;
	v[1] = QuadDesired->Pos.y - QuadGoal.Pos.y;
	v[2] = QuadDesired->Pos.z - QuadGoal.Pos.z;

	vnorm = sqrt(pow(v[0],2) + pow(v[1],2) + pow(v[2],2));

	if (vnorm <= batt)
	{
		f_grad[0]+= katt * v[0];
		f_grad[1]+= katt * v[1];
		f_grad[2]+= katt * v[2];
	}else
	{
		f_grad[0]+= katt * (v[0] / vnorm);
		f_grad[1]+= katt * (v[1] / vnorm);
		f_grad[2]+= katt * (v[2] / vnorm);
	}

	q[0] = QuadDesired->Pos.x;
	q[1] = QuadDesired->Pos.y;
	q[2] = QuadDesired->Pos.z;

	p[0] = ObstCurrent.Pos.x;
	p[1] = ObstCurrent.Pos.y;
	p[2] = ObstCurrent.Pos.z;

	v[0] = q[0] - p[0];
	v[1] = q[1] - p[1];
	v[2] = q[2] - p[2];

	vnorm = sqrt(pow(v[0],2) + pow(v[1],2) + pow(v[2],2));

	d = vnorm - (r + s);
	gradd[0] = v[0]/vnorm;
	gradd[1] = v[1]/vnorm;
	gradd[2] = v[2]/vnorm;

	if(d <= brep){
		f_grad[0]+= -krep*((1/d) - (1/brep))*(pow((1/d),2))*gradd[0];
		f_grad[1]+= -krep*((1/d) - (1/brep))*(pow((1/d),2))*gradd[1];
		f_grad[2]+= -krep*((1/d) - (1/brep))*(pow((1/d),2))*gradd[2];
	}else{
		f_grad[0]+= 0;
		f_grad[1]+= 0;
		f_grad[2]+= 0;
	}

	if(sqrt(pow(kdes*f_grad[0],2) + pow(kdes*f_grad[1],2) + pow(kdes*f_grad[2],2))<=brep){
		QuadDesired->Pos.x-= kdes*f_grad[0]; 
		QuadDesired->Pos.y-= kdes*f_grad[1];
		QuadDesired->Pos.z-= kdes*f_grad[2];
	}else{
		QuadDesired->Pos.x-= bdes*(f_grad[0]/sqrt(pow(f_grad[0],2) + pow(f_grad[1],2) + pow(f_grad[2],2))); 
		QuadDesired->Pos.y-= bdes*(f_grad[1]/sqrt(pow(f_grad[0],2) + pow(f_grad[1],2) + pow(f_grad[2],2)));
		QuadDesired->Pos.z-= bdes*(f_grad[2]/sqrt(pow(f_grad[0],2) + pow(f_grad[1],2) + pow(f_grad[2],2)));
	}
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


