#pragma once

#define LEFT_FOOT		0
#define RIGHT_FOOT		1
#define ROBOT_POSE		2
#include "Point.h"
#include "WalkingInformations.h"

class StepData
{
public:
	StepData()
	{
		direction=0;
		pitch=roll=0;
		footPos[0]=0;
		footPos[1]=0;
		onFloor[0]=true;
		onFloor[1]=true;
	}

	StepData operator = (const StepData &p)
	{
		footPos[0]=p.footPos[0];
		footPos[1]=p.footPos[1];
		onFloor[0]=p.onFloor[0];
		onFloor[1]=p.onFloor[1];
		direction=p.direction;
		pitch=p.pitch;
		roll=p.roll;
		return *this;
	}
	Point footPos[2];
	bool onFloor[2];
	double direction, pitch, roll;
};

class Footposition : public StepData
{
public:
	FreeLegPhase kickPhase;
	unsigned int timestamp;
	
	WalkingPhase phase;
	unsigned int singleSupportLen, doubleSupportLen;
	unsigned int frameInPhase;
	float stepDuration; // Additional info since dynamic step duration is possible

	Point speed;

  Point robotPoseAfterStep; // will not influence walking calculation, but used to calculate pose after preview

  //Footposition(Footposition &fp): kickPhase(freeLegNA) { *this=fp; };

	void operator = (const StepData &p)
	{
		this->StepData::operator =(p);
	}

	Footposition() : kickPhase(freeLegNA) {};
};


class ZMP
{
public:
	ZMP()
	{

	}

	unsigned int timestamp;

	ZMP(double x, double y)
	{
		this->x=x;
		this->y=y;
	}
	double x, y;

	void operator =(Point &p)
	{
		this->x=p.x;
		this->y=p.y;
	}
};

