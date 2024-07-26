/*
	Copyright 2011, Oliver Urbann
	All rights reserved.

	This file is part of MoToFlex.

    MoToFlex is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    MoToFlex is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MoToFlex.  If not, see <http://www.gnu.org/licenses/>.

	Contact e-mail: oliver.urbann@tu-dortmund.de
*/

/** 
* @file Modules/MotionControl/DortmundWalkingEngine/PatternGenerator.cpp
* Generator for foot steps
* @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
*/

#include "PatternGenerator.h"
#include <iostream>

//#define LOGGING

#ifndef WALKING_SIMULATOR
#include "Tools/Math/Common.h"

#include "Tools/Debugging/Modify.h"

#include "Tools/Debugging/CSVLogger.h"
#else
#include "../shared/CSVLogger.h"
#endif

using namespace std;


PatternGenerator::PatternGenerator( 
		const WalkingEngineParams	&theWalkingEngineParams,
		const PatternGenRequest		&thePatternGenRequest,
		const RobotModel			&theRobotModel,
		const RobotDimensions		&theRobotDimensions,
		const FallDownState			&theFallDownState,
		const ControllerParams		&theControllerParams,
		const MotionRequest 		&theMotionRequest,
		const WalkingInfo			&theWalkingInfo):
	theWalkingEngineParams(theWalkingEngineParams),
	thePatternGenRequest(thePatternGenRequest),
	theRobotModel(theRobotModel),
	theRobotDimensions(theRobotDimensions),
	theFallDownState(theFallDownState),
	theControllerParams(theControllerParams),
	theMotionRequest(theMotionRequest),
	theWalkingInfo(theWalkingInfo)
{
	reset();
	baseRot=0;
	curPitch=targetPitch=0;
}

void PatternGenerator::setPitch(double pitch)
{
	targetPitch=pitch;
}

double PatternGenerator::getCurrentPitch()
{
	return curPitch;
}

double PatternGenerator::getStepLength(double speed)
{
	return speed*curStepDuration/2;
}

void PatternGenerator::setStepLength()
{

	Point forward, sidestep;
	double r=0;

	forward.x=(currentMovement.speed.x*curStepDuration/2) * cos(robotPose.r);
	forward.y=(currentMovement.speed.x*curStepDuration/2) * sin(robotPose.r);

	sidestep.x=-(currentMovement.speed.y*curStepDuration) * sin(robotPose.r);
	sidestep.y=(currentMovement.speed.y*curStepDuration) * cos(robotPose.r);

	r=currentMovement.speed.r*curStepDuration;

	deltaDirection[firstSingleSupport]=deltaDirection[secondSingleSupport]=r/(2*singleSupportDuration());
	deltaDirection[firstDoubleSupport]=deltaDirection[secondDoubleSupport]=0;

	double z=0.02;

	Point p1(0,0,0,0);
	Point p2(0, 0, z, 0);

	footModifier[unlimitedDoubleSupport][RIGHT_FOOT]=p1;
	footModifier[unlimitedDoubleSupport][LEFT_FOOT]=p1;

	footModifier[unlimitedSingleSupportLeft][RIGHT_FOOT]=p2;
	footModifier[unlimitedSingleSupportLeft][LEFT_FOOT]=p1;
	footModifier[unlimitedSingleSupportRight][RIGHT_FOOT]=p1;
	footModifier[unlimitedSingleSupportRight][LEFT_FOOT]=p2;
  

	footModifier[firstSingleSupport][LEFT_FOOT] =p1;
	footModifier[firstSingleSupport][RIGHT_FOOT]=p2;
	footModifier[secondDoubleSupport][LEFT_FOOT]=p1;

	footModifier[firstDoubleSupport][LEFT_FOOT]=p1;
	footModifier[firstDoubleSupport][RIGHT_FOOT].x=forward.x;
	footModifier[firstDoubleSupport][RIGHT_FOOT].y=forward.y;
	footModifier[firstSingleSupport][RIGHT_FOOT]=footModifier[firstDoubleSupport][RIGHT_FOOT]+p2;


	footModifier[secondSingleSupport][RIGHT_FOOT]=p1;
	footModifier[secondSingleSupport][LEFT_FOOT]=p2;

	footModifier[secondDoubleSupport][LEFT_FOOT].x+=forward.x;
	footModifier[secondDoubleSupport][LEFT_FOOT].y+=forward.y;
	footModifier[secondDoubleSupport][LEFT_FOOT].z=0;
	footModifier[secondDoubleSupport][RIGHT_FOOT]=p1; 
	footModifier[secondSingleSupport][LEFT_FOOT]=footModifier[secondDoubleSupport][LEFT_FOOT]+p2;

	if (currentMovement.speed.y<0)
	{
		Point p1(forward.x, forward.y, 0, 0);
		Point p2(forward.x+sidestep.x, forward.y+sidestep.y, 0, 0);

		footModifier[secondSingleSupport][ROBOT_POSE]=p2;
    footModifier[unlimitedSingleSupportRight][ROBOT_POSE]=p2;
		footModifier[firstSingleSupport][ROBOT_POSE]=p1;
    footModifier[unlimitedSingleSupportLeft][ROBOT_POSE]=p1;


		footModifier[firstDoubleSupport][RIGHT_FOOT].y+=sidestep.y;
		footModifier[firstDoubleSupport][RIGHT_FOOT].x+=sidestep.x;
	}
	else
	{
		Point p1(forward.x, forward.y, 0, 0);
		Point p2(forward.x+sidestep.x, forward.y+sidestep.y, 0, 0);

		footModifier[secondSingleSupport][ROBOT_POSE]= p1;
    footModifier[unlimitedSingleSupportRight][ROBOT_POSE]= p1;
		footModifier[firstSingleSupport][ROBOT_POSE]=p2;
    footModifier[unlimitedSingleSupportLeft][ROBOT_POSE]=p2;

		footModifier[secondDoubleSupport][LEFT_FOOT].y+=sidestep.y;
		footModifier[secondDoubleSupport][LEFT_FOOT].x+=sidestep.x;
	}

	if (r<0)
	{
		footModifier[secondDoubleSupport][RIGHT_FOOT].r=r/2;
		footModifier[secondDoubleSupport][LEFT_FOOT].r=r/2;
		footModifier[firstDoubleSupport][RIGHT_FOOT].r=r;
		footModifier[secondSingleSupport][RIGHT_FOOT].r=r/2;
		footModifier[firstSingleSupport][ROBOT_POSE].r=r/2;
		footModifier[secondSingleSupport][ROBOT_POSE].r=r/2;
	}
	else
	{
		footModifier[firstDoubleSupport][RIGHT_FOOT].r=r/2;
		footModifier[firstDoubleSupport][LEFT_FOOT].r=r/2;
		footModifier[secondDoubleSupport][LEFT_FOOT].r=r;
		footModifier[firstSingleSupport][LEFT_FOOT].r=r/2;
		footModifier[secondSingleSupport][ROBOT_POSE].r=r/2;
		footModifier[firstSingleSupport][ROBOT_POSE].r=r/2;
	}



	//Charlie Chapling walk
	for (int i=0; i<4; i++)
	{
		footModifier[i][LEFT_FOOT].r+=baseRot;
		footModifier[i][RIGHT_FOOT].r-=baseRot;
	}	
}


PatternGenerator::~PatternGenerator(void)
{
}


inline unsigned int PatternGenerator::getPhaseLength(double ratio, double stepDur)
{
	return (int)floor(((ratio*stepDur*(1/theControllerParams.dt))/2)+0.5);
}

void PatternGenerator::initWalkingPhase()
{
	if (currentWalkingPhase!=unlimitedDoubleSupport &&
		currentWalkingPhase!=unlimitedSingleSupportLeft &&
		currentWalkingPhase!=unlimitedSingleSupportRight)
		return;

	if (currentWalkingPhase==unlimitedDoubleSupport)
	{
		currentWalkingPhase=secondSingleSupport;

		if (currentMovement.speed.y<0 || currentMovement.speed.r<0)
			currentWalkingPhase=firstSingleSupport;
	}
	else
	{
		if (currentWalkingPhase==unlimitedSingleSupportLeft)
		{
			//if (currentMovement.speed.r>0)
			//	currentWalkingPhase=unlimitedDoubleSupport;
			//else
				currentWalkingPhase=firstSingleSupport;
				currentMovement.speed.r=0;
				
		}
		else
		{
	/*		if (currentMovement.speed.r<0)
				currentWalkingPhase=unlimitedDoubleSupport;
			else*/
				currentWalkingPhase=secondSingleSupport;
				currentMovement.speed.r=0;
		}
	}
	stateCounter=singleSupportDuration();
}

int PatternGenerator::changeState(State newState, MovementInformation &moveInf)
{
	Footposition newStep;
	MovementInformation stopInf;
	switch(currentState)
	{
	case standby:
		if (newState==ready || newState==walking)
		{
			stateCounter=theWalkingEngineParams.crouchingDownPhaseLength;
			currentState=goingToReady;
			currentMovement=moveInf;
		}
		else return TRANSITION_NOT_POSSIBLE;
		break;
	case ready:
		switch(newState)
		{
		case standby:
			stateCounter=theWalkingEngineParams.crouchingDownPhaseLength;
			currentState=goingToStandby;
			running=false;
			break;
		case walking:
			if (curPitch<=theWalkingEngineParams.maxWalkPitch)
			{
				currentState=walking;
				currentMovement=moveInf;
				calcWalkParams();
				initWalkingPhase();
				setStepLength();
			}
			break;
		case ready:
			applyStandType();
			break;

		default:
			return TRANSITION_NOT_POSSIBLE;
		}
		break;
	case walking:
		switch (newState)
		{
			// This command stops the robot, but is only requests the stopping phase
			// (by settings newState to stopping). The stopping phase will be started by
			// stopRobot (which sets currentState to stopping)
		case ready:
			this->newState=stopping;
			this->newMovement=stopInf;
			break;

			// This command changes the speed, if robot is walking
		case walking:
			if (moveInf!=currentMovement)
			{
				initWalkingPhase();
				this->newState=walking;
				newMovement=moveInf;
			}
			break;

			// Nothing else possible during walking
		default:
			return TRANSITION_NOT_POSSIBLE;
		}
		break;
	default:
		break;
	}

	return OK;
}

#define MAX_DURATION	4
#define MIN_DURATION	1

void PatternGenerator::applyStandType()
{
	switch(theMotionRequest.standType)
	{
	case doubleSupport:
		stateCounter=1;
		if (currentWalkingPhase==unlimitedSingleSupportLeft)
		{
			currentWalkingPhase=firstSingleSupport;
			currentState=walking;
			stateCounter=singleSupportDuration();
		}
		else if(currentWalkingPhase==unlimitedSingleSupportRight)
		{
			currentWalkingPhase=secondSingleSupport;
			currentState=walking;
			stateCounter=singleSupportDuration();
		}
		else
			currentWalkingPhase=unlimitedDoubleSupport;
		break;
	case leftSingleSupport:
		if (currentWalkingPhase==unlimitedSingleSupportRight)
		{
			currentWalkingPhase=secondSingleSupport;
			currentState=walking;
			stateCounter=singleSupportDuration();
		}			
		else
			currentWalkingPhase=unlimitedSingleSupportLeft;
		break;
	case rightSingleSupport:
		if (currentWalkingPhase==unlimitedSingleSupportLeft)
		{
			currentWalkingPhase=firstSingleSupport;
			currentState=walking;
			stateCounter=singleSupportDuration();
		}
		else
			currentWalkingPhase=unlimitedSingleSupportRight;
		break;
	default:
		break;
	}
}

void PatternGenerator::calcWalkParams()
{
	newState=NA;

	//float translation, rotation;
	previewDelta=0;
	//float x=(float)currentMovement.speed.x;
	//float y=(float)abs(currentMovement.speed.y)+theWalkingEngineParams.footYDistance*2;

	if (dynamicDuration){
		if (fabs(currentMovement.speed.y)>0.01 ||
			fabs(currentMovement.speed.r)>0.2 ||
			fabs(currentMovement.speed.x)>0.05)
			curStepDuration=1;
		else
			curStepDuration=2.5;

	}
}

void PatternGenerator::updateRobotPose()
{
	robotPose+=footModifier[currentWalkingPhase][ROBOT_POSE];
	direction=robotPose.r;
	distanceLeft-=footModifier[currentWalkingPhase][ROBOT_POSE];
}

bool PatternGenerator::isWalking()
{
	return !(currentState==ready || currentState==standby);
}


void PatternGenerator::resetDelays()
{
	for (int i=0; i<3; i++)
		speedApplyDelay[i]=0;
}

void PatternGenerator::decreaseDelays()
{
 	for (int i=0; i<3; i++)
		if (speedApplyDelay[i]>0)
			speedApplyDelay[i]--;
}

void PatternGenerator::applySpeed(bool x, bool y, bool r)
{
	if (x && speedApplyDelay[0]==0)
	{
		if (currentMovement.speed.x!=newMovement.speed.x)
			speedApplyDelay[0]=theWalkingEngineParams.speedApplyDelay;

		currentMovement.speed.x=newMovement.speed.x;
	}
	if (y && speedApplyDelay[1]==0)
	{
		if (currentMovement.speed.y!=newMovement.speed.y)
		{
			speedApplyDelay[1]=theWalkingEngineParams.speedApplyDelay;
		}
		currentMovement.speed.y=newMovement.speed.y;
	}
	if (r && speedApplyDelay[2]==0)
	{
		if (currentMovement.speed.r!=newMovement.speed.r)
		{
			speedApplyDelay[2]=theWalkingEngineParams.speedApplyDelay;

		}
		if (currentMovement.speed.r*newMovement.speed.r<0)
			currentMovement.speed.r=0;
		else
			currentMovement.speed.r=newMovement.speed.r;
	}
}

void PatternGenerator::updateCounter()
{
	MovementInformation standInf;
	bool stopPossible=false;
	int previewLength;
	if (currentMovement.speed.x==0 &&
		currentMovement.speed.y==0 &&
		currentMovement.speed.r==0)
		stopPossible=true;

	if (stateCounter==0)
	{
		switch(currentState)
		{
		case ready:
		case standby:								
			stateCounter=1;
			break;

		case goingToReady:
			currentState=ready;
			running=true;
			currentMovement=standInf;
			calcWalkParams();
			setStepLength();
			currentWalkingPhase=unlimitedDoubleSupport;
			previewLength=theControllerParams.N+1;
			if (previewLength<(int)(doubleSupportDuration()+singleSupportDuration()+1))
				previewLength=doubleSupportDuration()+singleSupportDuration()+1;
			for (int i=0; i<previewLength; i++)
				addStep();
			break;

		case goingToStandby:
			stateCounter=1;
			currentState=standby;
			reset();
			break;

		case stopping:	
			if (newState==ready)
			{
				currentState=ready;
				stateCounter=1;
				newState=NA;
			}
		case walking:
			switch (currentWalkingPhase)
			{
			case firstSingleSupport:
				// If the kick is not finished we cannot start
				// a double support phase
				if (theWalkingInfo.kickPhase==ending)
					return;

				currentWalkingPhase=firstDoubleSupport;
				stateCounter=doubleSupportDuration();
				if (newState==stopping && stopPossible && theMotionRequest.standType!=rightSingleSupport)
				{
					calcWalkParams();
					stopRobot(robotPose);
				}
				break;
			case firstDoubleSupport:
				currentWalkingPhase=secondSingleSupport;
				if (newState==walking || newState==stopping)
				{
					applySpeed(true, !(newMovement.speed.y<0), (currentMovement.speed.r>=0));
					calcWalkParams();
				}
				updateRobotPose();
				setStepLength();
				if (theMotionRequest.standType==rightSingleSupport)
						stopRobot(robotPose);
				else
				{
					stateCounter=singleSupportDuration();
				}
				break;
			case secondSingleSupport:
				// If the kick is not finished we cannot start
				// a double support phase
				if (theWalkingInfo.kickPhase==ending)
					return;

				currentWalkingPhase=secondDoubleSupport;
				stateCounter=doubleSupportDuration();
				if (newState==stopping && stopPossible && theMotionRequest.standType!=leftSingleSupport)
				{
					calcWalkParams();
					stopRobot(robotPose);	
				}
				break;
			case secondDoubleSupport:
				currentWalkingPhase=firstSingleSupport;
				if (newState==walking || newState==stopping)
				{
					applySpeed(true, !(newMovement.speed.y>0), (currentMovement.speed.r<=0));
					calcWalkParams();
				}
				updateRobotPose();
				setStepLength();
				if (theMotionRequest.standType==leftSingleSupport)
						stopRobot(robotPose);
				else
				{
					stateCounter=singleSupportDuration();
				}
				break;
			default:
				if (currentState==walking)
				{
					cout << "Illegal support phase during walking" << endl;
					OUTPUT(idText, text, "Illegal support phase during walking");
				}
			}
			break;
			default:
			break;
		}
	}
	stateCounter--;
}

void PatternGenerator::reset()
{
	Point p(0,0,0,0);
	robotPose=p;
	newState=NA;
	currentState=standby;
	direction=0;
	previewDelta=0;
	running=false;
	resetDelays();
	currentTimeStamp=0;
}

double PatternGenerator::getPitchSpeed()
{
	if (targetPitch-curPitch>0)
		return theWalkingEngineParams.pitchSpeed;
	else
		return -theWalkingEngineParams.pitchSpeed;
}

void PatternGenerator::handlePitch()
{
	// interpolate and set the pitch

	if (curPitch>theWalkingEngineParams.maxWalkPitch && currentState!=ready && targetPitch-curPitch>=0)
		return;

	if (std::abs(targetPitch-curPitch)>theWalkingEngineParams.pitchSpeed*theControllerParams.dt)
	{
		curPitch+=getPitchSpeed()*theControllerParams.dt;
	}
	else
		curPitch=targetPitch;
}


StepData PatternGenerator::getNextStep() 
{
	Footposition step;
#ifndef WALKING_SIMULATOR
	double startZ=0.3042051-0.085, factor=0;
#else
	double startZ=0.331, factor=0;
#endif

	switch(currentState)
	{
	case ready:
		//currentWalkingPhase=unlimitedDoubleSupport;
		addStep();
		handlePitch();
		updateCounter();
		break;

	case standby:
		//startZ=sqrt(pow(theWalkingEngineParams.maxLegLength, 2)-pow(theWalkingEngineParams.xOffset, 2)-pow(theWalkingEngineParams.footYDistance, 2));
		step.direction=0;
		step.footPos[LEFT_FOOT]=0;
		step.footPos[RIGHT_FOOT]=0;
		step.footPos[RIGHT_FOOT].z=step.footPos[LEFT_FOOT].z=-startZ;
		step.footPos[RIGHT_FOOT].y=-theWalkingEngineParams.footYDistance;
		step.footPos[LEFT_FOOT].y=theWalkingEngineParams.footYDistance;
		step.footPos[LEFT_FOOT].r=baseRot;
		step.footPos[RIGHT_FOOT].r=-baseRot;
    step.robotPoseAfterStep = robotPose; // does not change...
		updateCounter();
		break;

	case goingToReady:
		if (theWalkingEngineParams.crouchingDownPhaseLength>0)
		{
			factor=1-(double(stateCounter)/theWalkingEngineParams.crouchingDownPhaseLength);
			//startZ=sqrt(pow(theWalkingEngineParams.maxLegLength, 2)-pow(theWalkingEngineParams.xOffset, 2)-pow(theWalkingEngineParams.footYDistance, 2));
			step.direction=0;
			step.footPos[LEFT_FOOT]=0;
			step.footPos[RIGHT_FOOT]=0;
			step.footPos[RIGHT_FOOT].x=CoM.x*factor;
			step.footPos[LEFT_FOOT].x=CoM.x*factor;
			step.footPos[RIGHT_FOOT].z=step.footPos[LEFT_FOOT].z=-startZ+(startZ-theControllerParams.z_h+CoM.z)*factor;
			step.footPos[RIGHT_FOOT].y=-theWalkingEngineParams.footYDistance;
			step.footPos[LEFT_FOOT].y=theWalkingEngineParams.footYDistance;
			step.footPos[LEFT_FOOT].r=baseRot;
			step.footPos[RIGHT_FOOT].r=-baseRot;
      step.robotPoseAfterStep = robotPose; // does not change...
		}
		updateCounter();
		break;

	case goingToStandby:
		if (theWalkingEngineParams.crouchingDownPhaseLength>0)
		{
			factor=(double(stateCounter)/theWalkingEngineParams.crouchingDownPhaseLength);
			//startZ=sqrt(pow(theWalkingEngineParams.maxLegLength, 2)-pow(theWalkingEngineParams.xOffset, 2)-pow(theWalkingEngineParams.footYDistance, 2));
			step.direction=0;
			step.footPos[LEFT_FOOT]=0;
			step.footPos[RIGHT_FOOT]=0;
			step.footPos[RIGHT_FOOT].x=CoM.x*factor;
			step.footPos[LEFT_FOOT].x=CoM.x*factor;
			step.footPos[RIGHT_FOOT].z=step.footPos[LEFT_FOOT].z=-startZ+(startZ-theControllerParams.z_h+CoM.z)*factor;
			step.footPos[RIGHT_FOOT].y=-theWalkingEngineParams.footYDistance;
			step.footPos[LEFT_FOOT].y=theWalkingEngineParams.footYDistance;
			step.footPos[LEFT_FOOT].r=baseRot;
			step.footPos[RIGHT_FOOT].r=-baseRot;
      step.robotPoseAfterStep = robotPose; // does not change...
		}
		updateCounter();
		break;

	case walking:

		// meaning of skip: skip only every second frame when previewDelta<0

		if (previewDelta==0 || !skip)
		{
			addStep();
			updateCounter();
			handlePitch();
			skip=!skip;
		}

		while (previewDelta>0) // Add more steps to get a higher preview
		{
			addStep();
			previewDelta--;
			updateCounter();
			handlePitch();
		}

		if (skip && previewDelta<0) // Skip this frame, cause we need a lower preview
		{
			previewDelta++;
			skip=!skip;
		}
		break;

	case stopping:
		addStep();
		handlePitch();
		updateCounter();
		break;

	default:
		break;
	}

	return step;
}

unsigned int PatternGenerator::singleSupportDuration()
{
	return getPhaseLength(1-theWalkingEngineParams.doubleSupportRatio, curStepDuration);
}

unsigned int PatternGenerator::doubleSupportDuration()
{
	return getPhaseLength(theWalkingEngineParams.doubleSupportRatio, curStepDuration);
}

void PatternGenerator::calcFootRotation(int footNum)
{

}

State& PatternGenerator::getCurrentState()
{
	return currentState;
}

void PatternGenerator::addStep()
{
	Footposition newStep;
	newStep.footPos[LEFT_FOOT]=0;
	newStep.footPos[RIGHT_FOOT]=0;

	newStep.footPos[LEFT_FOOT].y=theWalkingEngineParams.footYDistance;
	newStep.footPos[RIGHT_FOOT].y=-theWalkingEngineParams.footYDistance;

	newStep.footPos[LEFT_FOOT].rotate2D(robotPose.r+footModifier[currentWalkingPhase][LEFT_FOOT].r);
	newStep.footPos[RIGHT_FOOT].rotate2D(robotPose.r+footModifier[currentWalkingPhase][RIGHT_FOOT].r);

	newStep.footPos[LEFT_FOOT]+=robotPose+footModifier[currentWalkingPhase][LEFT_FOOT];
	newStep.footPos[RIGHT_FOOT]+=robotPose+footModifier[currentWalkingPhase][RIGHT_FOOT];

	newStep.onFloor[LEFT_FOOT]=(footModifier[currentWalkingPhase][LEFT_FOOT].z==0);
	newStep.onFloor[RIGHT_FOOT]=(footModifier[currentWalkingPhase][RIGHT_FOOT].z==0);

	newStep.direction=direction;

	newStep.phase=currentWalkingPhase;
	newStep.singleSupportLen=singleSupportDuration();
	newStep.doubleSupportLen=doubleSupportDuration();
	newStep.stepDuration=curStepDuration;
	newStep.timestamp=currentTimeStamp;
	currentTimeStamp++;

	newStep.frameInPhase=0;
	if (currentWalkingPhase==firstSingleSupport || currentWalkingPhase==secondSingleSupport)
		newStep.frameInPhase=newStep.singleSupportLen-stateCounter;
	if (currentWalkingPhase==firstDoubleSupport || currentWalkingPhase==secondDoubleSupport)
		newStep.frameInPhase=newStep.doubleSupportLen-stateCounter;

	newStep.speed=currentMovement.speed;


  // set "prediction" of robot pose after this step
  newStep.robotPoseAfterStep = robotPose + footModifier[currentWalkingPhase][ROBOT_POSE];
	if (currentWalkingPhase==firstDoubleSupport) // i.e. the 2nd foot is already on the ground
		newStep.robotPoseAfterStep += footModifier[secondSingleSupport][ROBOT_POSE];
	if (currentWalkingPhase==secondDoubleSupport) // i.e. the 1st foot is already on the ground
		newStep.robotPoseAfterStep += footModifier[firstSingleSupport][ROBOT_POSE];
  // TODO: Eventually predict the delays from applySpeed().


	// Add to buffer for ZMP/IP-Controller (sent via FootSteps)

	if (steps!=NULL)
	{
		steps->addStep(newStep);
	}

	bool ok=false;
	if (newStep.onFloor[LEFT_FOOT])
	{
		if (deltaDirection[currentWalkingPhase]<0 || newStep.footPos[LEFT_FOOT].r>=direction)
			ok=true;
	}

	if (newStep.onFloor[RIGHT_FOOT])
	{
		if (deltaDirection[currentWalkingPhase]>0 || newStep.footPos[RIGHT_FOOT].r<=direction)
			ok=true;
	}

	if (ok)
		direction+=deltaDirection[currentWalkingPhase];

	lastStep=newStep;

  
  LOG("PatternGenerator_addStep", "step.footPos[0].x", newStep.footPos[0].x);
  LOG("PatternGenerator_addStep", "step.footPos[0].y", newStep.footPos[0].y);
  LOG("PatternGenerator_addStep", "step.footPos[0].z", newStep.footPos[0].z);
  LOG("PatternGenerator_addStep", "step.footPos[0].r", newStep.footPos[0].r);
  LOG("PatternGenerator_addStep", "step.footPos[1].x", newStep.footPos[1].x);
  LOG("PatternGenerator_addStep", "step.footPos[1].y", newStep.footPos[1].y);
  LOG("PatternGenerator_addStep", "step.footPos[1].z", newStep.footPos[1].z);
  LOG("PatternGenerator_addStep", "step.footPos[1].r", newStep.footPos[1].r);
  LOG("PatternGenerator_addStep", "step.direction", newStep.direction);
}

void PatternGenerator::stopRobot(Point &pos)
{
	Footposition newStep;
	currentState=stopping;
	newState=ready;

	applyStandType();
	
	// That's not exact. Maybe there are more steps to do ...
	stateCounter=theWalkingEngineParams.stoppingPhaseLength;
	resetDelays();
}

void PatternGenerator::updateCoM(Point CoM)
{
	this->CoM=CoM;
}

void PatternGenerator::updateFootSteps(
  FootSteps & steps
	) 
{

  steps.reset();

	if (theFallDownState.state!=FallDownState::upright) reset();

	this->steps=&steps;
	//FootSteps steps;
	//stepBuffer=steps.steps;
	//stepBufferSize=MAX_STEPS;
	//stepBufferPos=0;

	if (theWalkingEngineParams.stepDuration!=0)
	{
		dynamicDuration=false;
		curStepDuration=theWalkingEngineParams.stepDuration;
	}
	else
		dynamicDuration=true;

	Point CoM(theRobotModel.centerOfMass.x/1000, theRobotModel.centerOfMass.y/1000, (theRobotModel.centerOfMass.z)/1000, 0);
	updateCoM(CoM);
	setPitch(thePatternGenRequest.pitch);

	if (thePatternGenRequest.newState!=PatternGenRequest::NA)
	{
		MovementInformation moveInf;
		moveInf.speed.x=thePatternGenRequest.speed.translation.x;
		moveInf.speed.y=thePatternGenRequest.speed.translation.y;
		moveInf.speed.r=thePatternGenRequest.speed.rotation;

    //moveInf.destination.x = thePatternGenRequest.target.translation.x;
    //moveInf.destination.y = thePatternGenRequest.target.translation.y;
    //moveInf.destination.r = thePatternGenRequest.target.rotation;

    // take new destination instantly
    currentMovement.destination = moveInf.destination;


		State newState=NA;

		// Translate public states to internal
		switch (thePatternGenRequest.newState)
		{
		case standby:
			newState=standby;
			break;

		case ready:
			newState=ready;
			break;

		case walking:
			newState=walking;
			break;
    default:
      OUTPUT(idText, text,"PatternGenerator: switch (thePatternGenRequest.newState) hit default. Should not happen...");
		}

		changeState(newState, moveInf);
	}

	steps.pitch=curPitch;
	steps.suggestedStep=getNextStep();
	steps.running=running;
	this->steps=NULL;

	decreaseDelays();
}

