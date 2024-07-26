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

#include "CSConverter.h"

//#define LOGGING
#include "Tools/Debugging/CSVLogger.h"

CSConverter::CSConverter(	
		const Footpositions			&theFootpositions,
		const TargetCoM				&theTargetCoM,
		const WalkingEngineParams	&theWalkingEngineParams,
		const ControllerParams		&theControllerParams,
		const RobotModel			&theRobotModel,
		const FallDownState			&theFallDownState,
		const SensorData			&theSensorData,
		const BodyTilt				&theBodyTilt,
		const FreeLegPhaseParams	&theFreeLegPhaseParams):
	theFootpositions(theFootpositions),
	theTargetCoM(theTargetCoM),
	theWalkingEngineParams(theWalkingEngineParams),
	theControllerParams(theControllerParams),
	theRobotModel(theRobotModel),
	theFallDownState(theFallDownState),
	theSensorData(theSensorData),
	theBodyTilt(theBodyTilt),
	theFreeLegPhaseParams(theFreeLegPhaseParams)
{
	reset();
}


CSConverter::~CSConverter(void)
{
	freeMem();
}

void CSConverter::Shrink()
{
	while(currentFootpos!=footPositions.begin())
	{
		Footposition *footPos=footPositions.front();
		footPositions.pop_front();
		delete footPos;
	}
}


void CSConverter::freeMem()
{
	int size;

	size=footPositions.size();
	for (int i=0; i<size; i++)
	{
		Footposition *footPos=footPositions.front();
		footPositions.pop_front();
		delete footPos;
	}
}

void CSConverter::reset()
{
	freeMem();
	lastFootPositionsValid=false;
	fallingDown=false;
	isRunning=false;
}


void CSConverter::addFootposition(const Footposition &fp)
{
	Footposition *newpos=new Footposition;
	*newpos=fp;

	footPositions.push_back(newpos);

	if (footPositions.front()==footPositions.back())
	{
		currentFootpos=footPositions.begin();
	}

}


void CSConverter::toRobotCoords(StepData *requiredOffset, Point newCoMTarget, Footposition curPos, Point CoM)
{
	StepData tempStep;
	Point temp;

	// Calculate the distance between current CoM and feet
	tempStep.footPos[RIGHT_FOOT]=curPos.footPos[RIGHT_FOOT]-newCoMTarget;
	tempStep.footPos[LEFT_FOOT]=curPos.footPos[LEFT_FOOT]-newCoMTarget;

	//tempStep.footPos[RIGHT_FOOT].y=curPos.footPos[RIGHT_FOOT].y-newCoMTarget.y;
	//tempStep.footPos[LEFT_FOOT].y=curPos.footPos[LEFT_FOOT].y-newCoMTarget.y;

	//tempStep.footPos[RIGHT_FOOT].z=curPos.footPos[RIGHT_FOOT].z;
	//tempStep.footPos[LEFT_FOOT].z=curPos.footPos[LEFT_FOOT].z;

	// Rotate around the CoM (z axis)
	*requiredOffset=tempStep;
	requiredOffset->footPos[RIGHT_FOOT].x=tempStep.footPos[RIGHT_FOOT].x*cos(curPos.direction)+tempStep.footPos[RIGHT_FOOT].y*sin(curPos.direction);
	requiredOffset->footPos[LEFT_FOOT].x=tempStep.footPos[LEFT_FOOT].x*cos(curPos.direction)+tempStep.footPos[LEFT_FOOT].y*sin(curPos.direction);

	requiredOffset->footPos[RIGHT_FOOT].y=tempStep.footPos[RIGHT_FOOT].y*cos(curPos.direction)-tempStep.footPos[RIGHT_FOOT].x*sin(curPos.direction);
	requiredOffset->footPos[LEFT_FOOT].y=tempStep.footPos[LEFT_FOOT].y*cos(curPos.direction)-tempStep.footPos[LEFT_FOOT].x*sin(curPos.direction);


	// Rotate the foot positions relative to CoM around the y axis (pitch)
	tempStep=*requiredOffset;
	tempStep.footPos[RIGHT_FOOT].x=requiredOffset->footPos[RIGHT_FOOT].x*cos(-curPos.pitch)+requiredOffset->footPos[RIGHT_FOOT].z*sin(-curPos.pitch);
	tempStep.footPos[RIGHT_FOOT].z=-requiredOffset->footPos[RIGHT_FOOT].x*sin(-curPos.pitch)+requiredOffset->footPos[RIGHT_FOOT].z*cos(-curPos.pitch);

	tempStep.footPos[LEFT_FOOT].x=requiredOffset->footPos[LEFT_FOOT].x*cos(-curPos.pitch)+requiredOffset->footPos[LEFT_FOOT].z*sin(-curPos.pitch);
	tempStep.footPos[LEFT_FOOT].z=-requiredOffset->footPos[LEFT_FOOT].x*sin(-curPos.pitch)+requiredOffset->footPos[LEFT_FOOT].z*cos(-curPos.pitch);
	*requiredOffset=tempStep;

	// Rotate the foot positions relative to CoM around the x axis (roll)
	tempStep=*requiredOffset;
	tempStep.footPos[RIGHT_FOOT].y=requiredOffset->footPos[RIGHT_FOOT].y*cos(-curPos.roll)-requiredOffset->footPos[RIGHT_FOOT].z*sin(-curPos.roll);
	tempStep.footPos[RIGHT_FOOT].z=requiredOffset->footPos[RIGHT_FOOT].y*sin(-curPos.roll)+requiredOffset->footPos[RIGHT_FOOT].z*cos(-curPos.roll);

	tempStep.footPos[LEFT_FOOT].y=requiredOffset->footPos[LEFT_FOOT].y*cos(-curPos.roll)-requiredOffset->footPos[LEFT_FOOT].z*sin(-curPos.roll);
	tempStep.footPos[LEFT_FOOT].z=requiredOffset->footPos[LEFT_FOOT].y*sin(-curPos.roll)+requiredOffset->footPos[LEFT_FOOT].z*cos(-curPos.roll);
	*requiredOffset=tempStep;

	// Add the position of the CoM to get the feet positions in robot coordinate system
	requiredOffset->footPos[RIGHT_FOOT]+=CoM;
	requiredOffset->footPos[LEFT_FOOT]+=CoM;

	// Set the foot pitch
	requiredOffset->footPos[LEFT_FOOT].ry=-curPos.pitch;
	requiredOffset->footPos[RIGHT_FOOT].ry=-curPos.pitch;
	requiredOffset->footPos[LEFT_FOOT].ry+=curPos.footPos[LEFT_FOOT].ry;
	requiredOffset->footPos[RIGHT_FOOT].ry+=curPos.footPos[RIGHT_FOOT].ry;

	// Set the foot roll
	requiredOffset->footPos[LEFT_FOOT].rx=-curPos.roll;
	requiredOffset->footPos[RIGHT_FOOT].rx=-curPos.roll;

	// Set the foot Orientation
	requiredOffset->footPos[RIGHT_FOOT].r=curPos.footPos[RIGHT_FOOT].r-curPos.direction;
	requiredOffset->footPos[LEFT_FOOT].r=curPos.footPos[LEFT_FOOT].r-curPos.direction;

	requiredOffset->onFloor[LEFT_FOOT]=curPos.onFloor[LEFT_FOOT];
	requiredOffset->onFloor[RIGHT_FOOT]=curPos.onFloor[RIGHT_FOOT];

	Point gCoM=CoM;
	gCoM.rotate2D(curPos.direction);
	robotPosition=newCoMTarget-gCoM;
	robotPosition.r=curPos.direction;

  // Calculate new position based on the definition of our robot coordinate frame.
  // This will be used for odometry and for the offsetToRobotPoseAfterPreview.
  // The CameraMatrix calculation must be based on this same 
  //   robot coordinate frame to provide consistent information!

  // ... for ground-projected robot coordinate frame
  //Point newPos = robotPosition;
  
  // ... for position between feet with rotation of body
  Point newPos = (curPos.footPos[RIGHT_FOOT]+curPos.footPos[LEFT_FOOT])*0.5;
  newPos.r = robotPosition.r;

  // ... for pose between feet
  //Point newPos = (curPos.footPos[RIGHT_FOOT]+curPos.footPos[LEFT_FOOT])*0.5;


  offsetToRobotPoseAfterPreview = footPositions.back()->robotPoseAfterStep - newPos;

	offsetToRobotPoseAfterPreview.rotate2D(-newPos.r);

#pragma region Odometry
  // odometry for pose between feet
	odometry=newPos-lastPos;
	odometry.rotate2D(-lastPos.r);
	lastPos=newPos;
#pragma endregion
}

void CSConverter::updateKinematicRequest(KinematicRequest &kinematicRequest)
{
	StepData currentStep;
	Point targetCoM;

	
#pragma region init
	for (int i = 2; i < JointData::numOfJoints; i++) {
		kinematicRequest.offsets.angles[i] = 0;
	}
	for (int i = 0; i < 6; i++) {
		kinematicRequest.body[i] = JointData::ignore;
	}
#pragma endregion

	for (int i=0; i<theFootpositions.numOfFP; i++)
		addFootposition(theFootpositions.getFP(i));

	if (!isRunning && theFootpositions.running)
	{
		isRunning=true;
	}

	if (isRunning && !theFootpositions.running)
	{
		reset();
		isRunning=false;
	}

	if (isRunning)
	{
		ASSERT(footPositions.size()>0);
		(*currentFootpos)->pitch=theBodyTilt.y;
		(*currentFootpos)->roll=theBodyTilt.x;
		Point actCoM(theRobotModel.centerOfMass.x/1000, theRobotModel.centerOfMass.y/1000, (theRobotModel.centerOfMass.z)/1000 , 0);
		targetCoM=theTargetCoM;
		toRobotCoords(&currentStep, targetCoM, *(*currentFootpos), actCoM);

		
#pragma region LOGGING
	
		LOG("WalkingEngine", "WCS left Foot x", (*currentFootpos)->footPos[LEFT_FOOT].x);
		LOG("WalkingEngine", "WCS left Foot y", (*currentFootpos)->footPos[LEFT_FOOT].y);
		LOG("WalkingEngine", "WCS left Foot z", (*currentFootpos)->footPos[LEFT_FOOT].z);
		LOG("WalkingEngine", "WCS left Foot r", (*currentFootpos)->footPos[LEFT_FOOT].r);
		LOG("WalkingEngine", "WCS right Foot x", (*currentFootpos)->footPos[RIGHT_FOOT].x);
		LOG("WalkingEngine", "WCS right Foot y", (*currentFootpos)->footPos[RIGHT_FOOT].y);
		LOG("WalkingEngine", "WCS right Foot z", (*currentFootpos)->footPos[RIGHT_FOOT].z);
		LOG("WalkingEngine", "WCS right Foot r", (*currentFootpos)->footPos[RIGHT_FOOT].r);

		LOG("WalkingEngine", "RCS left Foot x", currentStep.footPos[LEFT_FOOT].x);
		LOG("WalkingEngine", "RCS left Foot y", currentStep.footPos[LEFT_FOOT].y);
		LOG("WalkingEngine", "RCS left Foot z", currentStep.footPos[LEFT_FOOT].z);
		LOG("WalkingEngine", "RCS left Foot r", currentStep.footPos[LEFT_FOOT].r);
		LOG("WalkingEngine", "RCS right Foot x", currentStep.footPos[RIGHT_FOOT].x);
		LOG("WalkingEngine", "RCS right Foot y", currentStep.footPos[RIGHT_FOOT].y);
		LOG("WalkingEngine", "RCS right Foot z", currentStep.footPos[RIGHT_FOOT].z);
		LOG("WalkingEngine", "RCS right Foot r", currentStep.footPos[RIGHT_FOOT].r);

#pragma endregion


		currentFootpos++;
		Shrink();
	}
	else
	{
		currentStep=theFootpositions.suggestedStep;
		targetCoM=0;
	}


#pragma region SpeedAccCalculation
	Point speed=(targetCoM-lastTargetCoM)/theControllerParams.dt;
	lastTargetCoM=targetCoM;

	acc=speed-lastSpeed/theControllerParams.dt;
	lastSpeed=speed;
#pragma endregion
	
#pragma region isLeavingPossible


	isLeavingPossible=false;
	if (isRunning)
	{
		// check if the robot stands on both feets (CoM in the middle),
		// and does not move (speed of CoM < 0.01)
		if (fabs(currentStep.footPos[LEFT_FOOT].x)<theWalkingEngineParams.stopPosThresholdX &&
			fabs(currentStep.footPos[RIGHT_FOOT].x)<theWalkingEngineParams.stopPosThresholdX &&
			fabs(currentStep.footPos[LEFT_FOOT].y+currentStep.footPos[RIGHT_FOOT].y)<theWalkingEngineParams.stopPosThresholdY &&
			fabs(speed.x)<theWalkingEngineParams.stopSpeedThresholdX &&
			fabs(speed.y)<theWalkingEngineParams.stopSpeedThresholdY)
			isLeavingPossible=true;
	}
	else
	{
		isLeavingPossible=true;
	}
#pragma endregion
	
#pragma region KinematicRequestFill

	if (currentStep.onFloor[LEFT_FOOT] && currentStep.onFloor[RIGHT_FOOT])
		kinematicRequest.kinematicType = KinematicRequest::feet;
	if (currentStep.onFloor[LEFT_FOOT] && !currentStep.onFloor[RIGHT_FOOT])
	{
		kinematicRequest.kinematicType = KinematicRequest::bodyAndLeftFoot;
	}
	if (!currentStep.onFloor[LEFT_FOOT] && currentStep.onFloor[RIGHT_FOOT])
	{
		kinematicRequest.kinematicType = KinematicRequest::bodyAndRightFoot;
	}

	// position left foot
	kinematicRequest.leftFoot[0]=(float)(currentStep.footPos[LEFT_FOOT].x*1000-(theWalkingEngineParams.xOffset*1000));
	kinematicRequest.leftFoot[1]=(float)(currentStep.footPos[LEFT_FOOT].y*1000);
	kinematicRequest.leftFoot[2]=(float)(currentStep.footPos[LEFT_FOOT].z*1000);

	// position right foot
	kinematicRequest.rightFoot[0]=(float)(currentStep.footPos[RIGHT_FOOT].x*1000-(theWalkingEngineParams.xOffset*1000));
	kinematicRequest.rightFoot[1]=(float)(currentStep.footPos[RIGHT_FOOT].y*1000);
	kinematicRequest.rightFoot[2]=(float)(currentStep.footPos[RIGHT_FOOT].z*1000);

	kinematicRequest.leftFoot[3] = (float)(currentStep.footPos[LEFT_FOOT].rx);
	kinematicRequest.leftFoot[4] = (float)(currentStep.footPos[LEFT_FOOT].ry);
	kinematicRequest.leftFoot[5] = (float)(currentStep.footPos[LEFT_FOOT].r);

	// rotation right food
	kinematicRequest.rightFoot[3] = (float)(currentStep.footPos[RIGHT_FOOT].rx);
	kinematicRequest.rightFoot[4] = (float)(currentStep.footPos[RIGHT_FOOT].ry);
	kinematicRequest.rightFoot[5] = (float)(currentStep.footPos[RIGHT_FOOT].r);

	// body tilt
	kinematicRequest.body[4] = 0;

	// body shift
	kinematicRequest.body[3] = 0;
#pragma endregion


#pragma region FallingDownCheck

	if (fabs(theSensorData.data[SensorData::angleX])>theWalkingEngineParams.fallDownAngle ||
		fabs(theSensorData.data[SensorData::angleY])>theWalkingEngineParams.fallDownAngle ||
		theFallDownState.state!=FallDownState::upright)
	{
		// Falling down detected
		if (lastFootPositionsValid)
			kinematicRequest=lastRequest;
		fallingDown=true;

		if (theFallDownState.state!=FallDownState::upright)
			reset();
	}
	else
	{
		lastRequest=kinematicRequest;
		lastFootPositionsValid=true;
		fallingDown=false;
	}

#pragma endregion

	
#pragma region Offset
	for (int i=(int)JointData::legLeft0; i<12; i++)
	{
		kinematicRequest.offsets.angles[i]=0;
	}

	if (currentStep.onFloor[LEFT_FOOT] && currentStep.onFloor[RIGHT_FOOT])
	{
		
	}
	if (currentStep.onFloor[LEFT_FOOT] && !currentStep.onFloor[RIGHT_FOOT])
	{
		for (int i=0; i<6; i++)
		{
				if ((*currentFootpos)->kickPhase!=freeLegNA)
				{
					if (fabs(currentStep.footPos[RIGHT_FOOT].z-currentStep.footPos[LEFT_FOOT].z)>0.003)
						kinematicRequest.offsets.angles[i+(int)JointData::legLeft0]=theFreeLegPhaseParams.offsetLeft[i];
				}
				else
					kinematicRequest.offsets.angles[i+(int)JointData::legLeft0]=theWalkingEngineParams.offsetLeft[i];
		}	
	}
	if (!currentStep.onFloor[LEFT_FOOT] && currentStep.onFloor[RIGHT_FOOT])
	{
		for (int i=0; i<6; i++)
		{
			if ((*currentFootpos)->kickPhase!=freeLegNA)
			{
				if (fabs(currentStep.footPos[RIGHT_FOOT].z-currentStep.footPos[LEFT_FOOT].z)>0.003)
					kinematicRequest.offsets.angles[i+(int)JointData::legRight0]=theFreeLegPhaseParams.offsetRight[i];
			}
			else
				kinematicRequest.offsets.angles[i+(int)JointData::legRight0]=theWalkingEngineParams.offsetRight[i];
		}
	}
#pragma endregion

	
}

void CSConverter::updateWalkingInfo(WalkingInfo &walkingInfo)
{
	walkingInfo.robotPosition.translation.x=robotPosition.x;
	walkingInfo.robotPosition.translation.y=robotPosition.y;
	walkingInfo.robotPosition.rotation=robotPosition.r;


	if (fallingDown)
		walkingInfo.isLeavingPossible=true;
	else
		walkingInfo.isLeavingPossible=isLeavingPossible;

	if (isRunning)
	{
		walkingInfo.expectedAcc.x=acc.x;
		walkingInfo.expectedAcc.y=acc.y;

		walkingInfo.odometryOffset.translation.x=odometry.x*1000;
		walkingInfo.odometryOffset.translation.y=odometry.y*1000;
		walkingInfo.odometryOffset.rotation=odometry.r;

		walkingInfo.offsetToRobotPoseAfterPreview.translation.x=offsetToRobotPoseAfterPreview.x*1000;
		walkingInfo.offsetToRobotPoseAfterPreview.translation.y=offsetToRobotPoseAfterPreview.y*1000;
		walkingInfo.offsetToRobotPoseAfterPreview.rotation=offsetToRobotPoseAfterPreview.r;

		walkingInfo.kickPhase=(*currentFootpos)->kickPhase;
		walkingInfo.ballCSinWEWCS.x=lastPos.x;
		walkingInfo.ballCSinWEWCS.y=lastPos.y;
	}
	else
	{
		walkingInfo.expectedAcc.x=0;
		walkingInfo.expectedAcc.y=0;

		walkingInfo.odometryOffset.translation.x=0;
		walkingInfo.odometryOffset.translation.y=0;
		walkingInfo.odometryOffset.rotation=0;
	}
}