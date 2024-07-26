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
 * @file dortmundwalkingengine.cpp
 * Used by the simulation to retrieve the target angles for the robot.
 * Here the Dortmund Walking Engine
 * is used to calculate the angles on line.
 * @author <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a>
 */

#include "dortmundwalkingengine.h"
#include "eaconfig.h"

DortmundWalkingEngineAdapter::DortmundWalkingEngineAdapter() :
	patternGenerator(theWalkingEngineParams, thePatternGenRequest, theRobotModel, theRobotDimensions, theFallDownState, theControllerParams, theMotionRequest, theWalkingInfo),
	zmpGenerator(theFootSteps, theWalkingEngineParams, theControllerParams, theFreeLegPhaseParams, theWalkingInfo),
	zmpipController(theRobotModel, theRefZMP, theSensorData, theWalkingEngineParams, thePatternGenRequest, theFallDownState, theControllerParams, theZMPModel, theWalkingInfo),
	swingLegController(theWalkingEngineParams, theFootSteps, theBallModel, theMotionRequest, theWalkingInfo, theFreeLegPhaseParams, theFallDownState, theZMPModel),
	csConverter(theFootpositions, theTargetCoM, theWalkingEngineParams, theControllerParams, theRobotModel, theFallDownState, theSensorData, theBodyTilt, theFreeLegPhaseParams),
	tiltController(theSensorData, theWalkingEngineParams, theJointCalibration, theJointRequest)
{
	// The walk request. Set the desired speed (in directions x, y and r) here.
	thePatternGenRequest.newState=PatternGenRequest::walking;
	thePatternGenRequest.speed.translation.x=0.05;

	// The robot is upright all the time, if it falls down, the simulation ends.
	theFallDownState.state=FallDownState::upright;
}



void DortmundWalkingEngineAdapter::getAngles(float angles[], int numOfAngles, Robot &robot)
{
	// Fill some structures.
	theRobotModel.centerOfMass=robot.getCoM()*1000;

	theRobotDimensions.heightLeg5Joint=-robot.ankleToGround[2]*1000;
	theRobotDimensions.lowerLegLength=-robot.kneeToAnkle[2]*1000;
	theRobotDimensions.upperLegLength=-robot.hipToKnee[2]*1000;
	theRobotDimensions.lengthBetweenLegs=-2*robot.bodyToRightHip[1]*1000;

	// Now the modules of the walking engine are called.
	patternGenerator.updateFootSteps(theFootSteps);
	zmpGenerator.updateRefZMP(theRefZMP);
	swingLegController.updateFootpositions(theFootpositions);
	zmpipController.updateKinematicRequest(theTargetCoM);
	tiltController.updateBodyTilt(theBodyTilt);
	csConverter.updateKinematicRequest(theKinematicRequest);

	naoKinematic.theJointData=theJointData;
	naoKinematic.theKinematicRequest=theKinematicRequest;
	naoKinematic.theRobotDimensions=theRobotDimensions;
	naoKinematic.update(theKinematicOutput);

	angles[0]=converter[0]*(float)theKinematicOutput.angles[KinematicOutput::legLeft1];
	angles[1]=converter[1]*(float)theKinematicOutput.angles[KinematicOutput::legLeft2];
	angles[2]=converter[2]*(float)theKinematicOutput.angles[KinematicOutput::legLeft3];
	angles[3]=converter[3]*(float)theKinematicOutput.angles[KinematicOutput::legLeft4];
	angles[4]=converter[4]*(float)theKinematicOutput.angles[KinematicOutput::legLeft5];
	angles[5]=converter[5]*(float)theKinematicOutput.angles[KinematicOutput::legRight1];
	angles[6]=converter[6]*(float)theKinematicOutput.angles[KinematicOutput::legRight2];
	angles[7]=converter[7]*(float)theKinematicOutput.angles[KinematicOutput::legRight3];
	angles[8]=converter[8]*(float)theKinematicOutput.angles[KinematicOutput::legRight4];
	angles[9]=converter[9]*(float)theKinematicOutput.angles[KinematicOutput::legRight5];
}

void DortmundWalkingEngineAdapter::setSettingsPath(string path)
{
	string wep="walkingParams.cfg";
	string cntp="ZMPIPController.dat";
	loadWEParams((path+wep).c_str());
	loadCParams((path+cntp).c_str());
}

void DortmundWalkingEngineAdapter::loadWEParams(const char *path)
{
	FILE *stream;

	stream=fopen(path, "r");
	fscanf(stream, "%*[^\n]%f\r\n", &theWalkingEngineParams.footPitch);
	fscanf(stream, "%*[^\n]%f\r\n", &theWalkingEngineParams.maxWalkPitch);
	fscanf(stream, "%*[^\n]%f\r\n", &theWalkingEngineParams.pitchSpeed);
	fscanf(stream, "%*[^\n]%f\r\n", &theWalkingEngineParams.xOffset);
	fscanf(stream, "%*[^\n]%f\r\n", &theWalkingEngineParams.stepHeight);
	fscanf(stream, "%*[^\n]%f\r\n", &theWalkingEngineParams.sensorControlRatio);
	fscanf(stream, "%*[^\n]%f\r\n", &theWalkingEngineParams.doubleSupportRatio); 
	fscanf(stream, "%*[^\n]%d\r\n", &theWalkingEngineParams.crouchingDownPhaseLength);
	fscanf(stream, "%*[^\n]%d\r\n", &theWalkingEngineParams.startingPhaseLength);
	fscanf(stream, "%*[^\n]%d\r\n", &theWalkingEngineParams.stoppingPhaseLength);
	fscanf(stream, "%*[^\n]%f\r\n", &theWalkingEngineParams.armFactor);
	fscanf(stream, "%*[^\n]%f\r\n", &theWalkingEngineParams.arms1);
	fscanf(stream, "%*[^\n]%d\r\n", &theWalkingEngineParams.zmpSmoothPhase);
	fscanf(stream, "%*[^\n]%d\r\n", &theWalkingEngineParams.maxSpeedXForward);
	fscanf(stream, "%*[^\n]%d\r\n", &theWalkingEngineParams.maxSpeedXForwardOmni);
	fscanf(stream, "%*[^\n]%d\r\n", &theWalkingEngineParams.maxSpeedXBack);
	fscanf(stream, "%*[^\n]%d\r\n", &theWalkingEngineParams.maxSpeedYLeft);
	fscanf(stream, "%*[^\n]%d\r\n", &theWalkingEngineParams.maxSpeedYRight);
	fscanf(stream, "%*[^\n]%f\r\n", &theWalkingEngineParams.maxSpeedR);
	fscanf(stream, "%*[^\n]%f\r\n", &theWalkingEngineParams.maxStandPitch);
	fscanf(stream, "%*[^\n]%f\r\n", &theWalkingEngineParams.maxLegLength);
	fscanf(stream, "%*[^\n]%f\r\n", &theWalkingEngineParams.stepDuration);
	fscanf(stream, "%*[^\n]%f\r\n", &theWalkingEngineParams.footYDistance);
	fscanf(stream, "%*[^\n]%f\r\n", &theWalkingEngineParams.stopPosThresholdX);
	fscanf(stream, "%*[^\n]%f\r\n", &theWalkingEngineParams.stopPosThresholdY);
	fscanf(stream, "%*[^\n]%f\r\n", &theWalkingEngineParams.stopSpeedThresholdX);
	fscanf(stream, "%*[^\n]%f\r\n", &theWalkingEngineParams.stopSpeedThresholdY);
	fscanf(stream, "%*[^\n]%f\r\n", &theWalkingEngineParams.zmpLeftX);
	fscanf(stream, "%*[^\n]%f\r\n", &theWalkingEngineParams.zmpLeftY);
	fscanf(stream, "%*[^\n]%f\r\n", &theWalkingEngineParams.zmpRightX);
	fscanf(stream, "%*[^\n]%f\r\n", &theWalkingEngineParams.zmpRightY);
	fscanf(stream, "%*[^\n]%d\r\n", &theWalkingEngineParams.outFilterOrder);
	fscanf(stream, "%*[^\n]%f\r\n", &theWalkingEngineParams.tiltFactor);
	fscanf(stream, "%*[^\n]%f\r\n", &theWalkingEngineParams.rollFactor);
	fscanf(stream, "%*[^\n]%f\r\n", &theWalkingEngineParams.tiltPFactor);
	fscanf(stream, "%*[^\n]%f\r\n", &theWalkingEngineParams.rollPFactor);
	fscanf(stream, "%*[^\n]%d\r\n", &theWalkingEngineParams.sensorDelay);
	fscanf(stream, "%*[^\n]%f\r\n", &theWalkingEngineParams.maxFootSpeed);
	fscanf(stream, "%*[^\n]%f\r\n", &theWalkingEngineParams.fallDownAngle);
	fscanf(stream, "%*[^\n]%f %f %f %f\r\n", theWalkingEngineParams.polygonLeft, theWalkingEngineParams.polygonLeft+1, theWalkingEngineParams.polygonLeft+2, theWalkingEngineParams.polygonLeft+3);
	fscanf(stream, "%*[^\n]%f %f %f %f\r\n", theWalkingEngineParams.polygonRight, theWalkingEngineParams.polygonRight+1, theWalkingEngineParams.polygonRight+2,  theWalkingEngineParams.polygonRight+3);
	fscanf(stream, "%*[^\n]%f %f %f %f %f %f\r\n", theWalkingEngineParams.offsetLeft, theWalkingEngineParams.offsetLeft+1, theWalkingEngineParams.offsetLeft+2,  theWalkingEngineParams.offsetLeft+3, theWalkingEngineParams.offsetLeft+4, theWalkingEngineParams.offsetLeft+5);
	fscanf(stream, "%*[^\n]%f %f %f %f %f %f\r\n", theWalkingEngineParams.offsetRight, theWalkingEngineParams.offsetRight+1, theWalkingEngineParams.offsetRight+2,  theWalkingEngineParams.offsetRight+3, theWalkingEngineParams.offsetRight+4, theWalkingEngineParams.offsetRight+5);
	fscanf(stream, "%*[^\n]%d\r\n", &theWalkingEngineParams.walkRequestFilterLen);
	fscanf(stream, "%*[^\n]%f\r\n", &theWalkingEngineParams.maxAccX);
	fscanf(stream, "%*[^\n]%d\r\n", &theWalkingEngineParams.accDelayX);
	fscanf(stream, "%*[^\n]%f\r\n", &theWalkingEngineParams.maxAccY);
	fscanf(stream, "%*[^\n]%d\r\n", &theWalkingEngineParams.accDelayY);
	fscanf(stream, "%*[^\n]%f\r\n", &theWalkingEngineParams.maxAccR);
	fscanf(stream, "%*[^\n]%d\r\n", &theWalkingEngineParams.accDelayR);
	fscanf(stream, "%*[^\n]%d\r\n", &theWalkingEngineParams.speedApplyDelay);
	fscanf(stream, "%*[^\n]%d %d %d %d %d %d\r\n", theWalkingEngineParams.legJointHardness, theWalkingEngineParams.legJointHardness+1, theWalkingEngineParams.legJointHardness+2, theWalkingEngineParams.legJointHardness+3, theWalkingEngineParams.legJointHardness+4, theWalkingEngineParams.legJointHardness+5);
	fscanf(stream, "%*[^\n]%f %f %f %f %f %f\r\n", theWalkingEngineParams.heightPolygon, theWalkingEngineParams.heightPolygon+1, theWalkingEngineParams.heightPolygon+2, theWalkingEngineParams.heightPolygon+3, theWalkingEngineParams.heightPolygon+4, theWalkingEngineParams.heightPolygon+5);
}

int DortmundWalkingEngineAdapter::loadCParams(const char *path)
{
	FILE *stream;
	//int count=0;
	char tempstr[200];

	stream=fopen(path, "r");

	fscanf(stream, "%s", tempstr);
	theControllerParams.z_h = strtod(tempstr, NULL);

	fscanf(stream, "%s", tempstr);
	theControllerParams.dt = strtod(tempstr, NULL);

	fscanf(stream, "%s", tempstr);
	theControllerParams.N = (unsigned int)strtod(tempstr, NULL);

	fscanf(stream, "%s", tempstr);
	theControllerParams.L[0][0] = strtod(tempstr, NULL);
	fscanf(stream, "%s", tempstr);
	theControllerParams.L[1][0] = strtod(tempstr, NULL);
	fscanf(stream, "%s", tempstr);
	theControllerParams.L[2][0] = strtod(tempstr, NULL);

    fscanf(stream, "%s", tempstr);
	theControllerParams.L[0][1] = strtod(tempstr, NULL);
	fscanf(stream, "%s", tempstr);
	theControllerParams.L[1][1] = strtod(tempstr, NULL);
	fscanf(stream, "%s", tempstr);
	theControllerParams.L[2][1] = strtod(tempstr, NULL);

	for (int row=0; row<3; row++)
	{
		for (int col=0; col<3; col++)
		{
			fscanf(stream, "%s", tempstr);
			theControllerParams.A0[row][col]= strtod(tempstr, NULL);
		}
	}

	fscanf(stream, "%s", tempstr);
	theControllerParams.Gi = strtod(tempstr, NULL);

	for (int i=0; i<3; i++)
	{
		fscanf(stream, "%s", tempstr);
		theControllerParams.Gx[i] = strtod(tempstr, NULL);	
	}

	for (int i=0; i<3; i++)
	{
		fscanf(stream, "%s", tempstr);
		theControllerParams.b0[i] = strtod(tempstr, NULL);	
	}

	for (int i=0; i<3; i++)
	{
		fscanf(stream, "%s", tempstr);
		theControllerParams.c0[i] = strtod(tempstr, NULL);	
	}


	theControllerParams.Gd = new double[theControllerParams.N];

	for (int i=0; i<theControllerParams.N; i++)
	{   
		fscanf(stream, "%s", tempstr);
		theControllerParams.Gd[i] = strtod(tempstr, NULL);
	}


	fclose(stream);

	return 1;
}
