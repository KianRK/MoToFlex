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

#include "SwingLegController.h"
#include "Tools/Math/Bspline.h"

#ifndef WALKING_SIMULATOR
//#define LOGGING
#include "Tools/Debugging/CSVLogger.h"
#else
#include "csvlogger.h"
#endif

#define LEG_BACK_FACTOR 2

// Starts a new foot reset polygon
// pointcount is the number of points between start and end
#define START_POLYGON(pointcount, start, end) \
	int counter=1; \
	int pc=pointcount; \
	Point *polygon=new Point[pc+2]; \
	polygon[0]=start; \
	polygon[pc+1]=end;

// Defines the next point
// translation is the rate of the full translation from the start to the target
// height is the rate of the full step height
#define POINT(translation, height) \
	polygon[counter]=polygon[0]+(polygon[pc+1]-polygon[0])*translation; \
	polygon[counter].z=polygon[0].z+height; \
	counter++;

// Writes the curve to output 
#define END_POLYGON(output, count, degree) \
	bspline(pc+1, degree, polygon, output, count); \
	delete [] polygon;
// Check the curve in Matlab by using:
// >> plot(data(:,29), data(:,31))
// >> axis([0 0.1 -0.245 0.1-0.245])

void SwingLegController::freeMem()
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

void SwingLegController::reset()
{
	footPositions.clear();
	freeMem();
	expectedBufferSize=0;
	additionalBuffer=0;
	freeLeg=-1;
	currentFreeLegPhase=freeLegNA;
	if 	(initWalkPol!=NULL)
		delete initWalkPol;
	initWalkPol=NULL;
	
}

void SwingLegController::doKick(int footNum)
{
	Point realZMP(theZMPModel.zmp_acc.x/1000, theZMPModel.zmp_acc.y/1000);
	switch (kickPhase)
	{
	case starting:
		if (goToStartCount<=0)
		{
			kickPhase=ongoing;
			currentVec=kickVec;
		}
		
		realZMP.rotate2D(theWalkingInfo.robotPosition.rotation);
		realZMP+=Point(theWalkingInfo.robotPosition.translation.x, theWalkingInfo.robotPosition.translation.y);

    kickStartingCounter++;
		if (!stable &&
			fabs(realZMP.euklidDistance2D((*lastPlannedReset[!footNum])->footPos[!footNum]))<theWalkingEngineParams.zmpMax &&
			fabs(realZMP.euklidDistance2D((*lastPlannedReset[!footNum])->footPos[!footNum]))>theWalkingEngineParams.zmpMin)
		{
			stableCounter++;
			if (stableCounter>=theFreeLegPhaseParams.waitForGoodZMPFrames)
				stable=true;
		}
		if (stable || kickStartingCounter >= 3*theFreeLegPhaseParams.waitForGoodZMPFrames)
		{
			goToStartCount--;
			kickFootPos+=currentVec;
		}
		break;
	case ongoing:
		if (kickCount==theMotionRequest.kickTime)
		{
			currentVec.x=currentVec.y=currentVec.z=0;
			kickPhase=ending;
			endCounter=0;
		}
		kickCount++;
		kickFootPos+=currentVec;
		break;
	case ending:
		if (endCounter<goToEndCount && initWalkPol!=NULL)
		{
			kickFootPos=Vector3<double>(initWalkPol[endCounter].x,
				initWalkPol[endCounter].y,
				initWalkPol[endCounter].z);
				endCounter++;
			if (endCounter==goToEndCount)
			{
				kickPhase=freeLegNA;
				endCounter=0;
				if 	(initWalkPol!=NULL)
				{
					delete initWalkPol;
					initWalkPol=NULL;
				}
			}
		}
		break;
	default:
		break;
	}
}

void SwingLegController::initWalk(int footNum)
{
	stable=false;
	stableCounter=0;
  kickStartingCounter = 0;
	const double normalizeFactor=theFreeLegPhaseParams.normalizeFactor;
	kickPhase=ending;
	currentVec.x=freeLegBoundary[footNum].x-kickFootPos.x;
	currentVec.y=freeLegBoundary[footNum].y-kickFootPos.y;
	currentVec.z=-kickFootPos.z;
	double targetDistance=currentVec.abs();
	currentVec.normalize(normalizeFactor);
	goToEndCount=(int)((targetDistance/currentVec.abs())*LEG_BACK_FACTOR);
	if (goToEndCount<10) goToEndCount=10;
	initWalkPol=new Point[goToEndCount];

	START_POLYGON(5, kickFootPos, Point(freeLegBoundary[footNum].x, freeLegBoundary[footNum].y, 0));
	POINT(0.5f,0);
	POINT(1.0f,-kickFootPos.z*4/7);
	POINT(1.0f,-kickFootPos.z*5/7);
	POINT(1.0f,-kickFootPos.z*6/7);
	POINT(1.0f,-kickFootPos.z*7/7);
	END_POLYGON(initWalkPol, goToEndCount, 4);

	LOG("Kick", "End Vector x", currentVec.x);
	LOG("Kick", "End Vector y", currentVec.y);
	LOG("Kick", "End Vector z", currentVec.z);
	LOG("Kick", "goToEndCount", goToEndCount);
	FLUSH;
}

void SwingLegController::initKick(int footNum)
{
#ifndef WALKING_SIMULATOR

	Vector2<double> endEffectorOffset(-0.06-0.03, 0);
	endEffectorOffset.rotate(theWalkingInfo.robotPosition.rotation);
	Vector2<double> ballPos;

	double kickDirection=theMotionRequest.kickDirection;
	if (footNum==1)
	{
		if (kickDirection > theFreeLegPhaseParams.rotMax)
			kickDirection = theFreeLegPhaseParams.rotMax;
		if (kickDirection < theFreeLegPhaseParams.rotMin)
			kickDirection = theFreeLegPhaseParams.rotMin;
	}
	else
	{
		if (kickDirection < -1*theFreeLegPhaseParams.rotMax)
			kickDirection = -1*theFreeLegPhaseParams.rotMax;
		if (kickDirection > -1*theFreeLegPhaseParams.rotMin)
			kickDirection = -1*theFreeLegPhaseParams.rotMin;
	}
	if (theMotionRequest.kickTime>0)
	{

		ballPos=theBallModel.estimate.position;

		LOG("Kick", "BallPos.x", ballPos.x);
		LOG("Kick", "BallPos.y", ballPos.y);

		if (ballPos.x < theFreeLegPhaseParams.ballXMin*1000)
			ballPos.x = theFreeLegPhaseParams.ballXMin*1000;
		if (ballPos.x > theFreeLegPhaseParams.ballXMax*1000)
			ballPos.x = theFreeLegPhaseParams.ballXMax*1000;

		if (footNum==1)
		{
			if (ballPos.y < theFreeLegPhaseParams.ballYMin*1000)
				ballPos.y = theFreeLegPhaseParams.ballYMin*1000;
			if (ballPos.y > theFreeLegPhaseParams.ballYMax*1000)
				ballPos.y = theFreeLegPhaseParams.ballYMax*1000;
		}
		else
		{
			if (ballPos.y > -1*theFreeLegPhaseParams.ballYMin*1000)
				ballPos.y = -1*theFreeLegPhaseParams.ballYMin*1000;
			if (ballPos.y < -1*theFreeLegPhaseParams.ballYMax*1000)
				ballPos.y = -1*theFreeLegPhaseParams.ballYMax*1000;
		}
		
		LOG("Kick", "clipped BallPos.x", ballPos.x);
		LOG("Kick", "clipped BallPos.y", ballPos.y);

		ballPos=ballPos.rotate(theWalkingInfo.robotPosition.rotation)/1000+theWalkingInfo.ballCSinWEWCS+endEffectorOffset;

		LOG("Kick", "WCS BallPos.x", ballPos.x);
		LOG("Kick", "WCS BallPos.y", ballPos.y);
	}
	else
	{
		ballPos.x=(*lastPlannedReset[footNum])->footPos[footNum].x;
		ballPos.y=(*lastPlannedReset[footNum])->footPos[footNum].y;
		
		MARK("Kick", "BallPos.x");
		MARK("Kick", "BallPos.y");
		MARK("Kick", "clipped BallPos.x");
		MARK("Kick", "clipped BallPos.y");
		MARK("Kick", "WCS BallPos.x");
		MARK("Kick", "WCS BallPos.y");
	}
	
	
	double kickStartIn=theFreeLegPhaseParams.kickStart; // Distance before the ball (in kick direction) [m]
	double kickStopIn=theFreeLegPhaseParams.kickStop; // Distance after the ball [m]
	const double normalizeFactor=theFreeLegPhaseParams.normalizeFactor; 

	if (theMotionRequest.kickTime==0)
	{
		kickStartIn=kickStopIn=0;
	}
	

	// Ball left = leg left, otherwise we have to use the right leg
	if((ballPos.y>=0 && footNum==RIGHT_FOOT) ||
		(ballPos.y<0 && footNum==LEFT_FOOT))
	{
		// wrong foot
	}

	Vector2<double> kickStart2D(kickStartIn, 0);
	kickStart2D=kickStart2D.rotate(theMotionRequest.kickDirection+theWalkingInfo.robotPosition.rotation)+ballPos; // auf rotation!=0 achten bei ballPos
	kickStart.x=kickStart2D.x;
	kickStart.y=kickStart2D.y;
	kickStart.z=theFreeLegPhaseParams.stepHeight;

	LOG("Kick", "kickStart.x", kickStart.x);
	LOG("Kick", "kickStart.y", kickStart.y);
	LOG("Kick", "kickStart.z", kickStart.z);
	
	if (theMotionRequest.kickTime>0)
	{
		Vector2<double> kickVec2D(abs(kickStopIn-kickStartIn)/theMotionRequest.kickTime, 0);
		kickVec2D.rotate(theMotionRequest.kickDirection+theWalkingInfo.robotPosition.rotation);
		kickVec.x=kickVec2D.x;
		kickVec.y=kickVec2D.y;
		kickVec.z=0;
	}
	else
		kickVec=Vector3<double>();

	LOG("Kick", "kickTime", theMotionRequest.kickTime);
	LOG("Kick", "kickDirection", theMotionRequest.kickDirection);
	LOG("Kick", "Normalizer", normalizeFactor);

	LOG("Kick", "kickVec.x", kickVec.x);
	LOG("Kick", "kickVec.y", kickVec.y);

	kickFootPos.x=freeLegBoundary[footNum].x;
	kickFootPos.y=freeLegBoundary[footNum].y;
	goToStartVec.x=kickStart.x-kickFootPos.x;
	goToStartVec.y=kickStart.y-kickFootPos.y;
	goToStartVec.z=theFreeLegPhaseParams.stepHeight;
	double targetDistance=goToStartVec.abs();
	goToStartVec.normalize(normalizeFactor);
	goToStartCount=(int)(targetDistance/goToStartVec.abs());
	currentVec=goToStartVec;
	kickCount=0;
	goToEndCount=0;
	kickPhase=starting;

	LOG("Kick", "kickFootPos.x", kickFootPos.x);
	LOG("Kick", "kickFootPos.y", kickFootPos.y);
	LOG("Kick", "goToStartVec.x", goToStartVec.x);
	LOG("Kick", "goToStartVec.y", goToStartVec.y);
	LOG("Kick", "goToStartVec.z", goToStartVec.z);
	LOG("Kick", "goToStartCount", goToStartCount);

#endif
}

void SwingLegController::freeLegControl(int footNum)
{
	do
	{
		lastPlannedReset[footNum]++;
		
		doKick(footNum);
		(*lastPlannedReset[footNum])->footPos[footNum].x=kickFootPos.x;
		(*lastPlannedReset[footNum])->footPos[footNum].y=kickFootPos.y;
		(*lastPlannedReset[footNum])->footPos[footNum].z=kickFootPos.z;
		(*lastPlannedReset[footNum])->kickPhase=kickPhase;
	}
	while (currentFreeLegPhase==ending && kickPhase==ending && *lastPlannedReset[footNum]!=footPositions.back());
}

void SwingLegController::PlanFootReset(int footNum)
{
	int beginAir=-1, endAir=-1;
	double beginR=0.0, rStep=0.0;
	FootList::iterator _footList, beginAirElement;

	Point polygonStart, polygonEnd, point;
	

	for (_footList=lastPlannedReset[footNum]; footPositions.back()->timestamp!=(*_footList)->timestamp; ++_footList)
	{
		FootList::iterator next=_footList;
		next++;

		if ((*_footList)->phase==unlimitedDoubleSupport && freeLeg==-1)
		{
			lastPlannedReset[footNum]=_footList;	
		}

		if ((*_footList)->phase==unlimitedSingleSupportLeft || (*_footList)->phase==unlimitedSingleSupportRight)
		{
			// Detected upcomming or running free leg phase

			if (freeLeg==-1)
			{
				int newFreeLeg=((*_footList)->phase==unlimitedSingleSupportLeft);
				freeLeg=newFreeLeg;
				endAir=-1;
			}
		}

		if ((*_footList)->onFloor[footNum] && !(*next)->onFloor[footNum] && freeLeg!=footNum)
		{
			if (currentFreeLegPhase==freeLegNA)
			{
				freeLegBoundary[footNum]=(*_footList)->footPos[footNum];
			}

			beginAir=(*next)->timestamp;
			beginAirElement=next;
			// beginAir is first timestamp with foot not on ground
			// so we use the last position on ground, and then the first on ground
			// for our bezier polygon
			polygonStart=(*_footList)->footPos[footNum];
			beginR=polygonStart.r;
		}


		if ( !(*_footList)->onFloor[footNum] && (*next)->onFloor[footNum])
		{
			if (beginAir!=-1)
			{
				endAir=(*_footList)->timestamp;
				polygonEnd=(*next)->footPos[footNum];
				rStep=(polygonEnd.r-beginR)/(endAir-beginAir+1);
			}
		}

		if (beginAir!=-1 && endAir!=-1)
		{
			Point *output;
			output=new Point[endAir-beginAir+1];
			
			START_POLYGON(5, polygonStart, polygonEnd);
			POINT(0,theWalkingEngineParams.heightPolygon[0]*theWalkingEngineParams.stepHeight);
			POINT(0,theWalkingEngineParams.heightPolygon[1]*theWalkingEngineParams.stepHeight);
			POINT(0.5f,theWalkingEngineParams.heightPolygon[2]*theWalkingEngineParams.stepHeight);
			POINT(1.0f,theWalkingEngineParams.heightPolygon[3]*theWalkingEngineParams.stepHeight);
			POINT(1.0f,theWalkingEngineParams.heightPolygon[4]*theWalkingEngineParams.stepHeight);
			END_POLYGON(output, endAir-beginAir+1, POLYNOM_DEGREE);

			int timeOffset=(int)((*beginAirElement)->timestamp);
			for (int i=0; (*beginAirElement)->timestamp!=endAir+1; ++beginAirElement, i++)
			{
				(*beginAirElement)->footPos[footNum]=output[(*beginAirElement)->timestamp-timeOffset];
				(*beginAirElement)->footPos[footNum].r=beginR+i*rStep;
				
				// Add a pitch to the foot so that it does not kick into the ground while moving to its new position
				(*beginAirElement)->footPos[footNum].ry+=((*beginAirElement)->footPos[footNum].z/theWalkingEngineParams.stepHeight)*theWalkingEngineParams.footPitch;
			}

			delete [] output;
			beginAir=endAir=-1;
			lastPlannedReset[footNum]=beginAirElement;
		}
		else
		{
			// No start of an air phase, check if the foot is on ground
			// then we can skip this foot position
			if ((*_footList)->onFloor[footNum] && freeLeg!=footNum)
			{
				lastPlannedReset[footNum]=_footList;
			}

		}

	}
}


void SwingLegController::addFootsteps(const Footposition &fp)
{

	Footposition *newpos=new Footposition;
	*newpos=fp;

	footPositions.push_back(newpos);

	if (footPositions.front()==footPositions.back())
	{
		lastPlannedReset[LEFT_FOOT]=lastPlannedReset[RIGHT_FOOT]=footPositions.begin();
	}
}


void SwingLegController::Shrink(Footpositions &footpositions)
{
	// Check all iterators to be sure, that no one needs the elements to delete
	while(
		footPositions.size()>0 &&
		lastPlannedReset[0]!=footPositions.begin() &&
		lastPlannedReset[1]!=footPositions.begin()
		 )
	{
		Footposition *footPos=footPositions.front();
		footPositions.pop_front();
		footpositions.addFP(*footPos);
		delete footPos;
	}
}

void SwingLegController::updateFootpositions(Footpositions &footpositions)
{
	if (!isRunning && theFootSteps.running)
		reset();
	if (theFallDownState.state!=FallDownState::upright) reset();
	for (int i=0; i<theFootSteps.numOfSteps; i++)
		addFootsteps(theFootSteps.getStep(i));
	isRunning=theFootSteps.running;
	footpositions.init();
	if (footPositions.size()>0)
	{
		PlanFootReset(LEFT_FOOT);
		PlanFootReset(RIGHT_FOOT);
		// Detect current free leg phase
		if (currentFreeLegPhase==ongoing && (footPositions.back()->phase==firstSingleSupport || footPositions.back()->phase==secondSingleSupport || footPositions.back()->phase==unlimitedDoubleSupport))
		{
			freeLegBoundary[freeLeg]=footPositions.back()->footPos[freeLeg];
			endTimestamp=footPositions.back()->timestamp;
			currentFreeLegPhase=ending;
			initWalk(freeLeg);
		}
		if (currentFreeLegPhase==starting && ((*lastPlannedReset[freeLeg])->phase==unlimitedSingleSupportLeft || (*lastPlannedReset[freeLeg])->phase==unlimitedSingleSupportRight))
			currentFreeLegPhase=ongoing;

		if (freeLeg!=-1 && currentFreeLegPhase==freeLegNA && expectedBufferSize==1)
		{
			currentFreeLegPhase=starting;
			initKick(freeLeg);
		}
		// Handle free leg
		if (currentFreeLegPhase!=freeLegNA)
		{
			// Fill the buffer of this module to be able to set the
			// foot positions to the oldest element that will be executed imediatetly
			if (expectedBufferSize==1)
				freeLegControl(freeLeg);
			if (((*lastPlannedReset[freeLeg])->onFloor[freeLeg]) && currentFreeLegPhase==ending)
			{
				freeLeg=-1;
				currentFreeLegPhase=freeLegNA;
			}
		}
		// delete unused positions and fill the buffer of footpositions
		Shrink(footpositions);
	}

	expectedBufferSize+=footpositions.numOfFP-1;
	footpositions.running=theFootSteps.running;
	footpositions.suggestedStep=theFootSteps.suggestedStep;
}