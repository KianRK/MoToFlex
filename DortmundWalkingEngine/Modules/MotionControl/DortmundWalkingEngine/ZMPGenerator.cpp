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

#include "ZMPGenerator.h"
#include "Bezier.h"

#define FOOT_LEN	0.16f

ZMPGenerator::ZMPGenerator(		
		const FootSteps				&theFootSteps,
		const WalkingEngineParams	&theWalkingEngineParams,
		const ControllerParams		&theControllerParams,
		const FreeLegPhaseParams	&theFreeLegPhaseParams,
		const WalkingInfo			&theWalkingInfo):
	theFootSteps(theFootSteps),
	theWalkingEngineParams(theWalkingEngineParams),
	theControllerParams(theControllerParams),
	theFreeLegPhaseParams(theFreeLegPhaseParams),
	theWalkingInfo(theWalkingInfo)
{
	reset();
}


ZMPGenerator::~ZMPGenerator(void)
{
	freeMem();
}

void ZMPGenerator::reset()
{
	freeMem();
	lpxss=0;
}

void ZMPGenerator::Shrink()
{
	while(lastPlannedZMP!=footPositions.begin())
	{
		Footposition *footPos=footPositions.front();
		footPositions.pop_front();
		delete footPos;
	}
}

void ZMPGenerator::freeMem()
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

void ZMPGenerator::addFootsteps(const Footposition &fp)
{
	unsigned int addTime=0;

	Footposition *newpos=new Footposition;
	*newpos=fp;

	if (!footPositions.empty())
	{
		addTime=footPositions.back()->timestamp+1;
		newpos->timestamp=addTime;
	}
	else
		newpos->timestamp=0;

	footPositions.push_back(newpos);

	if (footPositions.front()==footPositions.back())
		lastPlannedZMP=footPositions.begin();

}


void ZMPGenerator::planZMP(RefZMP &refZMP)
{
	FootList::iterator _footList;
	double diff;

	for (_footList=lastPlannedZMP; _footList!=footPositions.end(); ++_footList)
	{
		float vxss;
		float spdx=(float)(*_footList)->speed.x;
		float tss=(*_footList)->singleSupportLen*(float)theControllerParams.dt;
		float tds=(*_footList)->doubleSupportLen*(float)theControllerParams.dt;
		unsigned int pc=(*_footList)->frameInPhase;
		float plgn[4];

		if (spdx<FOOT_LEN/tss)
			vxss=spdx;
		else
			vxss=FOOT_LEN/tss;
		float vxds=((*_footList)->stepDuration*spdx-(2*tss*vxss))/(2*tds);

		Point p, fp; // p: ZMP in foot coordinate system. fp: Position of foot (including
		// rotation. So, first create ZMP trajectory through foot and 
		// translate/rotate it into the woorld coordinate system by using fp

		switch ((*_footList)->phase)
		{
		case firstSingleSupport:
			fp=(*_footList)->footPos[LEFT_FOOT];
			break;
		case firstDoubleSupport:
			fp=(*_footList)->footPos[LEFT_FOOT];
			break;
		case secondSingleSupport:
			fp=(*_footList)->footPos[RIGHT_FOOT];
			break;
		case secondDoubleSupport:
			fp=(*_footList)->footPos[RIGHT_FOOT];
			break;
		default:
			break;
		}

		Point rf=(*_footList)->footPos[RIGHT_FOOT];
		Point lf=(*_footList)->footPos[LEFT_FOOT];

		rf.rotate2D(-fp.r);
		lf.rotate2D(-fp.r);

		// Diese Art der Erzeugung ist bei Kurven nicht mehr korrekt. Um wirklich eine möglichst Gleichförmige
		// Vorwärtsbewegung zu erzeugen muß der ZMP im Roboterkoordinatensystem in x und y Richtung erzeugt werden,
		// nicht wie derzeit im Fußkoordinatensystem. Allerdings erfordert das Bedingungen wie die Ausrichtung
		// des Standfußes zu Begin und zum Ende der Phase für die x-Geschwindigkeit (da diese dann nicht mehr nur
		// von der Fußlänge abhängt, sondern auch von der Rotation), und die aktuelle Ausrichtung, um die aktuelle
		// Breite des Fußes bestimmen zu können (wichtig für die y-Schwingung).
		
		float pxss=vxss*tss/2;
		bool toConvert=false;
		switch ((*_footList)->phase)
		{
		case firstSingleSupport:
			if (theWalkingInfo.kickPhase!=freeLegNA)
			{
				p=(*_footList)->footPos[LEFT_FOOT]+Point(theFreeLegPhaseParams.zmpLeftX, theFreeLegPhaseParams.zmpLeftY);
			}
			else
			{
				p.y=FourPointBezier1D(theWalkingEngineParams.polygonLeft, (float)(*_footList)->frameInPhase/(*_footList)->singleSupportLen);
				p.x=-pxss+pc*(float)theControllerParams.dt*vxss;
				toConvert=true;
			}
			break;
		case firstDoubleSupport:
			plgn[0]=theWalkingEngineParams.polygonLeft[3];
			plgn[1]=(float)(rf.y-lf.y)/2;
			plgn[2]=plgn[1];
			plgn[3]=-(float)(lf.y-rf.y)+theWalkingEngineParams.polygonRight[0];
			p.y=FourPointBezier1D(plgn, (float)(*_footList)->frameInPhase/(*_footList)->doubleSupportLen); 
			p.x=+pxss+pc*(float)theControllerParams.dt*vxds; 
			toConvert=true;
			break;
		case secondSingleSupport:
			if (theWalkingInfo.kickPhase!=freeLegNA)
			{
				p=(*_footList)->footPos[RIGHT_FOOT]+Point(theFreeLegPhaseParams.zmpRightX, theFreeLegPhaseParams.zmpRightY);
			}
			else
			{
				p.y=FourPointBezier1D(theWalkingEngineParams.polygonRight, (float)(*_footList)->frameInPhase/(*_footList)->singleSupportLen);
				p.x=-pxss+pc*(float)theControllerParams.dt*vxss;
				toConvert=true;
			}
			break;
		case secondDoubleSupport:
			plgn[0]=theWalkingEngineParams.polygonRight[3];
			plgn[1]=-(float)(rf.y-lf.y)/2;
			plgn[2]=plgn[1];
			plgn[3]=(float)(lf.y-rf.y)+theWalkingEngineParams.polygonLeft[0];
			p.y=FourPointBezier1D(plgn, (float)(*_footList)->frameInPhase/(*_footList)->doubleSupportLen); 
			p.x=+pxss+pc*(float)theControllerParams.dt*vxds;
			toConvert=true;
			break;
		case unlimitedSingleSupportLeft:
			p=(*_footList)->footPos[LEFT_FOOT]+Point(theFreeLegPhaseParams.zmpLeftX, theFreeLegPhaseParams.zmpLeftY);
			diff=p.y-zmp.y;
			if (fabs(diff)>fabs(theFreeLegPhaseParams.zmpMoveSpeedY))
				p.y=zmp.y+sgn(p.y-zmp.y)*theFreeLegPhaseParams.zmpMoveSpeedY;
			break;
		case unlimitedSingleSupportRight:
			p=(*_footList)->footPos[RIGHT_FOOT]+Point(theFreeLegPhaseParams.zmpRightX, theFreeLegPhaseParams.zmpRightY);
			diff=p.y-zmp.y;
			if (fabs(diff)>fabs(theFreeLegPhaseParams.zmpMoveSpeedY))
				p.y=zmp.y+sgn(p.y-zmp.y)*theFreeLegPhaseParams.zmpMoveSpeedY;
			break;
		default:
			p=(*_footList)->footPos[RIGHT_FOOT]+((*_footList)->footPos[LEFT_FOOT]-(*_footList)->footPos[RIGHT_FOOT])/2;
			break;
		}
			
		//float dpxss=lpxss;
		if (spdx>0 && -lpxss>p.x)
			// speed is >0, so the zmp should go forward, but wants to jump back. We won't allow it, and wait
			// until the zmp reaches the last zmp point
			p.x=-lpxss;
		else
			lpxss=pxss;

		// Translate and rotate the ZMP to world coordinate system
		if (toConvert)
		{
			p.rotate2D((*_footList)->direction);
			p+=fp;
		}

		
		zmp=p;
		zmp.timestamp=(*_footList)->timestamp;
		refZMP.addZMP(zmp);
	}

	lastPlannedZMP=_footList;
}

void ZMPGenerator::updateRefZMP(RefZMP& refZMP)
{
	for (int i=0; i<theFootSteps.numOfSteps; i++)
		addFootsteps(theFootSteps.getStep(i));


	refZMP.init();
	refZMP.running=theFootSteps.running;

	if (!isRunning && theFootSteps.running)
	{
		
		isRunning=true;
	}

	if (isRunning && !theFootSteps.running)
	{
		reset();
		isRunning=false;
	}

	if (isRunning)
	{
		planZMP(refZMP);
		Shrink();
	}

	
}