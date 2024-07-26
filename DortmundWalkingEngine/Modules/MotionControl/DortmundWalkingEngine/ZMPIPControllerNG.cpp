#include "ZMPIPControllerNG.h"

//#define LOGGING
#ifndef WALKING_SIMULATOR
#include "Tools/Debugging/CSVLogger.h"
#else
#include "csvlogger.h"
#endif

#include "CSTransform.h"

ZMPIPControllerNG::ZMPIPControllerNG(
		const RobotModel			&theRobotModel,
		const RefZMP				&theRefZMP,
		const SensorData			&theSensorData,
		const WalkingEngineParams	&theWalkingEngineParams,
		const PatternGenRequest		&thePatternGenRequest,
		const FallDownState			&theFallDownState,
		const ControllerParams		&theControllerParams,
		const ZMPModel				&theZMPModel,
		const WalkingInfo			&theWalkingInfo,
		const JointData				&theJointData,
		const ActualCoM				&theActualCoM):
  theRobotModel(theRobotModel),
  theRefZMP(theRefZMP),
  theSensorData(theSensorData),
  theWalkingEngineParams(theWalkingEngineParams),
  thePatternGenRequest(thePatternGenRequest),
  theFallDownState(theFallDownState),
  theControllerParams(theControllerParams),
  theZMPModel(theZMPModel),
  theWalkingInfo(theWalkingInfo),
  theJointData(theJointData),
  theActualCoM(theActualCoM)
{
	sensorScale=1.0f;
	reset();																				    
}

void ZMPIPControllerNG::setZMP()
{
	Point realZMP(theZMPModel.zmp_acc.x/1000, theZMPModel.zmp_acc.y/1000);
	Point CoM(0, 0, theControllerParams.z_h, 0);
	CoM.rotateAroundX(theSensorData.data[SensorData::angleX]*theWalkingEngineParams.rollFactor);
	CoM.rotateAroundY(theSensorData.data[SensorData::angleY]*theWalkingEngineParams.tiltFactor);

	positionDelayBuffer.add(robotPosition);

	realZMP+=CoM;
	if (positionDelayBuffer.getNumberOfEntries()>theWalkingEngineParams.sensorDelay)
	{
		realZMP.rotate2D(positionDelayBuffer.getEntry(theWalkingEngineParams.sensorDelay).r);
		realZMP+=positionDelayBuffer.getEntry(theWalkingEngineParams.sensorDelay);
	}
	realZMP.z=0;
	this->realZMP=realZMP;
}


void ZMPIPControllerNG::freeMem()
{
	int size;

	size=pRef.size();
	for (int i=0; i<size; i++)
	{
		ZMP *zmp=pRef.front();
		pRef.pop_front();
		delete zmp;		
	}
}


void ZMPIPControllerNG::Shrink()
{
	while(pRef.front()!=(*kElement))
	{
		ZMP *zmp=pRef.front();
		pRef.pop_front();
		delete zmp;
	}
}

void ZMPIPControllerNG::reset()
{
	cur_x[0]=cur_y[0]=obs_x[0]=obs_y[0]=0;
	cur_x[1]=cur_y[1]=obs_x[1]=obs_y[1]=0;
	cur_x[2]=cur_y[2]=obs_x[2]=obs_y[2]=0;
	vx = vy = 0;

	cont_x=cont_y=0;
	freeMem();
	pRef.clear();

	kElementEmpty=true;

	isRunning=false;
	positionDelayBuffer.init();
	delayBuffer.init();
	coMDelayBuffer.init();
	realZMP=0;
}

Vector2<double> ZMPIPControllerNG::getReferenceZMP()
{
	if (isRunning)
	{
		return Vector2<double>((*kElement)->x, (*kElement)->y);
	}
	else return Vector2<double>(0, 0);
}

Point ZMPIPControllerNG::controllerStep()
{
	static int delayCounter=0;

	Point target(cur_x[2], cur_y[2]);
	Point targetCoM(cur_x[0], cur_y[0]);

	delayBuffer.add(target);
	coMDelayBuffer.add(targetCoM);

	Point ZMPdiff, CoMdiff;
	if (delayBuffer.getNumberOfEntries()>theWalkingEngineParams.sensorDelay)
	{
		ZMPdiff.x=delayBuffer.getEntry(theWalkingEngineParams.sensorDelay).x-realZMP.x;
		ZMPdiff.y=delayBuffer.getEntry(theWalkingEngineParams.sensorDelay).y-realZMP.y;
		CoMdiff.x=coMDelayBuffer.getEntry(theWalkingEngineParams.halSensorDelay).x-theActualCoM.x+theWalkingEngineParams.xOffset;
		CoMdiff.y=coMDelayBuffer.getEntry(theWalkingEngineParams.halSensorDelay).y-theActualCoM.y;
	}


	LOG("CoM", "ActualCoM x", theActualCoM.x);
	LOG("CoM", "ActualCoM y", theActualCoM.y);

	LOG("CoM", "TargetCoM x", coMDelayBuffer.getEntry(theWalkingEngineParams.sensorDelay).x);
	LOG("CoM", "TargetCoM y", coMDelayBuffer.getEntry(theWalkingEngineParams.sensorDelay).y);

	LOG("CoM", "CoMdiff x", CoMdiff.x);
	LOG("CoM", "CoMdiff y", CoMdiff.y);

	bool sensorOn=false;

	if (thePatternGenRequest.newState==PatternGenRequest::walking ||
		theWalkingInfo.kickPhase!=freeLegNA)
		sensorOn=true;

	if (sensorOn && localSensorScale<1)
		localSensorScale+=1.0f/theWalkingEngineParams.zmpSmoothPhase;

	if (!sensorOn)
		delayCounter++;
	else
		delayCounter=0;

	if (delayCounter>theControllerParams.N && localSensorScale>0)
		localSensorScale-=1.0f/theWalkingEngineParams.zmpSmoothPhase;

	double _obs_x[3], _obs_y[3];

	ZMPdiff.rotate2D(-robotPosition.r);
	ZMPdiff *= (localSensorScale * theWalkingEngineParams.sensorControlRatio);
	ZMPdiff.rotate2D(robotPosition.r);

	CoMdiff.rotate2D(-robotPosition.r);
	CoMdiff *= (localSensorScale * theWalkingEngineParams.sensorControlRatio);
	CoMdiff.rotate2D(robotPosition.r);

	Point u;
	ZMPList::iterator _pRef;
	_pRef=kElement;

	for(int j=0; j<theControllerParams.N; j++)
	{
		ASSERT(_pRef!=pRef.end());
		u.x+=theControllerParams.Gd[j]*(*_pRef)->x;
		u.y+=theControllerParams.Gd[j]*(*_pRef)->y;	
		++_pRef;
	}

	cont_x = -theControllerParams.Gi*vx - (theControllerParams.Gx[0] * obs_x[0] + theControllerParams.Gx[1] * obs_x[1] + theControllerParams.Gx[2] * obs_x[2]) - u.x;
	cont_y = -theControllerParams.Gi*vy - (theControllerParams.Gx[0] * obs_y[0] + theControllerParams.Gx[1] * obs_y[1] + theControllerParams.Gx[2] * obs_y[2]) - u.y;

	// currently no CoM error is used, so only the second row of L is used here for the ZMP error

	_obs_x[0]=theControllerParams.A0[0][0]*obs_x[0]+theControllerParams.A0[0][1]*obs_x[1]+theControllerParams.A0[0][2]*obs_x[2]-theControllerParams.L[0][0]*CoMdiff.x-theControllerParams.L[0][1]*ZMPdiff.x;
	_obs_x[1]=theControllerParams.A0[1][0]*obs_x[0]+theControllerParams.A0[1][1]*obs_x[1]+theControllerParams.A0[1][2]*obs_x[2]-theControllerParams.L[1][0]*CoMdiff.x-theControllerParams.L[1][1]*ZMPdiff.x;
	_obs_x[2]=theControllerParams.A0[2][0]*obs_x[0]+theControllerParams.A0[2][1]*obs_x[1]+theControllerParams.A0[2][2]*obs_x[2]-theControllerParams.L[2][0]*CoMdiff.x-theControllerParams.L[2][1]*ZMPdiff.x+theControllerParams.dt*cont_x;

	_obs_y[0]=theControllerParams.A0[0][0]*obs_y[0]+theControllerParams.A0[0][1]*obs_y[1]+theControllerParams.A0[0][2]*obs_y[2]-theControllerParams.L[0][0]*CoMdiff.y-theControllerParams.L[0][1]*ZMPdiff.y;
	_obs_y[1]=theControllerParams.A0[1][0]*obs_y[0]+theControllerParams.A0[1][1]*obs_y[1]+theControllerParams.A0[1][2]*obs_y[2]-theControllerParams.L[1][0]*CoMdiff.y-theControllerParams.L[1][1]*ZMPdiff.y;
	_obs_y[2]=theControllerParams.A0[2][0]*obs_y[0]+theControllerParams.A0[2][1]*obs_y[1]+theControllerParams.A0[2][2]*obs_y[2]-theControllerParams.L[2][0]*CoMdiff.y-theControllerParams.L[2][1]*ZMPdiff.y+theControllerParams.dt*cont_y;

	obs_x[0]=_obs_x[0];
	obs_x[1]=_obs_x[1];
	obs_x[2]=_obs_x[2];

	obs_y[0]=_obs_y[0];
	obs_y[1]=_obs_y[1];
	obs_y[2]=_obs_y[2];

	cur_x[0]=_obs_x[0];
	cur_x[1]=_obs_x[1];
	cur_x[2]=_obs_x[2];

	cur_y[0]=_obs_y[0];
	cur_y[1]=_obs_y[1];
	cur_y[2]=_obs_y[2];

	vx += obs_x[2] - (*kElement)->x;
	vy += obs_y[2] - (*kElement)->y;

	kElement++;

	return Point(cur_x[0], cur_y[0], theControllerParams.z_h);
}


void ZMPIPControllerNG::addRefZMP(ZMP zmp)
{
	pRef.push_back(new ZMP(zmp));

	if (kElementEmpty)
	{
		kElement=pRef.begin();
		kElementEmpty=false;
	}
}

void ZMPIPControllerNG::getObservations(Vector3<double> &x, Vector3<double> &y)
{
	x[0]=obs_x[0];
	x[1]=obs_x[1];
	x[2]=obs_x[2];

	y[0]=obs_y[0];
	y[1]=obs_y[1];
	y[2]=obs_y[2];
}

void ZMPIPControllerNG::updateKinematicRequest(TargetCoM & targetCoM)
{
	for (int i=0; i<theRefZMP.numOfZMP; i++)
		addRefZMP(theRefZMP.getZMP(i));
	targetCoM.x=targetCoM.y=0;
	
	robotPosition.x=theWalkingInfo.robotPosition.translation.x;
	robotPosition.y=theWalkingInfo.robotPosition.translation.y;
	robotPosition.r=theWalkingInfo.robotPosition.rotation;

	if (!isRunning && theRefZMP.running)
	{
		Start();
		isRunning=true;
	}
	setZMP();
	if (isRunning && !theRefZMP.running)
	{
		End();
		isRunning=false;
	}
	if (isRunning)
	{
		ASSERT(pRef.size()>0);
		(Point &)targetCoM=controllerStep();
		Shrink();
#pragma region LOGGING
		LOG("WalkingEngine", "real ZMP x", realZMP.x);
		LOG("WalkingEngine", "real ZMP y", realZMP.y);
		LOG("WalkingEngine", "calc ZMP x", cur_x[2]);
		LOG("WalkingEngine", "calc ZMP y", cur_y[2]);
		LOG("WalkingEngine", "ref ZMP x", (*kElement)->x);
		LOG("WalkingEngine", "ref ZMP y", (*kElement)->y);
		LOG("WalkingEngine", "Acc x", theSensorData.data[SensorData::accX]);
		LOG("WalkingEngine", "Acc y", theSensorData.data[SensorData::accY]);
		LOG("WalkingEngine", "Acc z", theSensorData.data[SensorData::accZ]);
		LOG("WalkingEngine", "Roll", theSensorData.data[SensorData::angleX]);
		LOG("WalkingEngine", "Tilt", theSensorData.data[SensorData::angleY]);
		LOG("WalkingEngine", "Target CoM x", targetCoM.x);
		LOG("WalkingEngine", "Target CoM y", targetCoM.y);
		LOG("WalkingEngine", "Target CoM z", targetCoM.z);
#pragma endregion
	}
}