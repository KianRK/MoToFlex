#include "CoMProvider.h"
#include "CSTransform.h"

using namespace std;

CoMProvider::CoMProvider(		
		const JointData				&theJointData,
		const WalkingEngineParams	&theWalkingEngineParams,
		const JointRequest			&theJointRequest,
		const FootSteps				&theFootSteps,
		const RobotModel			&theRobotModel):
			theJointData(theJointData),
			theWalkingEngineParams(theWalkingEngineParams),
			theJointRequest(theJointRequest),
			theFootSteps(theFootSteps),
			theRobotModel(theRobotModel)
{
}


CoMProvider::~CoMProvider(void)
{
	
}


void CoMProvider::updateActualCoM(ActualCoM &theActualCoM)
{
	CSTransform cstrans;	
	HomMatrix bodyToGround;

	for (int i=0; i<theFootSteps.numOfSteps; i++)
		footPositions.push_back(new Footposition(theFootSteps.getStep(i)));

	if (footPositions.empty()) return;

	double zOffset=0;
#ifndef WALKING_SIMULATOR
	zOffset=0.085;
#endif


	
	HomMatrix measuredCoM(Vector3<double>(theRobotModel.centerOfMass.x/1000, theRobotModel.centerOfMass.y/1000, (theRobotModel.centerOfMass.z)/1000-zOffset));

	Footposition step(*footPositions.front());
	Point footPos, p;
	if (step.onFloor[LEFT_FOOT])
	{
		bodyToGround=cstrans.fromBodyToLeftGround(
			theJointData.angles[JointData::legLeft0],
			-theJointData.angles[JointData::legLeft1],
			theJointData.angles[JointData::legLeft2],
			theJointData.angles[JointData::legLeft3],
			theJointData.angles[JointData::legLeft4],
			-theJointData.angles[JointData::legLeft5]);
		footPos=step.footPos[LEFT_FOOT];
	}
	else
	{
		bodyToGround=cstrans.fromBodyToRightGround(
			theJointData.angles[JointData::legRight0],
			theJointData.angles[JointData::legRight1],
			theJointData.angles[JointData::legRight2],
			theJointData.angles[JointData::legRight3],
			theJointData.angles[JointData::legRight4],
			theJointData.angles[JointData::legRight5]);
		footPos=step.footPos[RIGHT_FOOT];
	}

	p=HomMatrix((bodyToGround * measuredCoM)).getTranslation();
	(Point &)theActualCoM=p+footPos;

	// Delete old foot step
	Footposition *oldPos=footPositions.front();
	footPositions.pop_front();
	delete oldPos;
}
