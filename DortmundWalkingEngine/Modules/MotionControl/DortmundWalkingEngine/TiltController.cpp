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

#include "TiltController.h"
#include <algorithm>
using namespace std;

TiltController::TiltController(		
		const SensorData			&theSensorData,
		const WalkingEngineParams	&theWalkingEngineParams,
		const JointCalibration		&theJointCalibration,
		const JointRequest				&theJointRequest):
	theSensorData(theSensorData),
	theWalkingEngineParams(theWalkingEngineParams),
	theJointCalibration(theJointCalibration),
	theJointRequest(theJointRequest)
{
	angleSum=0;
}


TiltController::~TiltController(void)
{
	
}

void TiltController::updateBodyTilt(BodyTilt &bodyTilt)
{
	bodyTilt.x=-(-theSensorData.data[SensorData::angleX]+(theSensorData.data[SensorData::angleX]/theWalkingEngineParams.rollPFactor));
	bodyTilt.y=-(-theSensorData.data[SensorData::angleY]+(theSensorData.data[SensorData::angleY]/theWalkingEngineParams.tiltPFactor));

	/*  Check joint constraints, if a joint is near to its min/max
		angles, adjust the body tilt */

	if (theJointRequest.angles[JointRequest::legLeft1]!=1000)
	{
		double diff1=0, diff2=0, diff=0, pBonus=0;

		if (theWalkingEngineParams.angleTiltP==0)
			pBonus=0.05;

		// Rotate if joint reach min/max
		// Left leg
		if (theJointRequest.angles[JointRequest::legLeft1]>theJointCalibration.joints[JointRequest::legLeft1].maxAngle)
			diff1=theJointRequest.angles[JointRequest::legLeft1]-theJointCalibration.joints[JointRequest::legLeft1].maxAngle;

		if (theJointRequest.angles[JointRequest::legLeft5]<theJointCalibration.joints[JointRequest::legLeft5].minAngle)
			diff2=theJointCalibration.joints[JointRequest::legLeft5].minAngle-theJointRequest.angles[JointRequest::legLeft5];

		angleSum.x-=theWalkingEngineParams.angleTiltP*max(diff1, diff2);

		// Right leg
		if (theJointRequest.angles[JointRequest::legRight1]>theJointCalibration.joints[JointRequest::legRight1].maxAngle)
			diff1=theJointRequest.angles[JointRequest::legRight1]-theJointCalibration.joints[JointRequest::legRight1].maxAngle;

		if (theJointRequest.angles[JointRequest::legRight5]<theJointCalibration.joints[JointRequest::legRight5].minAngle)
			diff2=theJointCalibration.joints[JointRequest::legRight5].minAngle-theJointRequest.angles[JointRequest::legRight5];

		angleSum.x+=theWalkingEngineParams.angleTiltP*max(diff1, diff2);

		// Rotate back if not
		if (theJointRequest.angles[JointRequest::legLeft5]>theJointCalibration.joints[JointRequest::legLeft5].minAngle &&
			theJointRequest.angles[JointRequest::legLeft1]<theJointCalibration.joints[JointRequest::legLeft1].maxAngle &&
			angleSum.x<0)
		{
			diff1=theJointCalibration.joints[JointRequest::legLeft1].maxAngle-theJointRequest.angles[JointRequest::legLeft1];
			diff2=theJointRequest.angles[JointRequest::legLeft5]-theJointCalibration.joints[JointRequest::legLeft5].minAngle;
			diff=min(diff1, diff2);
			if (fabs(diff)>fabs(angleSum.x))
				angleSum.x=0;
			else
				angleSum.x+=(theWalkingEngineParams.angleTiltP+pBonus)*diff;
		}

		if (theJointRequest.angles[JointRequest::legRight5]>theJointCalibration.joints[JointRequest::legRight5].minAngle &&
			theJointRequest.angles[JointRequest::legRight1]<theJointCalibration.joints[JointRequest::legRight1].maxAngle &&
			angleSum.x>0)
		{
			diff1=theJointCalibration.joints[JointRequest::legRight1].maxAngle-theJointRequest.angles[JointRequest::legRight1];
			diff2=theJointRequest.angles[JointRequest::legRight5]-theJointCalibration.joints[JointRequest::legRight5].minAngle;
			diff=min(diff1, diff2);
			if (fabs(diff)>fabs(angleSum.x))
				angleSum.x=0;
			else
				angleSum.x-=(theWalkingEngineParams.angleTiltP+pBonus)*diff;
		}
	}

	bodyTilt.x+=(float)angleSum.x;

}