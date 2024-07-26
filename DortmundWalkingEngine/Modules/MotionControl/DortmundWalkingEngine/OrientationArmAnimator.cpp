#include "OrientationArmAnimator.h"

OrientationArmAnimator::OrientationArmAnimator()
{

}


OrientationArmAnimator::~OrientationArmAnimator()
{

}

void OrientationArmAnimator::update(ArmMovement& armMovement)
{
	armMovement.angles[JointData::armLeft0]=0;
	armMovement.angles[JointData::armLeft1]=(float)(theWalkingEngineParams.arms1*3.1415/180);
	armMovement.angles[JointData::armLeft2]=0;
	armMovement.angles[JointData::armLeft3]=0;

	armMovement.angles[JointData::armRight0]=0;
	armMovement.angles[JointData::armRight1]=(float)(theWalkingEngineParams.arms1*3.1415/180);
	armMovement.angles[JointData::armRight2]=0;
	armMovement.angles[JointData::armRight3]=0;

	float leftArm,rightArm;

	rightArm=leftArm=-90*3.1415f/180 - theWalkingEngineParams.armFactor*((float)theSensorData.data[SensorData::angleY]);
	armMovement.angles[JointData::armLeft0] = leftArm; 
	armMovement.angles[JointData::armRight0]= rightArm;
	armMovement.usearms=true;
}

MAKE_MODULE(OrientationArmAnimator, Dortmund WalkingEngine)
