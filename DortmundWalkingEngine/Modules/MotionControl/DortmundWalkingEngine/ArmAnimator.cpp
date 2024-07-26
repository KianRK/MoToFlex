#include "ArmAnimator.h"

ArmAnimator::ArmAnimator()
{

}


ArmAnimator::~ArmAnimator()
{

}

void ArmAnimator::update(ArmMovement& armMovement)
{
	armMovement.angles[JointData::armLeft0]=0;
	armMovement.angles[JointData::armLeft1]=(float)(theWalkingEngineParams.arms1*3.1415/180);
	armMovement.angles[JointData::armLeft2]=0;
	armMovement.angles[JointData::armLeft3]=0;

	armMovement.angles[JointData::armRight0]=0;
	armMovement.angles[JointData::armRight1]=(float)(theWalkingEngineParams.arms1*3.1415/180);
	armMovement.angles[JointData::armRight2]=0;
	armMovement.angles[JointData::armRight3]=0;


	float xOffset=(theKinematicRequest.leftFoot[0]+theKinematicRequest.rightFoot[0])/2;
	float leftArm,rightArm;

	leftArm=-90*3.1415f/180 - theWalkingEngineParams.armFactor*(theKinematicRequest.leftFoot[0]-xOffset);
	rightArm=-90*3.1415f/180 - theWalkingEngineParams.armFactor*(theKinematicRequest.rightFoot[0]-xOffset);
	leftArm=leftArm<-3.1415F?-3.1415F:(leftArm>0?0:leftArm);
	rightArm=rightArm<-3.1415F?-3.1415F:(rightArm>0?0:rightArm);

	armMovement.angles[JointData::armLeft0] = leftArm + theBodyTilt.y; 
	armMovement.angles[JointData::armRight0]= rightArm + theBodyTilt.y;

	armMovement.usearms=true;
}

MAKE_MODULE(ArmAnimator, Dortmund WalkingEngine)
