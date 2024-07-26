#include "ConstantArmProvider.h"

ConstantArmProvider::ConstantArmProvider()
{
	intfac=0;
}


ConstantArmProvider::~ConstantArmProvider()
{

}

void ConstantArmProvider::update(ArmMovement& armMovement)
{
	if (intfac==0)
	{

	}

	//armMovement.angles[JointData::armLeft0]=intfac*-1.7886+(1-intfac)*intstart[0];
	//armMovement.angles[JointData::armLeft1]=intfac*-0.012314+(1-intfac)*intstart[0];
	//armMovement.angles[JointData::armLeft2]=intfac*-1.76261+(1-intfac)*intstart[0];
	//armMovement.angles[JointData::armLeft3]=intfac*-1.56157+(1-intfac)*intstart[0];

	//armMovement.angles[JointData::armRight0]=intfac*-1.75954+(1-intfac)*intstart[0];
	//armMovement.angles[JointData::armRight1]=intfac*-0.026036+(1-intfac)*intstart[0];
	//armMovement.angles[JointData::armRight2]=intfac*-1.6398+(1-intfac)*intstart[0];
	//armMovement.angles[JointData::armRight3]=intfac*-1.55092+(1-intfac)*intstart[0];

	armMovement.angles[JointData::armLeft0]=-2.09;
	armMovement.angles[JointData::armLeft1]=0;
	armMovement.angles[JointData::armLeft2]=-1.7;
	armMovement.angles[JointData::armLeft3]=-1.55;

	armMovement.angles[JointData::armRight0]=-2.09;
	armMovement.angles[JointData::armRight1]=0;
	armMovement.angles[JointData::armRight2]=-1.7;
	armMovement.angles[JointData::armRight3]=-1.55;


	armMovement.usearms=true;
}

MAKE_MODULE(ConstantArmProvider, Dortmund WalkingEngine)
