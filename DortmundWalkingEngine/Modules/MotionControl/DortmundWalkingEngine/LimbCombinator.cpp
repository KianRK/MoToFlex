#include "LimbCombinator.h"


//#define LOGGING
#include "Tools/Debugging/CSVLogger.h"
#include "Tools/Debugging/DebugDrawings.h"

LimbCombinator::LimbCombinator()
{
	init=false;
//	waitLegHardness=0;
//	waitLegSoftness=theWalkingEngineParams.softnessDelay;
}

 
LimbCombinator::~LimbCombinator()
{
}

void LimbCombinator::update(WalkingEngineOutput& walkingEngineOutput)
{
	if (!init)
	{
		for (int i=0; i<JointData::numOfJoints; i++)
			filter[i].createBuffer(theWalkingEngineParams.outFilterOrder);

		init=true;
	}

#pragma region Logging
	LOG("GlobalAngles", "JointData LHipYawPitch", theJointData.angles[JointData::legLeft0]);
	LOG("GlobalAngles", "JointData LHipRoll", theJointData.angles[JointData::legLeft1]);
	LOG("GlobalAngles", "JointData LHipPitch", theJointData.angles[JointData::legLeft2]);
	LOG("GlobalAngles", "JointData LKneePitch", theJointData.angles[JointData::legLeft3]);
	LOG("GlobalAngles", "JointData LAnklePitch", theJointData.angles[JointData::legLeft4]);
	LOG("GlobalAngles", "JointData LAnkleRoll", theJointData.angles[JointData::legLeft5]);

	LOG("GlobalAngles", "JointData RHipYawPitch", theJointData.angles[JointData::legRight0]);
	LOG("GlobalAngles", "JointData RHipRoll", theJointData.angles[JointData::legRight1]);
	LOG("GlobalAngles", "JointData RHipPitch", theJointData.angles[JointData::legRight2]);
	LOG("GlobalAngles", "JointData RKneePitch", theJointData.angles[JointData::legRight3]);
	LOG("GlobalAngles", "JointData RAnklePitch", theJointData.angles[JointData::legRight4]);
	LOG("GlobalAngles", "JointData RAnkleRoll", theJointData.angles[JointData::legRight5]);

	LOG("GlobalAngles", "LimbCombinator LHipYawPitch", walkingEngineOutput.angles[JointData::legLeft0]);
	LOG("GlobalAngles", "LimbCombinator LHipRoll", walkingEngineOutput.angles[JointData::legLeft1]);
	LOG("GlobalAngles", "LimbCombinator LHipPitch", walkingEngineOutput.angles[JointData::legLeft2]);
	LOG("GlobalAngles", "LimbCombinator LKneePitch", walkingEngineOutput.angles[JointData::legLeft3]);
	LOG("GlobalAngles", "LimbCombinator LAnklePitch", walkingEngineOutput.angles[JointData::legLeft4]);
	LOG("GlobalAngles", "LimbCombinator LAnkleRoll", walkingEngineOutput.angles[JointData::legLeft5]);

	LOG("GlobalAngles", "LimbCombinator RHipYawPitch", walkingEngineOutput.angles[JointData::legRight0]);
	LOG("GlobalAngles", "LimbCombinator RHipRoll", walkingEngineOutput.angles[JointData::legRight1]);
	LOG("GlobalAngles", "LimbCombinator RHipPitch", walkingEngineOutput.angles[JointData::legRight2]);
	LOG("GlobalAngles", "LimbCombinator RKneePitch", walkingEngineOutput.angles[JointData::legRight3]);
	LOG("GlobalAngles", "LimbCombinator RAnklePitch", walkingEngineOutput.angles[JointData::legRight4]);
	LOG("GlobalAngles", "LimbCombinator RAnkleRoll", walkingEngineOutput.angles[JointData::legRight5]);

	MARK("GlobalAngles", "Ego-CoM x");
	MARK("GlobalAngles", "Ego-CoM y");
	MARK("GlobalAngles", "Ego-CoM z");
#pragma endregion

#pragma region CombineAngles
	for (int i=0; i<JointData::numOfJoints; i++)
		walkingEngineOutput.angles[i]=filter[i].nextValue(theKinematicOutput.angles[i]);

	if (theArmMovement.usearms)
	{
		walkingEngineOutput.angles[JointData::armLeft0]=theArmMovement.angles[JointData::armLeft0];
		walkingEngineOutput.angles[JointData::armLeft1]=theArmMovement.angles[JointData::armLeft1];
		walkingEngineOutput.angles[JointData::armLeft2]=theArmMovement.angles[JointData::armLeft2];
		walkingEngineOutput.angles[JointData::armLeft3]=theArmMovement.angles[JointData::armLeft3];

		walkingEngineOutput.angles[JointData::armRight0]=theArmMovement.angles[JointData::armRight0];
		walkingEngineOutput.angles[JointData::armRight1]=theArmMovement.angles[JointData::armRight1];
		walkingEngineOutput.angles[JointData::armRight2]=theArmMovement.angles[JointData::armRight2];
		walkingEngineOutput.angles[JointData::armRight3]=theArmMovement.angles[JointData::armRight3];
	}
	else
	{
		walkingEngineOutput.angles[JointData::armLeft0]=JointData::ignore;
		walkingEngineOutput.angles[JointData::armLeft1]=JointData::ignore;
		walkingEngineOutput.angles[JointData::armLeft2]=JointData::ignore;
		walkingEngineOutput.angles[JointData::armLeft3]=JointData::ignore;

		walkingEngineOutput.angles[JointData::armRight0]=JointData::ignore;
		walkingEngineOutput.angles[JointData::armRight1]=JointData::ignore;
		walkingEngineOutput.angles[JointData::armRight2]=JointData::ignore;
		walkingEngineOutput.angles[JointData::armRight3]=JointData::ignore;
	}
#pragma endregion

	
	walkingEngineOutput.odometryOffset=theWalkingInfo.odometryOffset;
	walkingEngineOutput.isLeavingPossible=theWalkingInfo.isLeavingPossible;
	walkingEngineOutput.speed.translation.x=theSpeedInfo.speed.x*1000;
	walkingEngineOutput.speed.translation.y=theSpeedInfo.speed.y*1000;
	walkingEngineOutput.speed.rotation=theSpeedInfo.speed.r;
  
	walkingEngineOutput.offsetToRobotPoseAfterPreview = theWalkingInfo.offsetToRobotPoseAfterPreview;
//set leg hardness
	//if (theWalkingInfo.kickPhase==freeLegNA)
	if (theMotionRequest.standType==doubleSupport)
	{
		walkingEngineOutput.jointHardness.hardness[JointData::legLeft0]=theWalkingEngineParams.legJointHardness[0];
		walkingEngineOutput.jointHardness.hardness[JointData::legLeft1]=theWalkingEngineParams.legJointHardness[1];
		walkingEngineOutput.jointHardness.hardness[JointData::legLeft2]=theWalkingEngineParams.legJointHardness[2];
		walkingEngineOutput.jointHardness.hardness[JointData::legLeft3]=theWalkingEngineParams.legJointHardness[3];
		walkingEngineOutput.jointHardness.hardness[JointData::legLeft4]=theWalkingEngineParams.legJointHardness[4];
		walkingEngineOutput.jointHardness.hardness[JointData::legLeft5]=theWalkingEngineParams.legJointHardness[5];
		walkingEngineOutput.jointHardness.hardness[JointData::legRight0]=theWalkingEngineParams.legJointHardness[0];
		walkingEngineOutput.jointHardness.hardness[JointData::legRight1]=theWalkingEngineParams.legJointHardness[1];
		walkingEngineOutput.jointHardness.hardness[JointData::legRight2]=theWalkingEngineParams.legJointHardness[2];
		walkingEngineOutput.jointHardness.hardness[JointData::legRight3]=theWalkingEngineParams.legJointHardness[3];
		walkingEngineOutput.jointHardness.hardness[JointData::legRight4]=theWalkingEngineParams.legJointHardness[4];
		walkingEngineOutput.jointHardness.hardness[JointData::legRight5]=theWalkingEngineParams.legJointHardness[5];
	}
	else
	{
		walkingEngineOutput.jointHardness.hardness[JointData::legLeft0]=theFreeLegPhaseParams.legJointHardness[0];
		walkingEngineOutput.jointHardness.hardness[JointData::legLeft1]=theFreeLegPhaseParams.legJointHardness[1];
		walkingEngineOutput.jointHardness.hardness[JointData::legLeft2]=theFreeLegPhaseParams.legJointHardness[2];
		walkingEngineOutput.jointHardness.hardness[JointData::legLeft3]=theFreeLegPhaseParams.legJointHardness[3];
		walkingEngineOutput.jointHardness.hardness[JointData::legLeft4]=theFreeLegPhaseParams.legJointHardness[4];
		walkingEngineOutput.jointHardness.hardness[JointData::legLeft5]=theFreeLegPhaseParams.legJointHardness[5];
		walkingEngineOutput.jointHardness.hardness[JointData::legRight0]=theFreeLegPhaseParams.legJointHardness[0];
		walkingEngineOutput.jointHardness.hardness[JointData::legRight1]=theFreeLegPhaseParams.legJointHardness[1];
		walkingEngineOutput.jointHardness.hardness[JointData::legRight2]=theFreeLegPhaseParams.legJointHardness[2];
		walkingEngineOutput.jointHardness.hardness[JointData::legRight3]=theFreeLegPhaseParams.legJointHardness[3];
		walkingEngineOutput.jointHardness.hardness[JointData::legRight4]=theFreeLegPhaseParams.legJointHardness[4];
		walkingEngineOutput.jointHardness.hardness[JointData::legRight5]=theFreeLegPhaseParams.legJointHardness[5];
	}
}

MAKE_MODULE(LimbCombinator, Dortmund WalkingEngine)
