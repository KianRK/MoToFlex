#pragma once

#include "Tools/Module/Module.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/MotionControl/KinematicOutput.h"
#include "Representations/MotionControl/ArmMovement.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/SpeedInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Tools/Math/Filter/FastFilter.h"
#include "Representations/Infrastructure/JointData.h"

MODULE(LimbCombinator)
	REQUIRES(SpeedInfo)	
	REQUIRES(KinematicOutput)
	REQUIRES(ArmMovement)
	REQUIRES(JointData)
	REQUIRES(WalkingInfo)
	REQUIRES(MotionRequest)
	REQUIRES(WalkingEngineParams)
	REQUIRES(FreeLegPhaseParams)
	PROVIDES_WITH_MODIFY(WalkingEngineOutput)
END_MODULE

class LimbCombinator : public LimbCombinatorBase
{
public:
	LimbCombinator(void);
	~LimbCombinator(void); 
private:

	void update(WalkingEngineOutput& walkingEngineOutput);
	bool init;
	FastFilter<double> filter[JointData::numOfJoints];
};
