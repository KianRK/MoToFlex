#pragma once

#include "Tools/Module/Module.h"
#include "Representations/MotionControl/ArmMovement.h"
#include "Representations/MotionControl/KinematicRequest.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/BodyTilt.h"

MODULE(ArmAnimator)
	REQUIRES(WalkingEngineParams)
	REQUIRES(KinematicRequest)
	REQUIRES(WalkingInfo)
	REQUIRES(BodyTilt)
	PROVIDES_WITH_MODIFY(ArmMovement)
END_MODULE

class ArmAnimator : public ArmAnimatorBase
{
public:
	ArmAnimator(void);
	~ArmAnimator(void);
private:
	void update(ArmMovement& armMovement);
};
