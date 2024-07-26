#pragma once

#include "Tools/Module/Module.h"
#include "Representations/MotionControl/ArmMovement.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/Infrastructure/SensorData.h"

MODULE(OrientationArmAnimator)
	REQUIRES(SensorData)
	REQUIRES(WalkingEngineParams)
	PROVIDES_WITH_MODIFY(ArmMovement)
END_MODULE

class OrientationArmAnimator : public OrientationArmAnimatorBase
{
public:
	OrientationArmAnimator(void);
	~OrientationArmAnimator(void);
private:
	void update(ArmMovement& armMovement);
};
