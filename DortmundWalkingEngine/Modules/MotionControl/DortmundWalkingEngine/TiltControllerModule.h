/** 
* @file Modules/MotionControl/DortmundWalkingEngine/TiltControllerModule.h
* Module wrapper for the TiltController
* @author <a href="mailto:ciVic@web.de> Oliver Urbann</a>
*/

#pragma once

#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Tools/Module/Module.h"
#include "TiltController.h"
#include "Representations/Infrastructure/SensorData.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/MotionControl/BodyTilt.h"
#include "Representations/Configuration/JointCalibration.h"

MODULE(TiltControllerModule)
	REQUIRES(SensorData)
	USES(JointRequest)
	REQUIRES(JointCalibration)
	REQUIRES(WalkingEngineParams)
	PROVIDES_WITH_MODIFY(BodyTilt)
END_MODULE

class TiltControllerModule : public TiltControllerModuleBase
{
public:
	TiltControllerModule();

	TiltController controller;

	void update(BodyTilt &theBodyTilt);
};


