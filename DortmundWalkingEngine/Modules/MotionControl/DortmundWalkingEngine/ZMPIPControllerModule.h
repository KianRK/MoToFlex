/** 
* @file Modules/MotionControl/DortmundWalkingEngine/ZMPIPControllerModule.h
* Module wrapper for the ZMP/IP-Controller
* @author <a href="mailto:ciVic@web.de> Oliver Urbann</a>
*/

#pragma once

#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/Infrastructure/SensorData.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/Modeling/FallDownState.h"
#include "Representations/Sensing/RobotModel.h"
#include "Tools/Module/Module.h"
#include "Representations/MotionControl/ControllerParams.h"
#include "Representations/MotionControl/TargetCoM.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Representations/MotionControl/RefZMP.h"
#include "Representations/Sensing/ZMPModel.h"
#include "ZMPIPController.h"
#include "Representations/MotionControl/PatternGenRequest.h"

MODULE(ZMPIPControllerModule)
	REQUIRES(RobotModel)
	REQUIRES(SensorData)
	REQUIRES(WalkingEngineParams)
	REQUIRES(FallDownState)
	REQUIRES(PatternGenRequest)
	REQUIRES(ControllerParams)
	REQUIRES(ZMPModel)
	REQUIRES(RefZMP)
	USES(WalkingInfo)
	PROVIDES_WITH_MODIFY(TargetCoM)
END_MODULE

class ZMPIPControllerModule : public ZMPIPControllerModuleBase 
{
public:
  ZMPIPControllerModule();

	ZMPIPController controller;

	void update(TargetCoM &theTargetCoM);
};


