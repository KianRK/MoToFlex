/** 
* @file Modules/MotionControl/DortmundWalkingEngine/SwingLegControllerModule.h
* Module wrapper for the ZMPGenerator
* @author <a href="mailto:ciVic@web.de> Oliver Urbann</a>
*/

#pragma once

#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/WalkingInfo.h"
#include "Tools/Module/Module.h"
#include "Representations/MotionControl/Footpositions.h"
#include "Representations/MotionControl/FootSteps.h"
#include "SwingLegController.h"
#include "Representations/Sensing/ZMPModel.h"
#include "Representations/Modeling/FallDownState.h"

MODULE(SwingLegControllerModule)
	REQUIRES(WalkingEngineParams)
	REQUIRES(FreeLegPhaseParams)
	REQUIRES(FootSteps)
	REQUIRES(MotionRequest)
	REQUIRES(BallModel)
	REQUIRES(ZMPModel)
	REQUIRES(FallDownState)
	USES(WalkingInfo)
	PROVIDES(Footpositions)
END_MODULE

class SwingLegControllerModule : public SwingLegControllerModuleBase
{
public:
	SwingLegControllerModule();

	SwingLegController controller;

	void update(Footpositions &footpositions);
};


