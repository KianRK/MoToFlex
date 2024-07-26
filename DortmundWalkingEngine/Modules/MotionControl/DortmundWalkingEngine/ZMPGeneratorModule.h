/** 
* @file Modules/MotionControl/DortmundWalkingEngine/ZMPGeneratorModule.h
* Module wrapper for the ZMPGenerator
* @author <a href="mailto:ciVic@web.de> Oliver Urbann</a>
*/

#pragma once

#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Tools/Module/Module.h"
#include "Representations/MotionControl/ControllerParams.h"
#include "Representations/MotionControl/RefZMP.h"
#include "Representations/MotionControl/FootSteps.h"
#include "ZMPGenerator.h"
#include "Representations/MotionControl/WalkingInfo.h"

MODULE(ZMPGeneratorModule)
	REQUIRES(FootSteps)
	USES(WalkingInfo)
	REQUIRES(WalkingEngineParams)
	REQUIRES(FreeLegPhaseParams)
	REQUIRES(ControllerParams)
	PROVIDES(RefZMP)
END_MODULE

class ZMPGeneratorModule : public ZMPGeneratorModuleBase
{
public:
  ZMPGeneratorModule();

	ZMPGenerator generator;

	void update(RefZMP &theRefZMP);
};


