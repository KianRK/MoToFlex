/** 
* @file Modules/MotionControl/DortmundWalkingEngine/CoMProviderModule.h
* Module wrapper for the CoMProvider
* @author <a href="mailto:ciVic@web.de> Oliver Urbann</a>
*/

#pragma once

#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/FootSteps.h"
#include "Representations/Sensing/RobotModel.h"
#include "Tools/Module/Module.h"
#include "CoMProvider.h"
#include "Representations/Infrastructure/JointData.h"
#include "Representations/MotionControl/ActualCoM.h"

MODULE(CoMProviderModule)
	REQUIRES(JointData)
	REQUIRES(FootSteps)
	REQUIRES(RobotModel)
	USES(JointRequest)
	REQUIRES(WalkingEngineParams)
	PROVIDES_WITH_MODIFY(ActualCoM)
END_MODULE

class CoMProviderModule : public CoMProviderModuleBase
{
public:
	CoMProviderModule();

	CoMProvider controller;

	void update(ActualCoM &theActualCoM);
};


