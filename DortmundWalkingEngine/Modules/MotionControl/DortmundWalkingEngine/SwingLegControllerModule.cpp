/** 
* @file Modules/MotionControl/DortmundWalkingEngine/SwingLegControllerModule.cpp
* Module wrapper for the SwingLegController
* @author <a href="mailto:ciVic@web.de> Oliver Urbann</a>
*/

#include "SwingLegControllerModule.h"

SwingLegControllerModule::SwingLegControllerModule():
controller(
		theWalkingEngineParams,
		theFootSteps,
    theBallModel,
		theMotionRequest,
		theWalkingInfo,
		theFreeLegPhaseParams,
		theFallDownState,
		theZMPModel)
{
}

void SwingLegControllerModule::update(Footpositions &footpositions)
{
	controller.updateFootpositions(footpositions);
};

MAKE_MODULE(SwingLegControllerModule, Dortmund WalkingEngine)