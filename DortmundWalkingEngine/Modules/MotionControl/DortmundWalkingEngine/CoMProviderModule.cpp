/** 
* @file Modules/MotionControl/DortmundWalkingEngine/CoMProviderModule.cpp
* Module wrapper for the CoMProvider
* @author <a href="mailto:ciVic@web.de> Oliver Urbann</a>
*/

#include "CoMProviderModule.h"
#include "Tools/Debugging/DebugDrawings.h"

CoMProviderModule::CoMProviderModule():
controller(
		theJointData,
		theWalkingEngineParams,
		theJointRequest,
		theFootSteps,
		theRobotModel)
{
}

void CoMProviderModule::update(ActualCoM &theActualCoM)
{
	controller.updateActualCoM(theActualCoM);

	
	PLOT("module:CoMProvider:ActualCoM.x", theActualCoM.x);
	PLOT("module:CoMProvider:ActualCoM.y", theActualCoM.y);
};


MAKE_MODULE(CoMProviderModule, Dortmund WalkingEngine)