/** 
* @file Modules/MotionControl/DortmundWalkingEngine/PatternGenerator.cpp
* Module wrapper for PatternGenerator
* @author <a href="mailto:ciVic@web.de> Oliver Urbann</a>
*/

#include "PatternGeneratorModule.h"


PatternGeneratorModule::PatternGeneratorModule():
  patternGen(
	  theWalkingEngineParams,
	  thePatternGenRequest,
	  theRobotModel,
	  theRobotDimensions,
	  theFallDownState,
	  theControllerParams,
	  theMotionRequest,
	  theWalkingInfo)
{
}

void PatternGeneratorModule::update(FootSteps& steps)
{
	patternGen.updateFootSteps(
    steps
		);
}

MAKE_MODULE(PatternGeneratorModule, Dortmund WalkingEngine) 
