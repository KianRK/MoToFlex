/** 
* @file Modules/MotionControl/DortmundWalkingEngine/PatternGenerator.h
* Generator for foot steps
* @author <a href="mailto:ciVic@web.de> Oliver Urbann</a>
*/

#pragma once

#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Representations/MotionControl/PatternGenRequest.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/MotionControl/FootSteps.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Modeling/FallDownState.h"
#include "Representations/MotionControl/ControllerParams.h"
#include "Tools/Module/Module.h"
#include "PatternGenerator.h"
//#include "Tools/Debugging/DebugDrawings.h"

MODULE(PatternGeneratorModule)
  REQUIRES(WalkingEngineParams)
  REQUIRES(PatternGenRequest)
  REQUIRES(RobotModel)
  REQUIRES(RobotDimensions)
  REQUIRES(FallDownState)
  REQUIRES(ControllerParams)
  REQUIRES(MotionRequest)
  USES(WalkingInfo)
  PROVIDES_WITH_MODIFY(FootSteps)
END_MODULE

class PatternGeneratorModule : public PatternGeneratorModuleBase
{
public:
  PatternGeneratorModule();
private:
	PatternGenerator patternGen;

	void update(FootSteps& steps);
};
