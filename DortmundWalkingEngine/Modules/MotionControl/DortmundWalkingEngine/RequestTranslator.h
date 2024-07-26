/** 
* @file Modules/MotionControl/DortmundWalkingEngine/RequestTranslator.h
* Translates the MotionRequest for the WalkingEngine (some clipping and path-calculations)
* @author <a href="mailto:ciVic@web.de> Oliver Urbann</a>
* @author <a href="mailto:stefan.czarnetzki@uni-dortmund.de">Stefan Czarnetzki</a>
*/

#pragma once

#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Modeling/Path.h"
#include "Representations/MotionControl/ControllerParams.h"
#include "Representations/MotionControl/MotionSelection.h"
#include "Representations/MotionControl/PatternGenRequest.h"
#include "Representations/MotionControl/WalkingEngineParams.h"
#include "Tools/Module/Module.h"
#include "Tools/Math/Filter/FastFilter.h"
#include "Tools/Math/Vector3.h"
#include "Point.h"
#include "Representations/MotionControl/SpeedInfo.h"
#include "Tools/RingBuffer.h"
#include "Representations/MotionControl/WalkingInfo.h"

class GoToParameters : public Streamable
{
public:
  GoToParameters();

  double omniAreaThreshold_x_front,
         omniAreaThreshold_x_back,
         omniAreaThreshold_y,
         rotationOnlyThreshold_angle,
         omniThreshold_angle,
         fractionOfMaxRotForFastWalking,
         fractionOfMaxRotForOmniWalking,
         fractionOfMaxForwardForOmniWalking,
         fractionOfMaxSidewaysForOmniWalking,
         omniOnlyObstacleDistance,
         shortcutDistance;
         

  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN();
    STREAM( omniAreaThreshold_x_front );
    STREAM( omniAreaThreshold_x_back );
    STREAM( omniAreaThreshold_y );
    STREAM( rotationOnlyThreshold_angle );
    STREAM( omniThreshold_angle );
    STREAM( fractionOfMaxRotForFastWalking );
    STREAM( fractionOfMaxRotForOmniWalking );
    STREAM( fractionOfMaxForwardForOmniWalking );
    STREAM( fractionOfMaxSidewaysForOmniWalking );
    STREAM( omniOnlyObstacleDistance );
    STREAM( shortcutDistance );
    STREAM_REGISTER_FINISH();
  }

};


class RequestTranslator
{
public:
	RequestTranslator(const MotionSelection &theMotionSelection, const Path &thePath, const WalkingInfo &theWalkingInfo);
	~RequestTranslator(void);
	
	void updatePatternGenRequest(PatternGenRequest & patternGenRequest);
	void updateWalkingEngineParams(WalkingEngineParams & walkingEngineParams);
	void updateControllerParams(ControllerParams & controllerParams);
	void updateSpeedInfo(SpeedInfo & speedInfo);
	void updateFreeLegPhaseParams(FreeLegPhaseParams & freeLegPhaseParams);
private:
  GoToParameters goToParameters;
	const MotionSelection & theMotionSelection;
	const Path &thePath;
	const WalkingInfo &theWalkingInfo;
	WalkingEngineParams walkParams;
	FreeLegPhaseParams flpParams;
	ControllerParams contParams;
	int load(const char *path);
	PatternGenRequest::State getNewState();
	FastFilter<double> filter[3];
	PatternGenRequest::State currentState;
	int accCounter[3], lastN;
	double last[3];
	float localSensorScale;
	DynamicRingBuffer<Point> speedBuffer;
  
  // for goto with obstacle avoidance
  enum GotoMode {
    rotationOnly,
    forwardAndRotate,
    omniDirectional,
    numOfGotoModes
  };
  GotoMode gotoMode;
  bool isRotInOmni;
  
  // general goto stuff

  Pose2D getDesiredSpeedForNextStep(const Pose2D & destination);
  Pose2D getDesiredSpeedForNextStepWhileAvoidingObstacles(const bool &forceOmniOnly);
  
 //* for omni-only goto; if no speed limit is desired, set maxSpeed >= maxSpeedOmniForward */
  Pose2D getOmniSpeedForNextStep(const Pose2D & destination, const double maxSpeed);
  
  void drawPath();
  
  void drawPathWithSpeed(const Pose2D & speed);

};
