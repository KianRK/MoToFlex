/** 
* @file Modules/MotionControl/DortmundWalkingEngine/RequestTranslator.cpp
* Translates the MotionRequest for the WalkingEngine (some clipping and path-calculations)
* @author <a href="mailto:ciVic@web.de> Oliver Urbann</a>
* @author <a href="mailto:stefan.czarnetzki@uni-dortmund.de">Stefan Czarnetzki</a>
*/

#include "RequestTranslator.h"
#include "Tools/Streams/InStreams.h"
#include "Platform/SystemCall.h"
#include "Tools/Settings.h"
#include <string>

#include "Tools/Debugging/Modify.h"
#include "Tools/Math/Geometry.h"

//#define LOGGING
#include "Tools/Debugging/CSVLogger.h"

using namespace std;

//#define PAUSE_AT_START


GoToParameters::GoToParameters()
{
  InConfigFile stream(Global::getSettings().expandRobotFilename("goto.cfg"), "default");
  if(stream.exists())
    stream >> *this;
  else
  {
    InConfigFile stream("goto.cfg", "default");
    ASSERT(stream.exists());
    stream >> *this;
  }
}


RequestTranslator::RequestTranslator(const MotionSelection &theMotionSelection, const Path &thePath, const WalkingInfo &theWalkingInfo):
  theMotionSelection(theMotionSelection),
  thePath(thePath),
  theWalkingInfo(theWalkingInfo),
  speedBuffer(1)
{
  InConfigFile file(Global::getSettings().expandRobotFilename("walkingParams.cfg"));
  if(file.exists())
    file >> walkParams;
  else
  {
    InConfigFile file("walkingParams.cfg");
    ASSERT(file.exists());
    file >> walkParams;
  }

  InConfigFile flpfile(Global::getSettings().expandRobotFilename("freeLegParams.cfg"));
  if(flpfile.exists())
    flpfile >> flpParams;
  else
  {
    InConfigFile flpfile("freeLegParams.cfg");
    ASSERT(flpfile.exists());
    flpfile >> flpParams;
  }

  lastN=0;

	string str_path=File::getGTDir()+string("/Config/")
		+Global::getSettings().expandRobotFilename("ZMPIPController.dat");

	load(str_path.c_str());

	accCounter[0]=accCounter[1]=accCounter[2]=0;
	last[0]=last[1]=last[2]=0;

	for (int i=0; i<3; i++)
		filter[i].createBuffer(walkParams.walkRequestFilterLen);

  gotoMode = forwardAndRotate;
  isRotInOmni = false;
}

PatternGenRequest::State RequestTranslator::getNewState()
{
	if (currentState==PatternGenRequest::emergencyStop)
		return PatternGenRequest::standby;  // Try to restart engine (maybe interrupted if robot 
											// is still not upright

	if (!(theMotionSelection.targetMotion==MotionRequest::walk &&
		theMotionSelection.ratios[MotionRequest::walk]==1) &&
		(currentState==PatternGenRequest::ready || currentState==PatternGenRequest::standby))
		return PatternGenRequest::standby;

	if (!(theMotionSelection.targetMotion==MotionRequest::walk &&
		theMotionSelection.ratios[MotionRequest::walk]==1) &&
		currentState==PatternGenRequest::walking)
		return PatternGenRequest::ready;

	if (theMotionSelection.targetMotion==MotionRequest::walk &&
		theMotionSelection.ratios[MotionRequest::walk]==1)
	{
		if (fabs(theMotionSelection.walkRequest.speed.translation.x)>0.001 ||
			fabs(theMotionSelection.walkRequest.speed.translation.y)>0.001 ||
			fabs(theMotionSelection.walkRequest.speed.rotation)>0.01 ||
			fabs(theMotionSelection.walkRequest.target.translation.x)>0.01 ||
			fabs(theMotionSelection.walkRequest.target.translation.y)>0.01 ||
			fabs(theMotionSelection.walkRequest.target.rotation)>0.01)
			return PatternGenRequest::walking;
		else
			return PatternGenRequest::ready;
	}

	return PatternGenRequest::NA;
}

int RequestTranslator::load(const char *path)
{
	FILE *stream;
	//int count=0;
	char tempstr[200], *stopstring;

	stream=fopen(path, "r");

	fscanf(stream, "%s", tempstr);
	contParams.z_h = strtod(tempstr, &stopstring);

	fscanf(stream, "%s", tempstr);
	contParams.dt = strtod(tempstr, &stopstring);

	fscanf(stream, "%s", tempstr);
	contParams.N = (unsigned int)strtod(tempstr, &stopstring);

	fscanf(stream, "%s", tempstr);
	contParams.L[0][0] = strtod(tempstr, &stopstring);
	fscanf(stream, "%s", tempstr);
	contParams.L[1][0] = strtod(tempstr, &stopstring);
	fscanf(stream, "%s", tempstr);
	contParams.L[2][0] = strtod(tempstr, &stopstring);

    fscanf(stream, "%s", tempstr);
	contParams.L[0][1] = strtod(tempstr, &stopstring);
	fscanf(stream, "%s", tempstr);
	contParams.L[1][1] = strtod(tempstr, &stopstring);
	fscanf(stream, "%s", tempstr);
	contParams.L[2][1] = strtod(tempstr, &stopstring);

	for (int row=0; row<3; row++)
	{
		for (int col=0; col<3; col++)
		{
			fscanf(stream, "%s", tempstr);
			contParams.A0[row][col]= strtod(tempstr, &stopstring);
		}
	}

	fscanf(stream, "%s", tempstr);
	contParams.Gi = strtod(tempstr, &stopstring);

	for (int i=0; i<3; i++)
	{
		fscanf(stream, "%s", tempstr);
		contParams.Gx[i] = strtod(tempstr, &stopstring);	
	}

	for (int i=0; i<3; i++)
	{
		fscanf(stream, "%s", tempstr);
		contParams.b0[i] = strtod(tempstr, &stopstring);	
	}

	for (int i=0; i<3; i++)
	{
		fscanf(stream, "%s", tempstr);
		contParams.c0[i] = strtod(tempstr, &stopstring);	
	}


	contParams.Gd = new double[contParams.N];

	for (int i=0; i<contParams.N; i++)
	{   
		fscanf(stream, "%s", tempstr);
		contParams.Gd[i] = strtod(tempstr, &stopstring);
	}


	fclose(stream);

	return 1;
}

RequestTranslator::~RequestTranslator(void)
{
}


void RequestTranslator::updatePatternGenRequest(PatternGenRequest & patternGenRequest)
{
  DECLARE_DEBUG_DRAWING( "module:RequestTranslator:Path", "drawingOnField"); // should be displayed relative to RobotPoseAfterPreview

  MODIFY("module:RequestTranslator:GoToParameters", goToParameters);

  if (theMotionSelection.walkRequest.target.translation.x==0 && theMotionSelection.walkRequest.target.translation.y==0 && theMotionSelection.walkRequest.target.rotation==0)
  {
	  patternGenRequest.speed.translation.x=theMotionSelection.walkRequest.speed.translation.x;
	  patternGenRequest.speed.translation.y=theMotionSelection.walkRequest.speed.translation.y;
	  patternGenRequest.speed.rotation=theMotionSelection.walkRequest.speed.rotation;
    COMPLEX_DRAWING("module:RequestTranslator:Path",drawPathWithSpeed(patternGenRequest.speed));
  }
  else
  {
    if (theMotionSelection.walkRequest.target.translation.abs() < 150)
      patternGenRequest.speed = getDesiredSpeedForNextStep(theMotionSelection.walkRequest.target);
    else
      patternGenRequest.speed = getDesiredSpeedForNextStepWhileAvoidingObstacles(theMotionSelection.walkRequest.forceOmniOnly);
    COMPLEX_DRAWING("module:RequestTranslator:Path",drawPath());
  }
	patternGenRequest.pitch=0;

  PLOT("module:RequestTranslator:speed_before_clipping.x",patternGenRequest.speed.translation.x);
  PLOT("module:RequestTranslator:speed_before_clipping.y",patternGenRequest.speed.translation.y);
  PLOT("module:RequestTranslator:speed_before_clipping.rot",patternGenRequest.speed.rotation);

#pragma region Valuecheck // Check for impossible values
	if (patternGenRequest.speed.translation.x!=0 ||
		patternGenRequest.speed.translation.y!=0 ||
		patternGenRequest.speed.rotation!=0)
	{
		if (patternGenRequest.pitch>walkParams.maxWalkPitch)
			patternGenRequest.pitch=walkParams.maxWalkPitch;
	}
	else
	{
		if (patternGenRequest.pitch>walkParams.maxStandPitch)
			patternGenRequest.pitch=walkParams.maxStandPitch;
	}

	if (patternGenRequest.speed.translation.x>walkParams.maxSpeedXForward)
		patternGenRequest.speed.translation.x=walkParams.maxSpeedXForward;

	if (patternGenRequest.speed.translation.x<-walkParams.maxSpeedXBack)
		patternGenRequest.speed.translation.x=-walkParams.maxSpeedXBack;

	if (patternGenRequest.speed.translation.y>walkParams.maxSpeedYLeft)
		patternGenRequest.speed.translation.y=walkParams.maxSpeedYLeft;

	if (patternGenRequest.speed.translation.y<-walkParams.maxSpeedYRight)
		patternGenRequest.speed.translation.y=-walkParams.maxSpeedYRight;

	if (patternGenRequest.speed.rotation<-walkParams.maxSpeedR)
		patternGenRequest.speed.rotation=-walkParams.maxSpeedR;

	if (patternGenRequest.speed.rotation>walkParams.maxSpeedR)
		patternGenRequest.speed.rotation=walkParams.maxSpeedR;

#pragma endregion


	patternGenRequest.speed.translation.x/=1000;
	patternGenRequest.speed.translation.y/=1000;


	LOG("MotionRequests", "x",patternGenRequest.speed.translation.x);
	LOG("MotionRequests", "y",patternGenRequest.speed.translation.y);
	LOG("MotionRequests", "r",patternGenRequest.speed.rotation);

#ifdef PAUSE_AT_START
	if ((last[0]==0 && patternGenRequest.x!=0) ||
		(last[1]==0 && patternGenRequest.y!=0) ||
		(last[2]==0 && patternGenRequest.r!=0))
	{
		last[0]=patternGenRequest.x=0.001;
		last[1]=patternGenRequest.y=0;
		last[2]=patternGenRequest.r=0;
		accCounter[0]=accCounter[1]=accCounter[2]=50;
	}
#endif

#pragma region acceleration
	int rsign=1, ysign=1;
	if (patternGenRequest.speed.translation.x<last[0])
	{
		last[0] = patternGenRequest.speed.translation.x;
		accCounter[0] = 0;
	}
	else if (accCounter[0]==0 && patternGenRequest.speed.translation.x-last[0]>walkParams.maxAccX)
	{
		last[0]+=walkParams.maxAccX;		
		accCounter[0]=walkParams.accDelayX;		
	}
	if (accCounter[0]>0)
	{
		accCounter[0]--;
		patternGenRequest.speed.translation.x=last[0];
	}
	else
		last[0]=patternGenRequest.speed.translation.x;


	// check and set speed for y
	if (fabs(patternGenRequest.speed.translation.y)<fabs(last[1]))
	{
		last[1] = patternGenRequest.speed.translation.y;
		accCounter[1] = walkParams.accDelayY;
	}
	else if  (last[1]*patternGenRequest.speed.translation.y<0)
	{
	  	last[1] = 0;
		accCounter[1] = walkParams.accDelayY;
	}
	else if (accCounter[1]==0 && fabs(patternGenRequest.speed.translation.y-last[1])>walkParams.maxAccY)
	{	
		if (patternGenRequest.speed.translation.y-last[1]<0) ysign=-1;
		last[1]+=ysign*walkParams.maxAccY;
		accCounter[1]=walkParams.accDelayY;		
	}

	if (accCounter[1]>0)
	{
		accCounter[1]--;
		patternGenRequest.speed.translation.y=last[1];
	}
	else
		last[1]=patternGenRequest.speed.translation.y;


	// check and set rotational speed
	if (fabs(patternGenRequest.speed.translation.y)<fabs(last[2]))
	{
		last[2] = patternGenRequest.speed.rotation;
		accCounter[2] = walkParams.accDelayR;
	}
	else if  (last[2]*patternGenRequest.speed.rotation<0)
	{
	  	last[2] = 0;
		accCounter[2] = walkParams.accDelayR;
	}
	else if (accCounter[2]==0 && fabs(patternGenRequest.speed.rotation-last[2])>walkParams.maxAccR)
	{	
		if (patternGenRequest.speed.rotation-last[2]<0) rsign=-1;
		last[2]+=rsign*walkParams.maxAccR;
		accCounter[2]=walkParams.accDelayR;		
	}
	if (accCounter[2]>0)
	{
		accCounter[2]--;
		patternGenRequest.speed.rotation=last[2];
	}
	else
		last[2]=patternGenRequest.speed.rotation;
#pragma endregion


	patternGenRequest.speed.translation.x=filter[0].nextValue(patternGenRequest.speed.translation.x);
	patternGenRequest.speed.translation.y=filter[1].nextValue(patternGenRequest.speed.translation.y);
	patternGenRequest.speed.rotation=filter[2].nextValue(patternGenRequest.speed.rotation);
												  
	patternGenRequest.newState=getNewState();

	
	currentState=patternGenRequest.newState;

	if (lastN!=contParams.N)
		speedBuffer.init(contParams.N);
	lastN=contParams.N;

	if (contParams.N!=0)
	{	
		Point speed;
		speed.x=patternGenRequest.speed.translation.x;
		speed.y=patternGenRequest.speed.translation.y;
		speed.r=patternGenRequest.speed.rotation;
		speedBuffer.add(speed);
	}
  
  PLOT("module:RequestTranslator:speed_after_clipping.x",patternGenRequest.speed.translation.x);
  PLOT("module:RequestTranslator:speed_after_clipping.y",patternGenRequest.speed.translation.y);
  PLOT("module:RequestTranslator:speed_after_clipping.rot",patternGenRequest.speed.rotation);

}

void RequestTranslator::updateWalkingEngineParams(WalkingEngineParams & walkingEngineParams)
{
  MODIFY("representation:WalkingEngineParams", walkParams);
  if (theWalkingInfo.kickPhase!=freeLegNA)
	  walkingEngineParams=flpParams;
  else
	walkingEngineParams=walkParams;
}

void RequestTranslator::updateFreeLegPhaseParams(FreeLegPhaseParams & freeLegPhaseParams)
{
  MODIFY("representation:FreeLegPhaseParams", flpParams);
	freeLegPhaseParams=flpParams;
}

void RequestTranslator::updateControllerParams(ControllerParams & controllerParams)
{
	controllerParams = contParams;
}

void RequestTranslator::updateSpeedInfo(SpeedInfo & speedInfo)
{
	if (contParams.N!=0 && speedBuffer.getNumberOfEntries()>=contParams.N-1)
		speedInfo.speed=speedBuffer.getEntry(contParams.N-1);
}


Pose2D RequestTranslator::getDesiredSpeedForNextStepWhileAvoidingObstacles(const bool &forceOmniOnly)
{
	// Input:   destination in milimeter and rad, 
  //          maxAbsoluteSpeeds (from walkParams!)
  // Output:  speed in mm/s and rad/s

	Pose2D speedResult;
  const Pose2D robotPose = thePath.wayPoints.front();
  const Pose2D destination = thePath.wayPoints.back()-robotPose;

  const double destDistance = destination.translation.abs();

  double timeForOneStep = walkParams.stepDuration / 2;

  const double & fractionOfMaxRotForOmniWalking = goToParameters.fractionOfMaxRotForOmniWalking;
  const double & fractionOfMaxForwardForOmniWalking = goToParameters.fractionOfMaxForwardForOmniWalking;
  const double & fractionOfMaxSidewaysForOmniWalking = goToParameters.fractionOfMaxSidewaysForOmniWalking;
  
  const Pose2D destWorldCoordinates = thePath.wayPoints.back();

  short shortCutWayPoint = 0;
  double shortCutLength = 0.0;
  const double shortcutMaximumLength = goToParameters.shortcutDistance;
  bool isFutureRotSimilar = false;
  const double angleFirstRelative = thePath.wayPoints.at(1).rotation-robotPose.rotation;
  const double angleRotSmooth = thePath.wayPoints.size() > 2 ? 
    (angleFirstRelative + thePath.wayPoints.at(2).rotation - robotPose.rotation)/2.0 : (angleFirstRelative-robotPose.rotation)/2.0;

  for (unsigned int i = 1; i < thePath.wayPoints.size(); i++)
  {
    if (i > 1)
      if (i < thePath.wayPoints.size() - 1)
        isFutureRotSimilar = fabs(thePath.wayPoints.at(i).rotation - robotPose.rotation) < goToParameters.omniThreshold_angle;
      else if (destDistance < 400)
        isFutureRotSimilar = fabs(thePath.wayPoints.at(i).rotation - robotPose.rotation) < goToParameters.omniThreshold_angle;
    if (i > 2)
    {
      double newShortCutLength = (thePath.wayPoints.at(i).translation-robotPose.translation).abs();
      if (newShortCutLength < shortcutMaximumLength && newShortCutLength > shortCutLength)
        shortCutWayPoint = i;
    }
  }

  Vector2<double> shortcut(1000,0);
  if (shortCutWayPoint > 0)
    shortcut = Pose2D(-robotPose.getAngle()).translate(thePath.wayPoints.at(shortCutWayPoint).translation - robotPose.translation).translation;
  
  // near Target and near obstacle, or if forced, always use omni mode
  if (forceOmniOnly || shortcut.abs() < shortcutMaximumLength || 
        thePath.nearestObstacle < goToParameters.omniOnlyObstacleDistance || 
        destDistance < 400)
    gotoMode = omniDirectional;
  // angle to next wayPoint to high -> use omni or rotation
  else if (fabs(angleFirstRelative) > goToParameters.rotationOnlyThreshold_angle)
  {
    if (isFutureRotSimilar)
      gotoMode = omniDirectional;
    else gotoMode = rotationOnly;
  }
  // else use forward or, if current rotation is similar to rotation in the future and angle to big, omni
  else
  {
    if (fabs(angleFirstRelative) > goToParameters.omniThreshold_angle && (isFutureRotSimilar))
      gotoMode = omniDirectional;
    else gotoMode = forwardAndRotate;
  }

  // use new gotoMode to move along the path
  if (gotoMode == forwardAndRotate)
  {
    // try to smooth rotation speed here
    speedResult.translation.x = walkParams.maxSpeedXForward;
    speedResult.translation.y = 0.0;
    speedResult.rotation = normalize(angleRotSmooth);
  }
  else if (gotoMode == omniDirectional)
  {
    // if next thePath. are close together, take a shortcut
    if (shortcut.abs() < shortcutMaximumLength)
    {
      // this would be the speed necessary to reach the destination in one step:
      speedResult.translation = shortcut;
      if (destDistance > 600) speedResult.rotation = 0.0;
      else speedResult.rotation = normalize(destination.rotation);

      if (destination.translation.y > 0 && speedResult.rotation < 0) speedResult.rotation = speedResult.rotation + 2*pi;
      else if (destination.translation.y < 0 && speedResult.rotation > 0) speedResult.rotation = speedResult.rotation - 2*pi;
      speedResult.translation.x /= timeForOneStep;
      speedResult.translation.y /= timeForOneStep;
      speedResult.rotation /= timeForOneStep;

      // this is probably more than can be done in one step, so cut it down
      double factor_x = speedResult.translation.x>0 ? fabs(speedResult.translation.x/(walkParams.maxSpeedXForward*fractionOfMaxForwardForOmniWalking))
                                              : fabs(speedResult.translation.x/walkParams.maxSpeedXBack);
      double factor_y = speedResult.translation.y>0 ? fabs(speedResult.translation.y/walkParams.maxSpeedYLeft*fractionOfMaxSidewaysForOmniWalking)
                                              : fabs(speedResult.translation.y/walkParams.maxSpeedYRight*fractionOfMaxSidewaysForOmniWalking);
      double factor_r = fabs(speedResult.rotation/(walkParams.maxSpeedR*fractionOfMaxRotForOmniWalking));

      double factor = max(factor_x, max(factor_y, factor_r));
      
      if (factor>1)
      {
        speedResult.translation.x /= factor;
        speedResult.translation.y /= factor;
        speedResult.rotation /= factor;
      }
    }
    else
    {
      // this would be the speed necessary to reach the destination in one step:
      speedResult.translation = Geometry::fieldCoord2Relative(robotPose,thePath.wayPoints.at(1).translation);
      if (destDistance > 600) speedResult.rotation = 0.0;
      else speedResult.rotation = normalize(destination.rotation);
      speedResult.translation.x /= timeForOneStep;
      speedResult.translation.y /= timeForOneStep;
      speedResult.rotation /= timeForOneStep;

      // this is probably more than can be done in one step, so cut it down
      double factor_x = speedResult.translation.x>0 ? fabs(speedResult.translation.x/(walkParams.maxSpeedXForward*fractionOfMaxForwardForOmniWalking))
                                              : fabs(speedResult.translation.x/walkParams.maxSpeedXBack);
      double factor_y = speedResult.translation.y>0 ? fabs(speedResult.translation.y/walkParams.maxSpeedYLeft*fractionOfMaxSidewaysForOmniWalking)
                                              : fabs(speedResult.translation.y/walkParams.maxSpeedYRight*fractionOfMaxSidewaysForOmniWalking);
      double factor_r = fabs(speedResult.rotation/(walkParams.maxSpeedR*fractionOfMaxRotForOmniWalking));

      double factor = max(factor_x, max(factor_y, factor_r));
      
      if (factor>1)
      {
        speedResult.translation.x /= factor;
        speedResult.translation.y /= factor;
        speedResult.rotation /= factor;
      }
    }
  }
  // gotoMode == rotation only
  else
  {
    // easy peasy
    speedResult.translation.x = 0.0;
    speedResult.translation.y = 0.0;
    speedResult.rotation = normalize(angleFirstRelative)*min((double)fabs(walkParams.maxSpeedR), fabs(normalize(angleFirstRelative)));
  }

  return speedResult;
}

Pose2D RequestTranslator::getDesiredSpeedForNextStep(const Pose2D & destination)
{
  // Input:   destination in milimeter and rad, 
  //          maxAbsoluteSpeeds (from walkParams!)
  // Output:  speed in mm/s and rad/s

  // decide between "rotation only", short range "direct omnidirectional", or long range "forward and turning"
  Pose2D speed;

  const double & omniAreaThreshold_x_front = goToParameters.omniAreaThreshold_x_front;
  const double & omniAreaThreshold_x_back = goToParameters.omniAreaThreshold_x_back;
  const double & omniAreaThreshold_y = goToParameters.omniAreaThreshold_y;
  const double & rotationOnlyThreshold_angle = goToParameters.rotationOnlyThreshold_angle;
  const double & fractionOfMaxRotForFastWalking = goToParameters.fractionOfMaxRotForFastWalking;
  const double & fractionOfMaxRotForOmniWalking = goToParameters.fractionOfMaxRotForOmniWalking;
  const double & fractionOfMaxForwardForOmniWalking = goToParameters.fractionOfMaxForwardForOmniWalking;
  const double & fractionOfMaxSidewaysForOmniWalking = goToParameters.fractionOfMaxSidewaysForOmniWalking;

  double angleToDestinationPoint = destination.translation.angle();
  double delta = normalize(destination.rotation - angleToDestinationPoint);
  double angleCorrectionForTargetOrientation = sgn(delta)*min(0.2,fabs(delta));
  double correctedAngle = normalize(angleToDestinationPoint - angleCorrectionForTargetOrientation);
  
  double timeForOneStep = walkParams.stepDuration / 2;

  if ((destination.translation.x > -omniAreaThreshold_x_back && destination.translation.x < omniAreaThreshold_x_front) // prefer walking backwards to turning+forwards+turning
      && fabs(destination.translation.y) < omniAreaThreshold_y)
  { // walk omnidirectional for short range

    // exception: if target angle differs a lot, rotate on the spot first
    if (fabs(destination.rotation) > rotationOnlyThreshold_angle)
    { // rotate only
      speed.translation.x = 0;
      speed.translation.y = 0;
      speed.rotation = sgn(destination.rotation) * walkParams.maxSpeedR;
    }
    else
    {
      // this would be the speed necessary to reach the destination in one step:
      speed = destination;
      speed.rotation = normalize(speed.rotation);
      speed.translation.x /= timeForOneStep;
      speed.translation.y /= timeForOneStep;
      speed.rotation /= timeForOneStep;

      // this is probably more than can be done in one step, so cut it down
      double factor_x = speed.translation.x>0 ? fabs(speed.translation.x/(walkParams.maxSpeedXForward*fractionOfMaxForwardForOmniWalking))
                                              : fabs(speed.translation.x/walkParams.maxSpeedXBack);
      double factor_y = speed.translation.y>0 ? fabs(speed.translation.y/walkParams.maxSpeedYLeft*fractionOfMaxSidewaysForOmniWalking)
                                              : fabs(speed.translation.y/walkParams.maxSpeedYRight*fractionOfMaxSidewaysForOmniWalking);
      double factor_r = fabs(speed.rotation/(walkParams.maxSpeedR*fractionOfMaxRotForOmniWalking));

      double factor = max(factor_x, max(factor_y, factor_r));
      
      if (factor>1)
      {
        speed.translation.x /= factor;
        speed.translation.y /= factor;
        speed.rotation /= factor;
      }
    }
  }
  else if (fabs(correctedAngle) > rotationOnlyThreshold_angle)
  { // rotate only
    speed.translation.x = 0;
    speed.translation.y = 0;
    speed.rotation = sgn(correctedAngle) * walkParams.maxSpeedR;
  }
  else
  { // walk there using forward and rotation, but without strafing
    speed.translation.x = walkParams.maxSpeedXForward;
    speed.translation.y = 0;
    double p_factor = min(1.0, fabs(correctedAngle) /0.5);
    speed.rotation = sgn(correctedAngle) 
              * p_factor
              * fractionOfMaxRotForFastWalking * walkParams.maxSpeedR;
  }

  return speed;
}

Pose2D RequestTranslator::getOmniSpeedForNextStep(const Pose2D &destination, const double maxSpeed)
{
  Pose2D speed;
  
  const double maxSpeedYLeft = min(maxSpeed, walkParams.maxSpeedYLeft*goToParameters.fractionOfMaxSidewaysForOmniWalking);
  const double maxSpeedYRight = min(maxSpeed, walkParams.maxSpeedYRight*goToParameters.fractionOfMaxSidewaysForOmniWalking);
  const double maxSpeedXForward = min(maxSpeed, walkParams.maxSpeedXForward*goToParameters.fractionOfMaxForwardForOmniWalking);
  const double maxSpeedXBack = min(maxSpeed, (double)walkParams.maxSpeedXBack);
  const double maxSpeedRot = walkParams.maxSpeedR*goToParameters.fractionOfMaxRotForOmniWalking;

  double timeForOneStep = walkParams.stepDuration / 2;

  // this would be the speed necessary to reach the destination in one step:
  speed = destination;
  speed.rotation = normalize(speed.rotation);
  speed.translation.x /= timeForOneStep;
  speed.translation.y /= timeForOneStep;
  speed.rotation /= timeForOneStep;

  // this is probably more than can be done in one step, so cut it down
  double factor_x = speed.translation.x>0 ? fabs(speed.translation.x/maxSpeedXForward)
                                          : fabs(speed.translation.x/maxSpeedXBack);
  double factor_y = speed.translation.y>0 ? fabs(speed.translation.y/maxSpeedYLeft)
                                          : fabs(speed.translation.y/maxSpeedYRight);
  double factor_r = fabs(speed.rotation/maxSpeedRot);

  double factor = max(factor_x, max(factor_y, factor_r));
  
  if (factor>1)
  {
    speed.translation.x /= factor;
    speed.translation.y /= factor;
    speed.rotation /= factor;
  }

  return speed;
}

void RequestTranslator::drawPath()
{
  // draw current pose
  POSE_2D_SAMPLE("module:RequestTranslator:Path", Pose2D(),
                                                     ColorRGBA(255,255,255));

  // draw destination
  Pose2D destination = theMotionSelection.walkRequest.target;
  CIRCLE("module:RequestTranslator:Path", destination.translation.x, destination.translation.y, 100, 10, Drawings::ps_solid, ColorRGBA(255,0,0), Drawings::ps_null, ColorRGBA(255,0,0) );
  POSE_2D_SAMPLE("module:RequestTranslator:Path", destination,
                                                     ColorRGBA(255,0,0));


  
  double timeForOneStep = walkParams.stepDuration / 2;
  Pose2D tempSpeed;

  Pose2D tempPosition;
  Pose2D remainingPathAsPose = destination;

  // draw path that will be executed if the target does not change (and if the walk/odometry is perfect)
  for (int i=0; i<100 && (remainingPathAsPose.translation.abs() > 10 || fabs(remainingPathAsPose.rotation) > 0.05); i++)
  {
    tempSpeed = getDesiredSpeedForNextStep(remainingPathAsPose);
    tempPosition += Pose2D(tempSpeed.rotation * timeForOneStep, tempSpeed.translation.x*timeForOneStep, tempSpeed.translation.y*timeForOneStep);
    POSE_2D_SAMPLE("module:RequestTranslator:Path", tempPosition,
                                                       ColorRGBA(0,160,255));
    remainingPathAsPose = destination - tempPosition;
  }
}

void RequestTranslator::drawPathWithSpeed(const Pose2D & speed)
{
  // draw current pose
  POSE_2D_SAMPLE("module:RequestTranslator:Path", Pose2D(),
                                                     ColorRGBA(255,255,255));

  
  double timeForOneStep = walkParams.stepDuration / 2;

  Pose2D tempPosition;

  Pose2D offset(speed.rotation * timeForOneStep, speed.translation.x*timeForOneStep, speed.translation.y*timeForOneStep);

  // draw path that will be executed if the speed does not change
  for (int i=0; i<10; i++)
  {
    tempPosition += offset;
    POSE_2D_SAMPLE("module:RequestTranslator:Path", tempPosition,
                                                       ColorRGBA(0,160,255));
  }
}

