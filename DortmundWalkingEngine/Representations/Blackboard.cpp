/** 
* @file Blackboard.cpp
* Implementation of a class representing the blackboard.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
*/

#include "Blackboard.h"
#include <string.h>
#include <stdlib.h>

Blackboard::Blackboard() :
  // Initialize all representations by themselves:
  // Infrastructure
  theJointData(theJointData),
  theJointRequest(theJointRequest),
  theSensorData(theSensorData),
  theKeyStates(theKeyStates),
  theLEDRequest(theLEDRequest),
  theImage(theImage),
  theCameraInfo(theCameraInfo),
  theFrameInfo(theFrameInfo),
  theCognitionFrameInfo(theCognitionFrameInfo),
  theRobotInfo(theRobotInfo),
  theOwnTeamInfo(theOwnTeamInfo),
  theOpponentTeamInfo(theOpponentTeamInfo),
  theGameInfo(theGameInfo),
  theTeamStatus(theTeamStatus),
  theSoundRequest(theSoundRequest),
  theSoundOutput(theSoundOutput),
  theTeamMateData(theTeamMateData),
  theMotionRobotHealth(theMotionRobotHealth),
  theRobotHealth(theRobotHealth),
  theBoardInfo(theBoardInfo),
  theCognitionScheduleData(theCognitionScheduleData),

  // Configuration
  theRobotName(theRobotName),
  theColorTable64(theColorTable64),
  theCameraSettings(theCameraSettings),
  theFieldDimensions(theFieldDimensions),
  theRobotDimensions(theRobotDimensions),
  theJointCalibration(theJointCalibration),
  theSensorCalibration(theSensorCalibration),
  theCameraCalibration(theCameraCalibration),
  theBehaviorConfiguration(theBehaviorConfiguration),
  theMassCalibration(theMassCalibration),
  theTeamDependentCoordinates(theTeamDependentCoordinates),

  // Perception
  theCameraMatrix(theCameraMatrix),
  theRobotCameraMatrix(theRobotCameraMatrix),
  theInertiaMatrix(theInertiaMatrix),
  theImageCoordinateSystem(theImageCoordinateSystem),
  theBallSpots(theBallSpots),
  theLineSpots(theLineSpots),
  theBallPercept(theBallPercept),
  theExtendedBallPercepts(theExtendedBallPercepts),
  theLinePercept(theLinePercept),
  theRegionPercept(theRegionPercept),
  theFilteredJointData(theFilteredJointData),
  theFilteredSensorData(theFilteredSensorData),
  theGoalPerceptBH(theGoalPerceptBH),
  theGoalPercept(theGoalPercept),
  thePreGoalPercept(thePreGoalPercept),
  theGroundContactState(theGroundContactState),
  theObstaclesPercept(theObstaclesPercept),
  theRobotsPercept(theRobotsPercept),
  
  thePointsPercept(thePointsPercept),
  theFieldLinesPercept(theFieldLinesPercept),
  theLineCrossingsPercept(theLineCrossingsPercept),
  theCenterCirclePercept(theCenterCirclePercept),


  // Modeling
  theFallDownState(theFallDownState),
  theBallModel(theBallModel),
  theLocalBallModel(theLocalBallModel),
  theGroundTruthBallModel(theGroundTruthBallModel),
  theObstacleModel(theObstacleModel),
  theRobotPose(theRobotPose),
  theGroundTruthRobotPose(theGroundTruthRobotPose),
  theRobotPoseHypotheses(theRobotPoseHypotheses),
  theSelfLocatorSampleSet(theSelfLocatorSampleSet),
  theBallLocatorSampleSet(theBallLocatorSampleSet),
  theRobotModel(theRobotModel),
  theBodyContour(theBodyContour),
  theZMPModel(theZMPModel),
  theSampleTemplates(theSampleTemplates),
  theWorldState(theWorldState),
  theRobotPoseAfterPreview(theRobotPoseAfterPreview),
  theBallModelAfterPreview(theBallModelAfterPreview),
  theObstacleModelAfterPreview(theObstacleModelAfterPreview),
  theRobotMap(theRobotMap),
  theWMGDistributedPercepts(theWMGDistributedPercepts),
  thePath(thePath),

  // BehaviorControl
  theBehaviorControlOutput(theBehaviorControlOutput),

  // MotionControl
  theOdometryData(theOdometryData),
  theGroundTruthOdometryData(theGroundTruthOdometryData),
  theMotionRequest(theMotionRequest),
  theHeadMotionRequest(theHeadMotionRequest),
  theHeadJointRequest(theHeadJointRequest),
  theMotionSelection(theMotionSelection),
  theSpecialActionsOutput(theSpecialActionsOutput),  
  theMotionInfo(theMotionInfo),
  theUnstableJointRequest(theUnstableJointRequest),
  theSensorPrediction(theSensorPrediction),
  theTargetCoM(theTargetCoM),
  theRefZMP(theRefZMP),
  theFootpositions(theFootpositions),
  theSpeedInfo(theSpeedInfo),
  theArmMovement(theArmMovement),
  theControllerParams(theControllerParams),
  theFootSteps(theFootSteps),
  theKinematicOutput(theKinematicOutput),
  theKinematicRequest(theKinematicRequest),
  thePatternGenRequest(thePatternGenRequest),
  theWalkingEngineOutput(theWalkingEngineOutput),
  theWalkingEngineParams(theWalkingEngineParams),
  theWalkingInfo(theWalkingInfo),
  theRawKinematicOutput(theRawKinematicOutput),
  theBodyTilt(theBodyTilt),
  theFreeLegPhaseParams(theFreeLegPhaseParams),
  theActualCoM(theActualCoM)
{
}

void Blackboard::operator=(const Blackboard& other)
{
  memcpy(this, &other, sizeof(Blackboard)); 
}

void* Blackboard::operator new(unsigned size)
{
  return calloc(1, size);
}

void Blackboard::operator delete(void* p)
{
  return free(p);
}

void Blackboard::distract()
{
}

PROCESS_WIDE_STORAGE Blackboard* Blackboard::theInstance = 0;
