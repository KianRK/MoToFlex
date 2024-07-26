/**
* @file TeamDependentCoordinates.h
* Coordinates of global features, depending on the current team-color:
* e.g. if a red robot observes the own goal, the seen "left" goalpost is the right goalpost
* in global coordinates
* @author <a href="mailto:c_rohde@web.de">Carsten Rohde</a>
*/

#ifndef __TeamDependentCoordinates_H__
#define __TeamDependentCoordinates_H__

#include "Tools/Streams/Streamable.h"
#include "Tools/Math/Pose2D.h"

class TeamDependentCoordinates : public Streamable 
{
public:
  enum PoseFeature
  {
    centerCircle,
    leftBlueGoalpost,
    rightBlueGoalpost,
    leftYellowGoalpost,
    rightYellowGoalpost,
    numberOfPoseFeatures
  };

  Pose2D poseFeatures[numberOfPoseFeatures];

  void getFeature(Pose2D& pose, PoseFeature feature)
  {
    pose = poseFeatures[feature];
  }
/*
  Pose2D getFeature(PoseFeature feature)
  {
    return Pose2D(poseFeatures[feature]);
  }
*/
  void setFeature(const Pose2D& pose, PoseFeature feature)
  {
    poseFeatures[feature] = pose;
  }


private:
  virtual void serialize(In* in, Out* out)
  { 
    STREAM_REGISTER_BEGIN();
    STREAM_ARRAY(poseFeatures);
    STREAM_REGISTER_FINISH();
  }
};

#endif
