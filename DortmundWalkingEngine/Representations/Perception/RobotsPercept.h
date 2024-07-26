// Simple representation of the robot percepts used in world model

#ifndef __RobotsPercept_h_
#define __RobotsPercept_h_

#include "Tools/Streams/Streamable.h"
#include "Tools/Math/Pose2D.h"
#include "ObstaclesPercept.h"


class RobotsPercept : public Streamable
{
public:
  class RobotEstimate : public Streamable
  {
  public:
    RobotEstimate():robotType(ObstaclesPercept::unknownRobot),locationOnField(Pose2D(0,0,0)),distance(10000),validity(-1),onFloor(false){}
    ObstaclesPercept::RobotType robotType;
    Pose2D locationOnField;
    double distance;
    double validity;
    bool onFloor;
    Vector2<int> imageUpperLeft;
    Vector2<int> imageLowerRight;
  private:
    void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN();
      STREAM_ENUMASINT(robotType);
      STREAM(locationOnField);
      STREAM(distance);
      STREAM(validity);
      STREAM(onFloor);
      STREAM(imageUpperLeft);
      STREAM(imageLowerRight);
      STREAM_REGISTER_FINISH();
    }
  };

  RobotsPercept(){}
  std::vector<RobotEstimate> robots;

  void draw()
  {
    DECLARE_DEBUG_DRAWING("representation:RobotsPercept:Image", "drawingOnImage");
    for (std::vector<RobotEstimate>::const_iterator robotEstimate = robots.begin(); robotEstimate != robots.end();++robotEstimate)
    {
      RECTANGLE("representation:RobotsPercept:Image",
        robotEstimate->imageUpperLeft.x,robotEstimate->imageUpperLeft.y,
        robotEstimate->imageLowerRight.x,robotEstimate->imageLowerRight.y,5,Drawings::ps_solid,ColorClasses::green);
    }
  }

private:

  void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN();
    STREAM_VECTOR(robots);
    STREAM_REGISTER_FINISH();
  }

};

#endif // __RobotsPercept_h_