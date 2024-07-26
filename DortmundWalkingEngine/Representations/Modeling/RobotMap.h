/**
* @file RobotMap.h
* Declaration of a class that represents the map of tracked robots in this robot's environment.
* @author <a href="mailto:stefan.czarnetzki@tu-dortmund.de">Stefan Czarnetzki</a>
*/

#ifndef __RobotMap_h_
#define __RobotMap_h_

#include "Tools/Streams/Streamable.h"
#include "Tools/Math/Pose2D.h"
#include "Tools/Debugging/DebugDrawings.h"
#include <vector>

/**
* @class RobotMap
* A class represents the map of tracked robots in this robot's environment.
*/
class RobotMap : public Streamable
{
private:
  void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN();
    STREAM_VECTOR(robots);
    STREAM_REGISTER_FINISH();
  }

public:
  class RobotMapEntry : public Streamable
  {
  private:
    void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN();
      STREAM(pose);
      STREAM(isTeamMate);
      STREAM_REGISTER_FINISH();
    }

  public:
    Pose2D pose; /**< The position in absolute field coordinates. */
    bool isTeamMate;
  };

  std::vector<RobotMapEntry> robots; /**< The robots in absolute field coordinates. */

  /**
  * Default constructor.
  */
  RobotMap() {robots.reserve(10);}

  /** Reset the PointsPercept */
  void reset()
  {
    robots.clear();
  }

  

  /**
  * The method draws the map.
  */
  void draw() const
  {
    DECLARE_DEBUG_DRAWING("representation:RobotMap", "drawingOnField");
    COMPLEX_DRAWING("representation:RobotMap",
      for(std::vector<RobotMapEntry>::const_iterator i = robots.begin(); i != robots.end(); ++i)
      {
        CIRCLE("representation:RobotMap", i->pose.translation.x, i->pose.translation.y, 100, 10, Drawings::ps_solid, ColorClasses::black, Drawings::ps_solid, i->isTeamMate ? ColorRGBA(150,255,0) : ColorClasses::black);
      }
    );
  }
};


class GroundTruthRobotMap : public RobotMap 
{
public:
  /** Draws something*/
  void draw() const
  {
    DECLARE_DEBUG_DRAWING("representation:GroundTruthRobotMap", "drawingOnField");
    COMPLEX_DRAWING("representation:GroundTruthRobotMap",
      for(std::vector<RobotMapEntry>::const_iterator i = robots.begin(); i != robots.end(); ++i)
      {
        CIRCLE("representation:GroundTruthRobotMap", i->pose.translation.x, i->pose.translation.y, 100, 10, Drawings::ps_solid, ColorClasses::black, Drawings::ps_solid, i->isTeamMate ? ColorRGBA(50,255,100,150) : ColorRGBA(0,0,0,150));
      }
    );
  }
};

#endif //__RobotMap_h_
