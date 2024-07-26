/**
* @file Path.h
* Declaration of a class that represents the path to a robots target.
* @author <a href="mailto:ingmar.schwarz@tu-dortmund.de">Ingmar Schwarz</a>
*/

#ifndef __Path_h_
#define __Path_h_

#include "Tools/Streams/Streamable.h"
#include "Tools/Math/Pose2D.h"
#include "Tools/Debugging/DebugDrawings.h"
#include <vector>

/**
* @class Path
* A class that represents the path to the robots target.
*/
class Path : public Streamable
{
private:
  void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN();
    STREAM_VECTOR(wayPoints);
    STREAM(nearestObstacle);
    STREAM_REGISTER_FINISH();
  }

public:

  std::vector<Pose2D> wayPoints; /**< The wayPoints in absolute field coordinates. */
  double nearestObstacle;

  /**
  * Default constructor.
  */
  Path() {wayPoints.reserve(6);nearestObstacle = 10000.0;}

  /** Reset the path */
  void reset()
  {
    wayPoints.clear();
    nearestObstacle = 10000.0;
  }

  

  /**
  * The method draws the path.
  */
  void draw() const
  {
    DECLARE_DEBUG_DRAWING("representation:Path", "drawingOnField");
    // draw next path points
    for (std::vector<Pose2D>::const_iterator i = wayPoints.begin(); i != wayPoints.end(); ++i)
    {
      POSE_2D_SAMPLE("representation:Path", (*i), ColorRGBA(0,255,0));
    }
  }
};

#endif //__Path_h_
