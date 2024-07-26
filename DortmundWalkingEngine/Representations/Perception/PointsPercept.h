/**
* @file PointsPercept.h
* Declaration of a class that represents the points found in an image.
* @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
* @author <a href="mailto:stefan.czarnetzki@tu-dortmund.de">Stefan Czarnetzki</a>
*/

#ifndef __PointsPercept_h_
#define __PointsPercept_h_

#include "Tools/Streams/Streamable.h"
#include "Tools/Math/Pose2D.h"
#include "Tools/Debugging/DebugDrawings.h"
#include <vector>

/**
* @class PointsPercept
* A class that represents the points found in an image.
*/
class PointsPercept : public Streamable
{
private:
  void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN();
    STREAM_VECTOR(points);
    STREAM_REGISTER_FINISH();
  }

public:
  class Point : public Streamable
  {
  private:
    void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN();
      STREAM(inImage);
      //STREAM(inImageNormalToLine);
      STREAM(onField);
      //STREAM(onFieldNormalToLine);
      //STREAM(direction);
      STREAM(scanLineNumber);
      STREAM(foundWithPostScan);
      STREAM_REGISTER_FINISH();
    }

  public:
    Vector2<short> inImage; /**< The point in image coordinates. */
    //Vector2<double> inImageNormalToLine; /**< The normal to the line in image coordinates. */
    Vector2<int> onField; /**< The point in field coordinates. */
    //Vector2<double> onFieldNormalToLine; /**< The normal to the line in field coordinates. */
    //float direction; /**< The direction of the GRADIENT at an line point relative to the robot. */
    int scanLineNumber; /**< The number of the scan line that the point was found on. */
    bool foundWithPostScan; /**< True, if the point was found with a post scan. */
    int postScanNumber; /**< The number of the scan line that the point was found on. */
  };

  std::vector<Point> points; /**< The points in relative field coordinates. */

  /**
  * Default constructor.
  */
  PointsPercept() {points.reserve(500);}

  /** Reset the PointsPercept */
  void reset()
  {
    points.clear();
  }

  

  /**
  * The method draws the percept.
  */
  void draw() const
  {
    DECLARE_DEBUG_DRAWING("representation:PointsPercept:Field", "drawingOnField");
    COMPLEX_DRAWING("representation:PointsPercept:Field",
      for(std::vector<Point>::const_iterator i = points.begin(); i != points.end(); ++i)
      {
        //const double directionLength = 100;
        //Vector2<double> tmp(directionLength*cos(i->direction), directionLength*sin(i->direction));
        LARGE_DOT("representation:PointsPercept:Field", i->onField.x, i->onField.y, ColorClasses::black, ColorClasses::white);
        //ARROW("representation:PointsPercept:Field", i->onField.x, i->onField.y, i->onField.x+tmp.x, i->onField.y+tmp.y, 2, Drawings::ps_solid, ColorRGBA(255,0,255));
      }
    );
    DECLARE_DEBUG_DRAWING("representation:PointsPercept:Image", "drawingOnImage");
    COMPLEX_DRAWING("representation:PointsPercept:Image",
      for(std::vector<Point>::const_reverse_iterator i = points.rbegin(); i != points.rend(); ++i)
      {
        DOT("representation:PointsPercept:Image", i->inImage.x, i->inImage.y, ColorClasses::black, ColorClasses::white);
      }
    );
  }
};

#endif //__PointsPercept_h_
