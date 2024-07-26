/**
* @file CenterCirclePercept.h
* Declaration of a class that represents the center circle found in an image.
* @author <a href="mailto:stefan.czarnetzki@tu-dortmund.de">Stefan Czarnetzki</a>
*/

#ifndef __CenterCirclePercept_h_
#define __CenterCirclePercept_h_

#include "Tools/Streams/Streamable.h"
#include "Tools/Math/Pose2D.h"
#include "Tools/Debugging/DebugDrawings.h"
#include <vector>

/**
* @class CenterCirclePercept
* A class that represents the lines found in an image.
*/
class CenterCirclePercept : public Streamable
{
private:
  void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN();
    STREAM(centerCircle);
    STREAM(centerCircleWasSeen);
    STREAM_REGISTER_FINISH();
  }

public:
  class CenterCircle : public Streamable
  {
  public:
    Vector2<int> locationOnField;
    Vector2<int> locationInImage;
    double orientation; // angle of the center line (if detected) [in field coordinates relative to the robot]
    bool orientationKnown;

    void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN();
      STREAM(locationOnField);
      STREAM(locationInImage);
      STREAM(orientation);
      STREAM(orientationKnown);
      STREAM_REGISTER_FINISH();
    }
  };

  CenterCircle centerCircle;
  bool centerCircleWasSeen;

  /**
  * Default constructor.
  */
  CenterCirclePercept():centerCircleWasSeen(false) {}

  /** Reset the CenterCirclePercept */
  void reset()
  {
    centerCircleWasSeen = false;
  }

  

  /**
  * The method draws the percept.
  */
  void draw() const
  {
    DECLARE_DEBUG_DRAWING("representation:CenterCirclePercept:Image", "drawingOnImage"); //, "drawing of representation LineCrossingsPercept");
    DECLARE_DEBUG_DRAWING("representation:CenterCirclePercept:Field", "drawingOnField"); //, "drawing of representation LineCrossingsPercept");
    if (centerCircleWasSeen)
    {
      const double circleRadius = 625;

      CIRCLE("representation:CenterCirclePercept:Image", centerCircle.locationInImage.x, centerCircle.locationInImage.y, 6, 2,Drawings::ps_solid, ColorClasses::blue, Drawings::ps_null, ColorClasses::noColor);

      CIRCLE("representation:CenterCirclePercept:Field",centerCircle.locationOnField.x,centerCircle.locationOnField.y,circleRadius,30,Drawings::ps_solid,ColorClasses::blue, Drawings::bs_null, ColorClasses::noColor);
      if (centerCircle.orientationKnown)
      {
        LINE("representation:CenterCirclePercept:Field",(centerCircle.locationOnField.x+1.2*circleRadius*cos(centerCircle.orientation)),(centerCircle.locationOnField.y+1.2*circleRadius*sin(centerCircle.orientation)),(centerCircle.locationOnField.x-1.2*circleRadius*cos(centerCircle.orientation)),(centerCircle.locationOnField.y-1.2*circleRadius*sin(centerCircle.orientation)),30,Drawings::ps_solid,ColorClasses::blue);
      }
    }
  }
};

#endif //__CenterCirclePercept_h_
