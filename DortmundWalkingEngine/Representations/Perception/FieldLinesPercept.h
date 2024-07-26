/**
* @file FieldLinesPercept.h
* Declaration of a class that represents the field lines found in an image.
* @author <a href="mailto:stefan.czarnetzki@tu-dortmund.de">Stefan Czarnetzki</a>
*/

#ifndef __FieldLinesPercept_h_
#define __FieldLinesPercept_h_

#include "Tools/Streams/Streamable.h"
#include "Tools/Math/Vector2.h"
#include "Tools/Debugging/DebugDrawings.h"
#include <vector>

/**
* @class FieldLinesPercept
* A class that represents the lines found in an image.
*/
class FieldLinesPercept : public Streamable
{
private:
  void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN();
    STREAM_VECTOR(lines);
    STREAM_REGISTER_FINISH();
  }

public:
  class FieldLine : public Streamable
  {
  private:
    void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN();
      STREAM(startInImage);
      STREAM(endInImage);
      STREAM(startOnField);
      STREAM(endOnField);
      STREAM_REGISTER_FINISH();
    }

  public:
    Vector2<short> startInImage; /**< The start point in image coordinates. */
    Vector2<short> endInImage; /**< The end point in image coordinates. */
    Vector2<double> startOnField; /**< The start point in field coordinates. */
    Vector2<double> endOnField; /**< The end point in field coordinates. */
  };

  std::vector<FieldLine> lines; /**< The lines in relative field coordinates. */

  /**
  * Default constructor.
  */
  FieldLinesPercept() {lines.reserve(50);}

  /** Reset the FieldLinesPercept */
  void reset()
  {
    lines.clear();
  }

  

  /**
  * The method draws the percept.
  */
  void draw() const
  {
    DECLARE_DEBUG_DRAWING("representation:FieldLinesPercept:Field", "drawingOnField");
    COMPLEX_DRAWING("representation:FieldLinesPercept:Field",
      for(std::vector<FieldLine>::const_iterator i = lines.begin(); i != lines.end(); ++i)
      {
        LINE("representation:FieldLinesPercept:Field", i->startOnField.x, i->startOnField.y, i->endOnField.x, i->endOnField.y, 5, Drawings::ps_solid, ColorRGBA(0,255,255));
      }
    );
    DECLARE_DEBUG_DRAWING("representation:FieldLinesPercept:Image", "drawingOnImage");
    COMPLEX_DRAWING("representation:FieldLinesPercept:Image",
      for(std::vector<FieldLine>::const_reverse_iterator i = lines.rbegin(); i != lines.rend(); ++i)
      {
        LINE("representation:FieldLinesPercept:Image", i->startInImage.x, i->startInImage.y, i->endInImage.x, i->endInImage.y, 2, Drawings::ps_solid, ColorRGBA(0,255,255));
      }
    );
  }
};

#endif //__FieldLinesPercept_h_
