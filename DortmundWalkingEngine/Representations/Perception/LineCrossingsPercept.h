/**
* @file LineCrossingsPercept.h
* 
* Declaration of class LineCrossingsPercept
 * @author <A href=mailto:stefan.czarnetzki@tu-dortmund.de>Stefan Czarnetzki</A>
*/ 

#ifndef __LineCrossingsPercept_h_
#define __LineCrossingsPercept_h_

#include "Tools/Streams/Streamable.h"
#include "Tools/Math/Vector2.h"
#include "Tools/Debugging/DebugDrawings.h"
#include <vector>

/**
* The class represents percepted line crossings on the field with their type.
*/
class LineCrossingsPercept : public Streamable
{
public:

  /**
  * Used for visualizing the crossing characteristics
  */
  static const ColorRGBA crossingColors[];


  enum {UNDEF = -4};


  /** Defines what lies on the 4 "sides" of a line crossing*/
  enum CrossingCharacteristic
  {
    noLineOnThisSide, 
    dontKnow, 
    lineOnThisSide,
    numberOfCrossingCharacteristics
  }; 

  static const char* getCrossingCharacteristicName(CrossingCharacteristic id)
  {
    switch(id)
    {
    case lineOnThisSide: return "lineOnThisSide";
    case noLineOnThisSide: return "noLineOnThisSide";
    case dontKnow: default: return "dontKnow";
    }
  }

  class LineCrossing : public Streamable
  {
  public:
    LineCrossing(){}
    Vector2<int> locationOnField;
    Vector2<int> locationInImage;
    double angleOnField, angleInImage1, angleInImage2;
    CrossingCharacteristic side[4]; // sides in mathematical positive order beginning from the one directly left of the orientation vector
    bool outOfImage;

    void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN();
      STREAM(locationOnField);
      STREAM(locationInImage);
      STREAM(angleOnField);
      STREAM(angleInImage1);
      STREAM(angleInImage2);
      STREAM_ENUM(side[0], numberOfCrossingCharacteristics, LineCrossingsPercept::getCrossingCharacteristicName);
      STREAM_ENUM(side[1], numberOfCrossingCharacteristics, LineCrossingsPercept::getCrossingCharacteristicName);
      STREAM_ENUM(side[2], numberOfCrossingCharacteristics, LineCrossingsPercept::getCrossingCharacteristicName);
      STREAM_ENUM(side[3], numberOfCrossingCharacteristics, LineCrossingsPercept::getCrossingCharacteristicName);
      STREAM(outOfImage);
      STREAM_REGISTER_FINISH();
    }
  };

  std::vector<LineCrossing> lineCrossings; /**< The crossings in relative field coordinates. */


  /**
  * Constructor.
  */
  LineCrossingsPercept() {lineCrossings.reserve(20);}

  /**
  * The function empties the crossings percept.
  */
  void reset()
  {
    lineCrossings.clear();
  }

  /** Draws the percept*/
  void draw();

private:
  void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN();
    STREAM_VECTOR(lineCrossings);
    STREAM_REGISTER_FINISH();
  }
};

#endif //__LineCrossingsPercept_h_
