/**
* @file ObstaclesPercept.h
* 
* Declaration of class ObstaclesPercept
* @author <a href="mailto:juengel@informatik.hu-berlin.de">Matthias Juengel</a>
*/ 

#ifndef __ObstaclesPercept_h_
#define __ObstaclesPercept_h_


#include "Tools/Streams/InOut.h"
#include "Tools/Math/Vector2.h"
#include "Tools/Streams/Streamable.h"
//#include "Tools/ColorClasses.h"

/**
* The class represents perceived obstacles on the field.
*/
class ObstaclesPercept : public Streamable
{
  void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN();
    STREAM_DYN_ARRAY(segments, numberOfSegments);
    STREAM(timeStamp);
    STREAM_REGISTER_FINISH();
  }

public:
  enum ObstacleType {border, goal, teammate, opponent, unknown};
  enum RobotType{teammateRobot, opponentRobot, unknownRobot, noRobot, invalid};      /**<hack used by additional robot scanning in fieldspecialist */
  enum {maxNumberOfSegments = 200};                                                  /**< Specifies the maximum number of segments. */
  enum WhoseGoal{ownGoal = 0, opponentGoal = 1};

  class Segment : public Streamable
  {
    void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN();
      STREAM(nearPointOnField);
      STREAM(farPointOnField);
      STREAM(nearPointInImage);
      STREAM(farPointInImage);
      STREAM(farPointIsOnImageBorder);
      STREAM_ENUMASINT(obstacleType);
      STREAM_ENUMASINT(robotType);
      STREAM_REGISTER_FINISH();
    }

  public:
      Vector2<double> nearPointOnField, /**< The nearest green point in ego-centric field coordinates. */
          farPointOnField; /**< The most distant green point in ego-centric field coordinates. */
      Vector2<double> nearPointInImage, /**< The nearest green point in image coordinates. */
          farPointInImage; /**< The most distant green point in image coordinates. */
      bool farPointIsOnImageBorder; /**< Indicates if the far point is an obstacle or the image border */
      ObstacleType obstacleType; /**< The type of the obstacle limiting free space. */
      RobotType    robotType;

    Segment() : farPointIsOnImageBorder(false), obstacleType(unknown), robotType(noRobot) {}
  };

  /**
  * Constructor.
  */
  ObstaclesPercept() {reset(0);}

  
  /** Draws the ObstaclesPercept */
  void draw();

  /**
  * The function empties the obstacle percept.
  */
  void reset(unsigned long timeStamp)
  {
    this->timeStamp = timeStamp;
    numberOfSegments = 0;
  }

  /**
  * The function adds a new pair of points to the obstacle percept.
  * @param segment The pair of points.
  */
  void add(const Segment& segment)
  {
    if(numberOfSegments < maxNumberOfSegments)
      segments[numberOfSegments++] = segment;
  }

  Segment segments[maxNumberOfSegments]; /**< The obstacle points. */
  int numberOfSegments; /**< The number of pairs of points */

  /** The frame number when perceived. */
  unsigned long timeStamp;
};

#endif //__ObstaclesPercept_h_
