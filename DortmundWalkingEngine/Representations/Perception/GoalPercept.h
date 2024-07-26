/**
* @file GoalPercept.h
*
* Very simple representation of a seen goal
*
* @author <a href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</a>
*/

#ifndef __GoalPercept_h_
#define __GoalPercept_h_

#include "Tools/Streams/Streamable.h"
#include "Tools/Math/Vector2.h"
#include "Tools/ColorClasses.h"

class GoalPercept: public Streamable
{
  /** Streaming function
  * @param in  streaming in ...
  * @param out ... streaming out.
  */
  void serialize(In* in, Out* out);

public:
  /** Constructor */
  GoalPercept() : numberOfGoalPosts(0) {}

  /** Draws the goal */
  void draw();


  
  /** Defines the different goal posts */
  enum GoalPostSide {
    leftPost,
    rightPost,
    unknownPost,
  };
  
  class GoalPost : public Streamable
  {
  public:
    GoalPost(){}
    Vector2<int> locationOnField;
    Vector2<int> locationInImage;
    double bearing;
    double distance;
    Vector2<int> outlineInImage[4]; // bottomLeft, bottomRight, topRight, topLeft
    ColorClasses::Color goalColor;
    GoalPostSide goalPostSide;
    double distanceFromSize;
    bool isDistanceOnlyFromSize; // less reliable!
    double orientation; // angle of the ground line (if detected) [in field coordinates relative to the robot]
    bool orientationKnown;

    void serialize(In* in, Out* out)
    {
      STREAM_REGISTER_BEGIN();
      STREAM(locationOnField);
      STREAM(locationInImage);
      STREAM(bearing);
      STREAM(distance);
      STREAM_ARRAY(outlineInImage);
      STREAM_ENUMASINT(goalColor);
      STREAM_ENUMASINT(goalPostSide);
      STREAM(distanceFromSize);
      STREAM(isDistanceOnlyFromSize);
      STREAM(orientation);
      STREAM(orientationKnown);
      STREAM_REGISTER_FINISH();
    }
  };


  GoalPost goalPosts[4];
  int numberOfGoalPosts;
};

class PreGoalPercept : public GoalPercept
{
public:
  /** Draws the goal */
  void draw(); // only overwrite draw(), rest stays the same
};

#endif// __GoalPercept_h_
