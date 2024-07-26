/**
* @file GoalPerceptBH.h
*
* Representation of a seen goal
*
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
*/

#ifndef __GoalPerceptBH_h_
#define __GoalPerceptBH_h_

#include "Tools/ColorClasses.h"
#include "Tools/Streams/Streamable.h"
#include "Tools/Math/Vector2.h"

/**
* @class GoalPost
* Description of a perceived goal post
*/
class GoalPost: public Streamable
{
  /** Streaming function
  * @param in  streaming in ...
  * @param out ... streaming out.
  */
  void serialize(In* in, Out* out);

public:
  /** The position of the goal post in the current image */
  Vector2<int> positionInImage;      
  /** The position of the goal post relative to the robot*/
  Vector2<int> positionOnField;  
  /** Timestamp of the last perception of this pole */
  unsigned timeWhenLastSeen;
  /** The two different kinds of distance computation*/
  enum DistanceType {HEIGHT_BASED = 0, BEARING_BASED, IS_CLOSER} distanceType;
  /** PerceptionType **/
  enum PerceptionType {SEEN_IN_IMAGE = 0, CALCULATED, NEVER_SEEN} perceptionType;
  
  /** Constructor */
  GoalPost() : positionInImage(Vector2<int>(0,0)), positionOnField(Vector2<int>(0,0)),
               timeWhenLastSeen(0), distanceType(GoalPost::BEARING_BASED),
               perceptionType(GoalPost::NEVER_SEEN) {}
};


/**
* @class GoalPerceptBH
* Set of perceived goal posts
*/
class GoalPerceptBH: public Streamable
{
  /** Streaming function
  * @param in  streaming in ...
  * @param out ... streaming out.
  */
  void serialize(In* in, Out* out);

public:
  /** Constants*/
  enum {LEFT_OPPONENT=0,RIGHT_OPPONENT,LEFT_OWN,RIGHT_OWN,NUMBER_OF_GOAL_POSTS,
        UNKNOWN_OPPONENT=0, UNKNOWN_OWN, NUMBER_OF_UNKNOWN_GOAL_POSTS};
  /** The known posts*/
  GoalPost posts[NUMBER_OF_GOAL_POSTS];
  /** Unknown posts, only one per team is possible */
  GoalPost unknownPosts[NUMBER_OF_UNKNOWN_GOAL_POSTS];
  /** Keep color for nicer drawing of the representation*/
  ColorClasses::Color ownTeamColorForDrawing;
  /** Information for behaviors (redundant)*/
  unsigned timeWhenOppGoalLastSeen;
  unsigned timeWhenOwnGoalLastSeen;

  /** Constructor */
  GoalPerceptBH() : 
    timeWhenOppGoalLastSeen(0),
    timeWhenOwnGoalLastSeen(0)       
  {}

  /** Draws the perceived goal posts*/
  void draw();
};

#endif// __GoalPerceptBH_h_
