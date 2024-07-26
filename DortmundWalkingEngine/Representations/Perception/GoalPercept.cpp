/**
* @file GoalPercept.cpp
*
* Very simple representation of a seen goal
*
* @author <a href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</a>
*/

#include "GoalPercept.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Debugging/DebugDrawings.h"


void GoalPercept::serialize(In* in, Out* out)
{
  STREAM_REGISTER_BEGIN();
    STREAM(numberOfGoalPosts);
    STREAM_ARRAY(goalPosts);
  STREAM_REGISTER_FINISH();
}

void GoalPercept::draw()
{
  DECLARE_DEBUG_DRAWING("representation:GoalPercept:Image", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("representation:GoalPercept:Field", "drawingOnField");

  for (int i=0; i<numberOfGoalPosts; i++)
  {
    QUADRANGLE("representation:GoalPercept:Image",
      goalPosts[i].outlineInImage[0].x,
      goalPosts[i].outlineInImage[0].y,
      goalPosts[i].outlineInImage[1].x,
      goalPosts[i].outlineInImage[1].y,
      goalPosts[i].outlineInImage[2].x,
      goalPosts[i].outlineInImage[2].y,
      goalPosts[i].outlineInImage[3].x,
      goalPosts[i].outlineInImage[3].y,
      2, Drawings::ps_solid, goalPosts[i].goalColor);
    Vector2<int> center = ( goalPosts[i].outlineInImage[0]
                          + goalPosts[i].outlineInImage[1]
                          + goalPosts[i].outlineInImage[2]
                          + goalPosts[i].outlineInImage[3] ) / 4;

                        
    CIRCLE("representation:GoalPercept:Image",center.x,center.y, 5, 2, Drawings::ps_solid, ColorClasses::gray, Drawings::ps_null, ColorClasses::noColor);
    int factor = -1;
    if (goalPosts[i].goalPostSide == leftPost)
    {
      factor = 1;
    }
    if (goalPosts[i].goalPostSide != unknownPost)
    {
      ARROW("representation:GoalPercept:Image",center.x,center.y,center.x+factor*30,center.y,2,Drawings::ps_solid, ColorClasses::white);
    }


    LINE("representation:GoalPercept:Field",0,0,goalPosts[i].locationOnField.x, goalPosts[i].locationOnField.y, 10, (goalPosts[i].isDistanceOnlyFromSize ? Drawings::ps_dot : Drawings::ps_solid), goalPosts[i].goalColor);
    CIRCLE("representation:GoalPercept:Field", goalPosts[i].locationOnField.x, goalPosts[i].locationOnField.y, 60, 10, Drawings::ps_solid, ColorClasses::white, Drawings::ps_solid, goalPosts[i].goalColor);
    
    if (goalPosts[i].goalPostSide != unknownPost)
    {      
      Vector2<double> dir((double)goalPosts[i].locationOnField.y,(double)-goalPosts[i].locationOnField.x);
      ColorClasses::Color arrowColor = ColorClasses::white;
      if (goalPosts[i].orientationKnown)
      {
        dir.x = 200*cos(goalPosts[i].orientation);
        dir.y = 200*sin(goalPosts[i].orientation);
      }
      else
      {
        dir.normalize();
        dir *= factor * 200;
        arrowColor = ColorClasses::gray;
      }
      ARROW("representation:GoalPercept:Field",
            goalPosts[i].locationOnField.x,
            goalPosts[i].locationOnField.y,
            goalPosts[i].locationOnField.x + dir.x,
            goalPosts[i].locationOnField.y + dir.y,
            20,Drawings::ps_solid, arrowColor);
    }
  }

}

void PreGoalPercept::draw()
{
  DECLARE_DEBUG_DRAWING("representation:PreGoalPercept:Image", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("representation:PreGoalPercept:Field", "drawingOnField");

  for (int i=0; i<numberOfGoalPosts; i++)
  {
    QUADRANGLE("representation:PreGoalPercept:Image",
      goalPosts[i].outlineInImage[0].x,
      goalPosts[i].outlineInImage[0].y,
      goalPosts[i].outlineInImage[1].x,
      goalPosts[i].outlineInImage[1].y,
      goalPosts[i].outlineInImage[2].x,
      goalPosts[i].outlineInImage[2].y,
      goalPosts[i].outlineInImage[3].x,
      goalPosts[i].outlineInImage[3].y,
      2, Drawings::ps_solid, goalPosts[i].goalColor);
    Vector2<int> center = ( goalPosts[i].outlineInImage[0]
                          + goalPosts[i].outlineInImage[1]
                          + goalPosts[i].outlineInImage[2]
                          + goalPosts[i].outlineInImage[3] ) / 4;

                        
    CIRCLE("representation:PreGoalPercept:Image",center.x,center.y, 5, 2, Drawings::ps_solid, ColorClasses::gray, Drawings::ps_null, ColorClasses::noColor);
    int factor = -1;
    if (goalPosts[i].goalPostSide == leftPost)
    {
      factor = 1;
    }
    if (goalPosts[i].goalPostSide != unknownPost)
    {
      ARROW("representation:PreGoalPercept:Image",center.x,center.y,center.x+factor*30,center.y,2,Drawings::ps_solid, ColorClasses::white);
    }


    LINE("representation:PreGoalPercept:Field",0,0,goalPosts[i].locationOnField.x, goalPosts[i].locationOnField.y, 10, (goalPosts[i].isDistanceOnlyFromSize ? Drawings::ps_dot : Drawings::ps_solid), goalPosts[i].goalColor);
    CIRCLE("representation:PreGoalPercept:Field", goalPosts[i].locationOnField.x, goalPosts[i].locationOnField.y, 60, 10, Drawings::ps_solid, ColorClasses::white, Drawings::ps_solid, goalPosts[i].goalColor);
    
    if (goalPosts[i].goalPostSide != unknownPost)
    {      
      Vector2<double> dir((double)goalPosts[i].locationOnField.y,(double)-goalPosts[i].locationOnField.x);
      ColorClasses::Color arrowColor = ColorClasses::white;
      if (goalPosts[i].orientationKnown)
      {
        dir.x = 200*cos(goalPosts[i].orientation);
        dir.y = 200*sin(goalPosts[i].orientation);
      }
      else
      {
        dir.normalize();
        dir *= factor * 200;
        arrowColor = ColorClasses::gray;
      }
      ARROW("representation:PreGoalPercept:Field",
            goalPosts[i].locationOnField.x,
            goalPosts[i].locationOnField.y,
            goalPosts[i].locationOnField.x + dir.x,
            goalPosts[i].locationOnField.y + dir.y,
            20,Drawings::ps_solid, arrowColor);
    }
  }

}
