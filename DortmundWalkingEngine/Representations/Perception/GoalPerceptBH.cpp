/**
* @file GoalPerceptBH.h
*
* Representation of a seen goal
*
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
*/

#include "GoalPerceptBH.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"


void GoalPost::serialize(In* in, Out* out)
{
  STREAM_REGISTER_BEGIN();
    STREAM(positionInImage);
    STREAM(positionOnField);
    STREAM(timeWhenLastSeen);
    STREAM_ENUMASINT(perceptionType);
    STREAM_ENUMASINT(distanceType);
  STREAM_REGISTER_FINISH();
}

void GoalPerceptBH::serialize(In* in, Out* out)
{
  STREAM_REGISTER_BEGIN();
    STREAM_ARRAY(posts);
    STREAM_ARRAY(unknownPosts);
    STREAM_ENUMASINT(ownTeamColorForDrawing);
    STREAM(timeWhenOppGoalLastSeen);
  STREAM_REGISTER_FINISH();
}

void GoalPerceptBH::draw()
{
  DECLARE_DEBUG_DRAWING("representation:GoalPerceptBH:Image", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("representation:GoalPerceptBH:Field", "drawingOnField");
  DECLARE_DEBUG_DRAWING3D("representation:GoalPerceptBH", "origin");
  TRANSLATE3D("representation:GoalPerceptBH", 0, 0, -230);
  ColorRGBA ownColor = ColorRGBA(ownTeamColorForDrawing);
  ColorRGBA oppColor = ownTeamColorForDrawing == ColorClasses::yellow ? ColorRGBA(ColorClasses::blue) : ColorRGBA(ColorClasses::yellow);
  for(int i=0; i<NUMBER_OF_GOAL_POSTS; ++i)
  {
    const GoalPost& p = posts[i];
    ColorRGBA color = i > 1 ? ownColor : oppColor;
    if(p.perceptionType == GoalPost::SEEN_IN_IMAGE)
    { 
      CIRCLE("representation:GoalPerceptBH:Field", p.positionOnField.x, p.positionOnField.y, 
        50, 1, Drawings::ps_solid, ColorClasses::white, Drawings::bs_solid, color);
      int barY = (i == GoalPerceptBH::LEFT_OWN) || (i == GoalPerceptBH::LEFT_OPPONENT) ? -700 : 700;
      LINE("representation:GoalPerceptBH:Field", p.positionOnField.x, p.positionOnField.y,
        p.positionOnField.x, p.positionOnField.y + barY, 60, Drawings::ps_solid, color);
      MID_DOT("representation:GoalPerceptBH:Image", p.positionInImage.x, p.positionInImage.y, 
        ColorClasses::white, color);
      LINE("representation:GoalPerceptBH:Image", p.positionInImage.x, p.positionInImage.y, p.positionInImage.x, 0,
        5, Drawings::ps_solid, color);
      // Sorry, no access to field dimensions here, so the dimensions are hard coded
      CYLINDER3D("representation:GoalPerceptBH", p.positionOnField.x, p.positionOnField.y, 400, 0, 0, 0, 50, 800, color);
    }
  }
  for(int i=0; i<NUMBER_OF_UNKNOWN_GOAL_POSTS; ++i)
  {
    const GoalPost& p = unknownPosts[i];
    ColorRGBA color = i > 0 ? ownColor : oppColor;
    if(p.perceptionType == GoalPost::SEEN_IN_IMAGE)
    { 
      CIRCLE("representation:GoalPerceptBH:Field", p.positionOnField.x, p.positionOnField.y, 
        50, 1, Drawings::ps_solid, ColorRGBA(255,0,0), Drawings::bs_solid, color);
      MID_DOT("representation:GoalPerceptBH:Image", p.positionInImage.x, p.positionInImage.y, 
        ColorRGBA(255,0,0), color);
      LINE("representation:GoalPerceptBH:Image", p.positionInImage.x, p.positionInImage.y, p.positionInImage.x, 0,
        5, Drawings::ps_dot, color);
      // Sorry, no access to field dimensions here, so the dimensions are hard coded
      CYLINDER3D("representation:GoalPerceptBH", p.positionOnField.x, p.positionOnField.y, 400, 0, 0, 0, 50, 800, color);
    }
  }
}
