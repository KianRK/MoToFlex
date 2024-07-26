/**
 * @file ObstaclesPercept.cpp
 *
 * Implementation of class ObstaclesPercept.
 * @author <a href="mailto:juengel@informatik.hu-berlin.de">Matthias Juengel</a>
 */

#include "ObstaclesPercept.h"

#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/ColorClasses.h"




void ObstaclesPercept::draw()
{
  DECLARE_DEBUG_DRAWING("representation:ObstaclesPercept:Image", "drawingOnImage");//, "drawing of representation ObstaclesPercept");
  DECLARE_DEBUG_DRAWING("representation:ObstaclesPercept:Field", "drawingOnField");//, "drawing of representation ObstaclesPercept");

  for (int i=0; i<numberOfSegments; i++)
  {
    LINE("representation:ObstaclesPercept:Image",
      segments[i].nearPointInImage.x,segments[i].nearPointInImage.y,
      segments[i].farPointInImage.x,segments[i].farPointInImage.y,
      1,Drawings::ps_solid,ColorClasses::gray);
    LINE("representation:ObstaclesPercept:Field",
      segments[i].nearPointOnField.x,segments[i].nearPointOnField.y,
      segments[i].farPointOnField.x,segments[i].farPointOnField.y,
      1,Drawings::ps_solid,ColorClasses::gray);
    if (!segments[i].farPointIsOnImageBorder)
    {
      ColorRGBA pen,fill;
      switch (segments[i].obstacleType)
      {
        case border:
          pen = ColorRGBA(ColorClasses::yellow);
          fill = ColorRGBA(ColorClasses::white);
        case goal:
          pen = ColorRGBA(ColorClasses::yellow);
          fill = ColorRGBA(255,0,255);//ColorClasses::pink;
        case opponent:
          pen = ColorRGBA(ColorClasses::black);
          fill = ColorRGBA(255,0,255);//ColorClasses::pink;
        case teammate:
          pen = ColorRGBA(ColorClasses::black);
          fill = ColorRGBA(255,0,255);//ColorClasses::pink;
        case unknown:
        default:
          pen = ColorRGBA(ColorClasses::black);
          fill = ColorRGBA(ColorClasses::white);
          break;
      }
      DOT("representation:ObstaclesPercept:Image",segments[i].farPointInImage.x,segments[i].farPointInImage.y,pen,fill);
      LARGE_DOT("representation:ObstaclesPercept:Field",segments[i].farPointOnField.x,segments[i].farPointOnField.y,pen,fill);
    }
  }
}
