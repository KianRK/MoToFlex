/**
 * @file LineCrossingsPercept.cpp
 *
 * Implementation of class LineCrossingsPercept.
 * @author <A href=mailto:stefan.czarnetzki@tu-dortmund.de>Stefan Czarnetzki</A>
 */

#include "LineCrossingsPercept.h"

const ColorRGBA LineCrossingsPercept::crossingColors[] = {
										   ColorRGBA(0,0,0),  //noLineOnThisSide
 										   ColorRGBA(128,128,128),   //dontKnow
										   ColorRGBA(255,255,255),  //lineOnThisSide
										  };


void LineCrossingsPercept::draw()
{
  DECLARE_DEBUG_DRAWING("representation:LineCrossingsPercept:Image", "drawingOnImage");
  DECLARE_DEBUG_DRAWING("representation:LineCrossingsPercept:Field", "drawingOnField");

  // Draw line crossings:
  for(std::vector<LineCrossing>::const_iterator i = lineCrossings.begin(); i != lineCrossings.end(); ++i)
  {
    const LineCrossing & c(*i);
    const Vector2<int>& pI = c.locationInImage;
    const Vector2<int>& pf = c.locationOnField;
    const double & af = c.angleOnField;

    // draw characteristics of the sides: white->lineOnThisSide, black->noLineOnThisSide, light_gray->dontKnow
    LARGE_DOT("representation:LineCrossingsPercept:Field", pf.x, pf.y, ColorClasses::red, ColorClasses::red);
    CIRCLE("representation:LineCrossingsPercept:Field",pf.x,pf.y,60,15,Drawings::ps_solid,ColorClasses::red, Drawings::bs_null, ColorClasses::noColor);
    LINE("representation:LineCrossingsPercept:Field", // side1
      pf.x,
      pf.y,
      pf.x + (int)(200.0*cos(af)),
      pf.y + (int)(200.0*sin(af)),
      15,
      Drawings::ps_solid,
      LineCrossingsPercept::crossingColors[c.side[0]]
    );
    LINE("representation:LineCrossingsPercept:Field",// side2
      pf.x,
      pf.y,
      pf.x + (int)(200.0*cos(af+pi_2)),
      pf.y + (int)(200.0*sin(af+pi_2)),
      15,
      Drawings::ps_solid,
      LineCrossingsPercept::crossingColors[c.side[1]]
    );
    LINE("representation:LineCrossingsPercept:Field",// side3
      pf.x,
      pf.y,
      pf.x + (int)(200.0*cos(af+pi)),
      pf.y + (int)(200.0*sin(af+pi)),
      15,
      Drawings::ps_solid,
      LineCrossingsPercept::crossingColors[c.side[2]]
    );
    LINE("representation:LineCrossingsPercept:Field",// side4
      pf.x,
      pf.y,
      pf.x + (int)(200.0*cos(af+pi3_2)),
      pf.y + (int)(200.0*sin(af+pi3_2)),
      15,
      Drawings::ps_solid,
      LineCrossingsPercept::crossingColors[c.side[3]]
    );

    CIRCLE("representation:LineCrossingsPercept:Image",pI.x,pI.y,5,2,Drawings::ps_solid,ColorClasses::red, Drawings::bs_null, ColorClasses::noColor);
    LINE("representation:LineCrossingsPercept:Image", // side1
      pI.x,
      pI.y,
      pI.x + (int)(20.0*cos(c.angleInImage1)),
      pI.y + (int)(20.0*sin(c.angleInImage1)),
      2,
      Drawings::ps_solid,
      LineCrossingsPercept::crossingColors[c.side[0]]
    );
    LINE("representation:LineCrossingsPercept:Image",// side2
      pI.x,
      pI.y,
      pI.x + (int)(20.0*cos(c.angleInImage2)),
      pI.y + (int)(20.0*sin(c.angleInImage2)),
      2,
      Drawings::ps_solid,
      LineCrossingsPercept::crossingColors[c.side[1]]
    );
    LINE("representation:LineCrossingsPercept:Image",// side3
      pI.x,
      pI.y,
      pI.x + (int)(20.0*cos(c.angleInImage1+pi)),
      pI.y + (int)(20.0*sin(c.angleInImage1+pi)),
      2,
      Drawings::ps_solid,
      LineCrossingsPercept::crossingColors[c.side[2]]
    );
    LINE("representation:LineCrossingsPercept:Image",// side4
      pI.x,
      pI.y,
      pI.x + (int)(20.0*cos(c.angleInImage2+pi)),
      pI.y + (int)(20.0*sin(c.angleInImage2+pi)),
      2,
      Drawings::ps_solid,
      LineCrossingsPercept::crossingColors[c.side[3]]
    );
    MID_DOT("representation:LineCrossingsPercept:Image", pI.x, pI.y, ColorClasses::red, ColorClasses::red);
  }
}
