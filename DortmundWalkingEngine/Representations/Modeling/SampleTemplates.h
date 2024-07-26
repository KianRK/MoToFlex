/**
* @file SampleTemplates.h
*
* Declaration of class SampleTemplates
*
* @author <a href="mailto:Tim.Laue@dfki.de">Tim Laue</a>
*/

#ifndef __SampleTemplates_h_
#define __SampleTemplates_h_

#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/Pose2D.h"


/**
* @class SampleTemplate
* Robot position generated from perceptions
*/
class SampleTemplate : public Pose2D
{
public:
  /** Constructor */
  SampleTemplate():Pose2D(),timestamp(0) {}

  /** Constructor */
  SampleTemplate(const Pose2D& pose):Pose2D(pose),timestamp(0) {}

  /** Timestamp of visual input for construction of this template */
  unsigned timestamp;
};


/**
* @class SampleTemplates
* List of templates for self-localization
*/
class SampleTemplates : public Streamable
{
private:
  /** Streaming function 
  * @param in Object for streaming in the one direction
  * @param out Object for streaming in the other direction
  */
  void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN();
      STREAM_VECTOR(templates);
    STREAM_REGISTER_FINISH();
  }

public:
  enum {
    MAX_TEMPLATES = 20,
    MIN_TEMPLATES = 3,
    MAX_TIME_TO_KEEP = 2000};

  /** Default constructor. */
  SampleTemplates() {templates.reserve(MAX_TEMPLATES);}

  void draw()
  {
    DECLARE_DEBUG_DRAWING("representation:SampleTemplates", "drawingOnField");
    for(unsigned int i=0; i<templates.size(); ++i)
    {
      /*
      CIRCLE("representation:SampleTemplates", templates[i].translation.x, templates[i].translation.y,
        400, 50, Drawings::ps_solid, ColorClasses::blue, Drawings::bs_null, ColorClasses::blue);
      Vector2<double> target(400.0,0.0);
      target.rotate(templates[i].rotation);
      target += templates[i].translation;
      LINE("representation:SampleTemplates", templates[i].translation.x, templates[i].translation.y,
        target.x, target.y, 50, Drawings::ps_solid, ColorClasses::yellow);*/
      POSE_2D_SAMPLE("representation:SampleTemplates", templates[i], ColorClasses::green);
    }
  }

  /** List of sample templates*/
  std::vector<SampleTemplate> templates;

  void getLastAndRemove(SampleTemplate& sampleTemplate)
  {
    SampleTemplate t(templates[templates.size()-1]);
    sampleTemplate = t;
    templates.pop_back();
  }
};

#endif //__SampleTemplates_h_
