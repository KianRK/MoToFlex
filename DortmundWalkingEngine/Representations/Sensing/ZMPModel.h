/**
* @file ZMPModel.h
*
* Declaration of class ZMPModel
*
* @author <A href="ingmar.schwarz@tu-dortmund.de">Ingmar Schwarz</A>
*/

#ifndef ZMPModel_H
#define ZMPModel_H

#ifndef WALKING_SIMULATOR
#include "Tools/Streams/Streamable.h"
#endif
#include "Tools/Math/Vector3.h"

/**
 * @class ZMPModel
 *
 * Contains information about ZMP.
 */
class ZMPModel : public Streamable
{
  /** Streaming */
  virtual void serialize(In *in, Out *out)
  {
    STREAM_REGISTER_BEGIN();
	  STREAM(zmp_acc);
    STREAM_REGISTER_FINISH();
  }

public:

  Vector3<double> zmp_acc; //ZMP from acceleration sensors

  /** Constructor */
  ZMPModel() : zmp_acc() {}

  
};

#endif //ZMPModel_H
