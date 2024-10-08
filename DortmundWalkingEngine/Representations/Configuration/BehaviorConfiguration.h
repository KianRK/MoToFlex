/**
 * @file BehaviorConfiguration.h
 * Declaration of class BehaviorConfiguration
 *
 * @author Max Risler
 */ 

#ifndef __BehaviorConfiguration_h_
#define __BehaviorConfiguration_h_

#include <string>
#include "Tools/Streams/Streamable.h"

class BehaviorConfiguration : public Streamable
{
public:
  BehaviorConfiguration() : agent(""), printDebugSymbols(false) {}

  std::string agent;
  bool printDebugSymbols;

  virtual void serialize( In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN();
    STREAM(agent);
	STREAM(printDebugSymbols);
    STREAM_REGISTER_FINISH();
  }
};

#endif //__BehaviorConfiguration_h_
