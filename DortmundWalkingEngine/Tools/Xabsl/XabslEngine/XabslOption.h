/**
* @file XabslOption.h
* 
* Definition of class Option and Helper classes
*
* @author <a href="http://www.martin-loetzsch.de">Martin Loetzsch</a>
* @author <a href="http://www.sim.informatik.tu-darmstadt.de/pers/card/risler.html">Max Risler</a>
*/

#ifndef __XabslOption_h_
#define __XabslOption_h_

#include "XabslState.h"
#include "XabslBehavior.h"

#include "Tools/Debugging/DebugSymbolsLog.h"

namespace xabsl 
{

/**
* @class OptionParameters
*
* Represents the current set of parameters of an option or basic behavior
*
* @author <a href="http://www.martin-loetzsch.de">Martin Loetzsch</a>
* @author <a href="http://www.sim.informatik.tu-darmstadt.de/pers/card/risler.html">Max Risler</a>
*/
class OptionParameters : public Parameters
{
public:
  /** 
  * Constructor.
  * @param input An input source for the intermediate code. It must be opened and read until 
  *              A position where an option starts.
  * @param errorHandler A reference to a ErrorHandler instance
  * @param symbols All available symbols
  */
  OptionParameters(InputSource& input, 
                        ErrorHandler& errorHandler,
                        Symbols& symbols);

  /** Destructor */
  ~OptionParameters();
};

/** 
* @class Option
* 
* Represents a single option written in 
* 
* @author <a href="http://www.martin-loetzsch.de">Martin Loetzsch</a>
* @author <a href="http://www.sim.informatik.tu-darmstadt.de/pers/card/risler.html">Max Risler</a>
*/
class Option : public Behavior
{
public:
  /** 
  * Constructor. Does not create the option.
  * @param name The name of the option. For debugging purposes.
  * @param input An input source for the intermediate code. It must be opened and read until 
  *              A position where an option starts.
  * @param errorHandler A reference to a ErrorHandler instance
  * @param symbols All available symbols
  * @param pTimeFunction a pointer to a function that returns the system time in ms.
  */
  Option(const char* name, InputSource& input,
    ErrorHandler& errorHandler,
    Symbols& symbols,
    unsigned (*pTimeFunction)());
  
  /** Destructor. Deletes the states */
  ~Option();
  
  /** 
  * Creates the option and it's states from the intermediate code.
  * @param input An input source for the intermediate code. It must be opened and read until 
  *              A position where an option starts.
  * @param options All other options
  * @param basicBehaviors All basic behaviors
  * @param symbols All available symbols
  */
  void create(InputSource& input,    
    Array<Option*>& options,
    Array<BasicBehavior*>& basicBehaviors,
    Symbols& symbols);
  
  /**
  * Executes the option. The state machine is carried out to determine the subsequent 
  * option or the subsequent basic behavior. Output symbols are set.
  */
  virtual void execute();
  
  /** The states of the option */
  Array<State*> states;
  
  /** A pointer to the active state. If 0, then no state is active */
  State* activeState;
  
  /** Returns wheter the option reached a target state */
  bool getOptionReachedATargetState() const;

private:

  /** A pointer to the initial state */
  State* initialState;
  
  /** Used for error handling */
  ErrorHandler& errorHandler;

  /** A pointer to a function that returns the system time in ms. */
  unsigned (*pTimeFunction)();
};

} // namespace

#endif //__XabslOption_h_
