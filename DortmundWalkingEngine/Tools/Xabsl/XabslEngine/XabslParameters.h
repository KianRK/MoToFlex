/**
* @file XabslParameters.h
*
* Definition of class Parameters
*
* @author <a href="http://www.sim.informatik.tu-darmstadt.de/pers/card/risler.html">Max Risler</a>
*/

#ifndef __XabslParameters_h_
#define __XabslParameters_h_

#include "XabslTools.h"

namespace xabsl 
{

// class prototypes needed for declaration of Parameters
class Symbols;
class Enumeration;
class OptionParameters;
class Action;
class DecimalExpression;
class BooleanExpression;
class EnumeratedExpression;

/**
* @class Parameters
*
* Represents the current set of parameters of a behavior or an input symbol
*/
class Parameters
{
public:
  /** 
  * Constructor.
  * @param errorHandler A reference to a ErrorHandler instance
  */
  Parameters(ErrorHandler& errorHandler)
                  : errorHandler(errorHandler), pEnumerations(0)
  {}

  /**
  * Destructor
  */
  virtual ~Parameters() {};


  /** Reset all parameters to default values (=0) */
  void reset();

  /** The decimal parameters */
  Array<double*> decimal;

  /** The boolean parameters */
  Array<bool*> boolean;

  /** The enumerated parameters */
  Array<int*> enumerated;

  /** The enumeration domain of the enumerated parameters */
  Array<const Enumeration*> enumerations;

  /**
  * Registers a reference to the enumerations of the engine
  * This is only required when registerEnumerated is called afterwards
  */
  void registerEnumerations(Array<Enumeration*>& enumerations);

  /** 
  * Registers a reference to a decimal parameter at the parameters array.
  * @param name The name of the parameter
  * @param parameter The reference to a parameter
  */
  void registerDecimal(const char* name, double& parameter);
  
  /** 
  * Registers a reference to a boolean parameter at the parameters array.
  * @param name The name of the parameter
  * @param parameter The reference to a parameter
  */
  void registerBoolean(const char* name, bool& parameter);

  /** 
  * Registers a reference to an enumerated parameter at the parameters array.
  * @param name The name of the parameter
  * @param enumName The name of the corresponding enumeration
  * @param parameter The reference to a parameter
  */
  void registerEnumerated(const char* name, const char* enumName, int& parameter);

  void logAllParam();

protected:
  /** Used for error handling */
  ErrorHandler& errorHandler;

private:
  /** Pointer to the enumerations of the engine */
  Array<Enumeration*>* pEnumerations;
};

/**
* @class ParameterAssignment
*
* Represents the assignment of parameters of a subsequent basic behaviors or an option or an input symbol
*/
class ParameterAssignment : public Parameters
{
public:
  /** Constructor.
  * @param refParameters The referenced set of parameters
  * @param errorHandler A reference to a ErrorHandler instance
  */
  ParameterAssignment(
    Parameters* refParameters,
    ErrorHandler& errorHandler
    );

  /** 
  * Creates the parameter assignment.
  * @param input An input source for the intermediate code. It must be opened and read until 
  *              A position where a state starts.
  * @param optionParameters The parameters of the option
  * @param symbols All available symbols
  * @param timeOfOptionExecution The time how long the option is already active
  * @param timeOfStateExecution The time how long the current state is already active
  * @param actions The subsequent behaviors i.e options and basic behaviors of the state.
  */
  void create(
    InputSource& input,    
    OptionParameters& optionParameters,
    Symbols& symbols,
    unsigned& timeOfOptionExecution,
    unsigned& timeOfStateExecution,
    Array<Action*>& actions);
  
  /** Destructor */
  ~ParameterAssignment();

  /** Adds a decimal parameter assignment 
  * @param param name of the parameter
  * @param exp expression to be set to the parameter value when executing
  * @return False, if an error occurred
  */
  bool setDecimalParameter(const char* param, const DecimalExpression* exp);
  /** Adds a decimal parameter assignment 
  * @param param name of the parameter
  * @param value value to be set to the parameter value when executing
  * @return False, if an error occurred
  */
  bool setDecimalParameter(const char* param, double value);

  /** Adds a boolean parameter assignment 
  * @param param name of the parameter
  * @param exp expression to be set to the parameter value when executing
  * @return False, if an error occurred
  */
  bool setBooleanParameter(const char* param, const BooleanExpression* exp);
  /** Adds a boolean parameter assignment 
  * @param param name of the parameter
  * @param value value to be set to the parameter value when executing
  * @return False, if an error occurred
  */
  bool setBooleanParameter(const char* param, bool value);

  /** Adds an enumerated parameter assignment 
  * @param param name of the parameter
  * @param exp expression to be set to the parameter value when executing
  * @return False, if an error occurred
  */
  bool setEnumeratedParameter(const char* param, const EnumeratedExpression* exp);

  /** Adds an enumerated parameter assignment 
  * @param param name of the parameter
  * @param enumeration The enumeration.
  * @param value value to be set to the parameter value when executing
  * @return False, if an error occurred
  */
  bool setEnumeratedParameter(const char* param, const Enumeration* enumeration, int value);

  /** Adds an enumerated parameter assignment 
  * @param param name of the parameter
  * @param value value to be set to the parameter value when executing
  * @return False, if an error occurred
  */
  bool setEnumeratedParameter(const char* param, const char* value);

  /** Current parameter values, these are stored just for debugging purposes */
  Array<double> decimalValues;
  Array<bool> booleanValues;
  Array<int> enumeratedValues;

  /** Decimal expressions for the parameters */
  Array<const DecimalExpression*> decimalExpressions;
  /** Boolean expressions for the parameters */
  Array<const BooleanExpression*> booleanExpressions;
  /** Enumerated expressions for the parameters */
  Array<const EnumeratedExpression*> enumeratedExpressions;

  /**
  * sets parameter variables to current expression values
  * returns true when parameter values have been changed
  */
  bool set();
};

} // namespace

#endif // __XabslParameters_h_


