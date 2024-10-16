/**
* @file GTXabslEngineExecutor.h
* 
* Implementation of class GTXabslEngineExecutor.
*
* @author Martin L�tzsch
*/

#ifndef __GTXabslEngineExecutor_h_
#define __GTXabslEngineExecutor_h_

#include "Tools/Xabsl/XabslEngine/XabslEngine.h"
#include "Tools/Streams/InStreams.h"
#include "Tools/MessageQueue/InMessage.h"

/** Implements xabsl::ErrorHandler using the OUTPUT macro */
class GTXabslErrorHandler : public xabsl::ErrorHandler
{
public:
/** 
* Constructor 
* @param id The id of the xabsl::Engine. 
  */
  GTXabslErrorHandler(const char* id);
  
  /** 
  * Prints out an error
  * @param text The text to display
  */
  virtual void printError(const char* text);
  
  /**
  * Prints out a message
  * @param text The text to display
  */
  virtual void printMessage(const char* text);

private:
  /** The id of the xabsl::Engine. */
  const char* id;
};

/** Implements xabsl::InputSource using the InConfigFile class */
class XabslFileInputSource : public xabsl::InputSource, public xabsl::NamedItem
{
public:
/** 
* Constructor. Does not open the file
* @param fileName The file name to open
  */
  XabslFileInputSource(const char* fileName);
  
  /** Destructor */
  ~XabslFileInputSource();
  
  /** opens the source that contains the intermediate code */
  virtual bool open();
  
  /** closes the source */
  virtual void close();
  
  /** reads a numeric value from the input source */
  virtual double readValue();
  
  /** 
  * reads a string from the input source
  * @param destination The position where to write the string
  * @param maxLength the maximum length of the string
  * @return if the read succeded
  */
  virtual bool readString(char* destination, int maxLength);
  
private:
  /** The file to read the data from */
  InConfigFile* file;
};

/** Implements xabsl::InputSource using the InConfigMessage class */
class XabslMessageInputSource : public xabsl::InputSource
{
public:
/** 
* Constructor. Does not open the file
* @param message A reference to the message that contains the intermediate code
  */
  XabslMessageInputSource(InConfigMessage& message);
  
  /** Destructor */
  ~XabslMessageInputSource() {};
  
  /** opens the source that contains the intermediate code */
  virtual bool open() {return true;};
  
  /** closes the source */
  virtual void close() {};
  
  /** reads a numeric value from the input source */
  virtual double readValue();
  
  /** 
  * reads a string from the input source
  * @param destination The position where to write the string
  * @param maxLength the maximum length of the string
  * @return if the read succeded
  */
  virtual bool readString(char* destination, int maxLength);
  
private:
  /** The file to read the data from */
  InConfigMessage& message;
};

/**
* @class GTXabslEngineExecutor
*
* Executes an xabsl::Engine in the GT - architecture
*
* @author Martin L�tzsch
*/ 
class GTXabslEngineExecutor 
{
public:
  /** 
  * Constructor.
  * @param id The id of the xabsl::Engine derivate.
  */
  GTXabslEngineExecutor(const char* id);
  
  /** destructor */
  ~GTXabslEngineExecutor();
  
  /** 
  * Creates a new engine 
  */
  void init();

  /** 
  * Creates a new engine 
  * @param input An input source to read to intermediate code from
  */
  void init(xabsl::InputSource& input);
  
  /** Executes the engine */
  void executeEngine();
  
  /** Registers symbols and basic behaviors at the engine */
  virtual void registerSymbolsAndBasicBehaviors() = 0;
  
  /** Sets the selected Agent. If the last selected agent was different from
  * the new one, the root option is changed depending on the new agent.
  * @param name The name of the agent
  * @return if the agent was set
  */
  bool setSelectedAgent(const char* name);
  
  /** 
  * Is called for every incoming debug message.
  * @param message An interface to read the message from the queue
  * @return if the messag was read
  */
  bool handleXabslMessage(InMessage& message);
  
protected:
  
  /** An engine that executes the XABSL formalized behaviors */
  xabsl::Engine* pEngine;
  
  /** Is invoked when errors occur */
  GTXabslErrorHandler errorHandler;
  
  /** Is called if the engine could not be created */
  virtual void executeIfEngineCouldNotBeCreated() = 0;

  /** 
  * Prints the main action that was generated by the execution of the engine to a string
  * @param buf the string where to print the action
  */
  virtual void printGeneratedMainActionToString(char* buf) const = 0;

private:
  
  /** The id of the xabsl::Engine derivate. */
  const char* id;

  
  //!@name Debug interface to the Xabsl Dialog
  //!@{
  
  void sendActiveOptionsToStream(Out &out) const;

  /** Sends a debug message to the Xabsl dialog depending on the last request */
  void sendDebugMessage(Out &out) const;
  
  /** Sends a debug message to the Xabsl dialog containing names of agents, options, basic behaviors, and symbols*/
  void sendDebugSymbols(Out &out) const;

  /** The decimal input symbols that are watched by the Xabsl Dialog */
  xabsl::Array<const xabsl::DecimalInputSymbol*> watchedDecimalInputSymbols;
  
  /** The boolean input symbols that are watched by the Xabsl Dialog */
  xabsl::Array<const xabsl::BooleanInputSymbol*> watchedBooleanInputSymbols;
  
  /** The enumerated input symbols that are watched by the Xabsl Dialog */
  xabsl::Array<const xabsl::EnumeratedInputSymbol*> watchedEnumeratedInputSymbols;
  
  /** The decimal output symbols that are watched by the Xabsl Dialog */
  xabsl::Array<const xabsl::DecimalOutputSymbol*> watchedDecimalOutputSymbols;

  /** The boolean output symbols that are watched by the Xabsl Dialog */
  xabsl::Array<const xabsl::BooleanOutputSymbol*> watchedBooleanOutputSymbols;

  /** The enumerated output symbols that are watched by the Xabsl Dialog */
  xabsl::Array<const xabsl::EnumeratedOutputSymbol*> watchedEnumeratedOutputSymbols;
  
  /** The decimal output symbols that are set from the Xabsl Dialog */
  xabsl::Array<xabsl::DecimalOutputSymbol*> setDecimalOutputSymbols;

  /** The values for the set decimal output symbols */
  xabsl::Array<double> setDecimalOutputSymbolValues;

  /** The boolean output symbols that are set from the Xabsl Dialog */
  xabsl::Array<xabsl::BooleanOutputSymbol*> setBooleanOutputSymbols;

  /** The values for the set boolean output symbols */
  xabsl::Array<bool> setBooleanOutputSymbolValues;

  /** The output symbols that are set from the Xabsl Dialog */
  xabsl::Array<xabsl::EnumeratedOutputSymbol*> setEnumeratedOutputSymbols;

  /** The values for the set output symbols */
  xabsl::Array<int> setEnumeratedOutputSymbolValues;

  //!@}
};


#endif// __GTXabslEngineExecutor_h_
