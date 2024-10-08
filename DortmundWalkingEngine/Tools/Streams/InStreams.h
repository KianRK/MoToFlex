/**
* @file InStreams.h
*
* Declaration of in stream classes for different media and formats.
*
* @author Thomas R�fer
* @author Martin L�tzsch
*/

#ifndef __InStreams_h_
#define __InStreams_h_

#include "InOut.h"
#include "Platform/File.h"
#include <stdlib.h>

/** 
* @class PhysicalInStream
*
* The base class for physical in streams. Derivates of PhysicalInStream only handle the 
* reading of data from a medium, not of formating data.
*/
class PhysicalInStream
{
public:
  /**
  * The function reads a number of bytes from a stream.
  * @param p The address the data is written to. Note that p
  *          must point to a memory area that is at least
  *          "size" bytes large.
  * @param size The number of bytes to be read.
  */
  virtual void readFromStream(void* p,int size) = 0;
  
  /**
  * The function skips a number of bytes in a stream.
  * @param size The number of bytes to be read.
  */
  virtual void skipInStream(int size);
  
  /**
  * The function states whether this stream actually exists.
  * This function is relevant if the stream represents a file.
  * @return Does the stream exist?
  */
  virtual bool exists() const {return true;};
  
  /**
  * The function states whether the end of the stream has been reached.
  * @return End of stream reached?
  */
  virtual bool getEof() const = 0;
};

/**
* @class StreamReader
*
* Generic class for formated reading of data to be used in streams. 
* The physical reading is then done by PhysicalOutStream derivates.
*/
class StreamReader
{
protected:
  /**
  * reads a character from a stream
  * @param d the data to read from the stream
  * @param stream the stream to read from 
  */
  virtual void readChar(char& d, PhysicalInStream& stream) = 0;
  
  /**
  * reads a unsigned character from a stream
  * @param d the data to read from the stream
  * @param stream the stream to read from 
  */
  virtual void readUChar(unsigned char& d, PhysicalInStream& stream) = 0;
  
  /**
  * reads a short from a stream
  * @param d the data to read from the stream
  * @param stream the stream to read from 
  */
  virtual void readShort(short& d, PhysicalInStream& stream) = 0;
  
  /**
  * reads a unsigned short from a stream
  * @param d the data to read from the stream
  * @param stream the stream to read from 
  */
  virtual void readUShort(unsigned short& d, PhysicalInStream& stream) = 0;
  
  /**
  * reads a int from a stream
  * @param d the data to read from the stream
  * @param stream the stream to read from 
  */
  virtual void readInt(int& d, PhysicalInStream& stream) = 0;
  
  /**
  * reads a unsigned int from a stream
  * @param d the data to read from the stream
  * @param stream the stream to read from 
  */
  virtual void readUInt(unsigned int& d, PhysicalInStream& stream) = 0;
  
  /**
  * reads a long from a stream
  * @param d the data to read from the stream
  * @param stream the stream to read from 
  */
  virtual void readLong(long& d, PhysicalInStream& stream) = 0;
  
  /**
  * reads a unsigned long from a stream
  * @param d the data to read from the stream
  * @param stream the stream to read from 
  */
  virtual void readULong(unsigned long& d, PhysicalInStream& stream) = 0;
  
  /**
  * reads a float from a stream
  * @param d the data to read from the stream
  * @param stream the stream to read from 
  */
  virtual void readFloat(float& d, PhysicalInStream& stream) = 0;
  
  /**
  * reads a double from a stream
  * @param d the data to read from the stream
  * @param stream the stream to read from 
  */
  virtual void readDouble(double& d, PhysicalInStream& stream) = 0;
  
  /**
  * reads a string from a stream
  * @param d the data to read from the stream
  * @param stream the stream to read from 
  */
  virtual void readString(std::string& d, PhysicalInStream& stream) = 0;
  
  /**
  * reads the 'end of line' from a stream
  * @param stream the stream to read from 
  */
  virtual void readEndl(PhysicalInStream& stream) = 0;
  
  /**
  * The function reads a number of bytes from the file.
  * @param p The address the data is written to. Note that p
  *          must point to a memory area that is at least
  *          "size" bytes large.
  * @param size The number of bytes to be read.
  * @param stream The stream to read from.
  */
  virtual void readData(void* p,int size, PhysicalInStream& stream) = 0;

  /**
  * The function skips a number of bytes in the file.
  * @param size The number of bytes to be skipped.
  * @param stream The stream to read from.
  */
  virtual void skipData(int size, PhysicalInStream& stream);

  /**
  * The function states whether the end of the stream has been reached.
  * @param stream The stream to be tested 
  * @return End of stream reached?
  */
  virtual bool isEof(const PhysicalInStream& stream) const = 0;
};

/**
* @class InStream
*
* Generic class for classes that do both formated and physical reading of data from streams.
*/
template <class S, class R> class InStream : public S, public R, public In
{
public:
  /** Standard constructor */
  InStream() {};
  
  /**
  * The function reads a number of bytes from a stream.
  * @param p The address the data is written to. Note that p
  *          must point to a memory area that is at least
  *          "size" bytes large.
  * @param size The number of bytes to be read.
  */
  virtual void read(void* p,int size)
  { readData(p, size, *this); }
  
  /**
  * The function skips a number of bytes in the stream.
  * @param size The number of bytes to be skipped.
  */
  virtual void skip(int size)
  { skipData(size, *this); }

  /**
  * Determines whether the end of file has been reached.
  */ 
  virtual bool eof() const { return isEof(*this); }
  
protected:
  /**
  * Virtual redirection for operator>>(char& value).
  */
  virtual void inChar(char& d) 
  { readChar(d, *this); }
  
  /**
  * Virtual redirection for operator>>(unsigend char& value).
  */
  virtual void inUChar(unsigned char& d) 
  { readUChar(d, *this); }
  
  /**
  * Virtual redirection for operator>>(short& value).
  */
  virtual void inShort(short& d) 
  { readShort(d, *this); }
  
  /**
  * Virtual redirection for operator>>(unsigend short& value).
  */
  virtual void inUShort(unsigned short& d) 
  { readUShort(d, *this); }
  
  /**
  * Virtual redirection for operator>>(int& value).
  */
  virtual void inInt(int& d) 
  { readInt(d, *this); }
  
  /**
  * Virtual redirection for operator>>(unsigend int& value).
  */
  virtual void inUInt(unsigned int& d) 
  { readUInt(d, *this); }
  
  /**
  * Virtual redirection for operator>>(long& value).
  */
  virtual void inLong(long& d) 
  { readLong(d, *this); }
  
  /**
  * Virtual redirection for operator>>(unsigend long& value).
  */
  virtual void inULong(unsigned long& d) 
  { readULong(d, *this); }
  
  /**
  * Virtual redirection for operator>>(float& value).
  */
  virtual void inFloat(float& d) 
  { readFloat(d, *this); }
  
  /**
  * Virtual redirection for operator>>(double& value).
  */
  virtual void inDouble(double& d) 
  { readDouble(d, *this); }
  
  /**
  * Virtual redirection for operator>>(std::string& value).
  */
  virtual void inString(std::string& d) 
  { readString(d, *this); }
  
  /**
  * Virtual redirection for operator>>(In& (*f)(In&)) that reads
  * the symbol "endl";
  */
  virtual void inEndL() 
  { readEndl(*this); }
};

/**
* @class InText
*
* Formated reading of text data to be used in streams. 
* The physical reading is done by PhysicalInStream derivates.
*/
class InText : public StreamReader
{
private:
  std::string buf; /**< A buffer to convert read strings. */
  bool eof, /**< Stores whether the end of file was reached during the last call to nextChar. */
       nextEof;

public:
  /** Default constructor */
  InText() { buf.reserve(200); reset(); };
  
  /** Resets theChar to be able to use the same instance of InText or InConfig
  * more than once. 
  */
  void reset()
   { theChar = theNextChar = ' '; eof = nextEof = false;}


protected:
  /** The last character read. */
  char theChar, 
       theNextChar;

  /**
  * The function initializes the end-of-file flag.
  * It has to be called only once after the stream was initialized.
  * @param stream The stream.
  */
  virtual void initEof(PhysicalInStream& stream) 
  {
    eof = nextEof = stream.getEof();
    if(stream.exists())
      nextChar(stream);
  }
  
  /**
  * The function returns whether the end of stream has been reached.
  * If this function returns false, "theChar" is valid, otherwise it is not.
  * @param stream The stream.
  * @return End of stream reached?
  */
  virtual bool isEof(const PhysicalInStream& stream) const {return eof;}
  
  /**
  * reads a character from a stream
  * @param d the data to read from the stream
  * @param stream the stream to read from 
  */
  virtual void readChar(char& d, PhysicalInStream& stream)
  { readString(buf,stream); d = (char)strtol(buf.c_str(),(char**)NULL,0); }
  
  /**
  * reads a unsigned character from a stream
  * @param d the data to read from the stream
  * @param stream the stream to read from 
  */
  virtual void readUChar(unsigned char& d, PhysicalInStream& stream) 
  { readString(buf, stream); d = (unsigned char)strtoul(buf.c_str(),(char**)NULL,0); }
  
  /**
  * reads a short from a stream
  * @param d the data to read from the stream
  * @param stream the stream to read from 
  */
  virtual void readShort(short& d, PhysicalInStream& stream)
  { readString(buf, stream); d = (short)strtol(buf.c_str(),(char**)NULL,0); }
  
  /**
  * reads a unsigned short from a stream
  * @param d the data to read from the stream
  * @param stream the stream to read from 
  */
  virtual void readUShort(unsigned short& d, PhysicalInStream& stream) 
  {readString(buf, stream); d = (unsigned short)strtoul(buf.c_str(),(char**)NULL,0);}
  
  /**
  * reads a int from a stream
  * @param d the data to read from the stream
  * @param stream the stream to read from 
  */
  virtual void readInt(int& d, PhysicalInStream& stream)
  {readString(buf, stream); d = (int)strtol(buf.c_str(),(char**)NULL,0);}
  
  
  /**
  * reads a unsigned int from a stream
  * @param d the data to read from the stream
  * @param stream the stream to read from 
  */
  virtual void readUInt(unsigned int& d, PhysicalInStream& stream) 
  {readString(buf, stream); d = (unsigned int)strtoul(buf.c_str(),(char**)NULL,0);}
  
  /**
  * reads a long from a stream
  * @param d the data to read from the stream
  * @param stream the stream to read from 
  */
  virtual void readLong(long& d, PhysicalInStream& stream)
  {readString(buf, stream); d = (long)strtol(buf.c_str(),(char**)NULL,0);}
  
  /**
  * reads a unsigned long from a stream
  * @param d the data to read from the stream
  * @param stream the stream to read from 
  */
  virtual void readULong(unsigned long& d, PhysicalInStream& stream) 
  {readString(buf, stream); d = (unsigned long)strtoul(buf.c_str(),(char**)NULL,0);}
  
  /**
  * reads a float from a stream
  * @param d the data to read from the stream
  * @param stream the stream to read from 
  */
  virtual void readFloat(float& d, PhysicalInStream& stream) 
  {readString(buf, stream); d = float(atof(buf.c_str()));}
  
  /**
  * reads a double from a stream
  * @param d the data to read from the stream
  * @param stream the stream to read from 
  */
  virtual void readDouble(double& d, PhysicalInStream& stream)
  {readString(buf, stream); d = atof(buf.c_str());}
  
  /**
  * The function reads a string from a stream.
  * It skips all whitespace characters, and then reads
  * a sequence of non-whitespace characters to a buffer, until it 
  * again recognizes a whitespace.
  * @param d The value that is read.
  * @param stream the stream to read from
  */
  virtual void readString(std::string& d, PhysicalInStream& stream);
  
  /**
  * reads the 'end of line' from a stream
  * @param stream the stream to read from 
  */
  virtual void readEndl(PhysicalInStream& stream) {};
  
  /**
  * The function determines whether the current character is a whitespace.
  */
  virtual bool isWhitespace();
  
  /**
  * The function skips the whitespace.
  */
  virtual void skipWhitespace(PhysicalInStream& stream);
  
  /**
  * The function reads the next character from the stream.
  */
  virtual void nextChar(PhysicalInStream& stream) 
  {
    if(!eof)
    {
      eof = nextEof;
      theChar = theNextChar;
      if(stream.getEof())
      {
        nextEof = true;
        theNextChar = ' ';
      }
      else
        stream.readFromStream(&theNextChar, 1);
    }
  }
  
  /**
  * The function reads a number of bytes from the file.
  * @param p The address the data is written to. Note that p
  *          must point to a memory area that is at least
  *          "size" bytes large.
  * @param size The number of bytes to be read.
  * @param stream The stream to read from.
  */
  virtual void readData(void* p,int size, PhysicalInStream& stream);
};

/**
* The class InConfig reads text data from config (file) streams
* that contain comments and sections.
* The following comment styles are supported:
* / * comment * / (ignore the space between "*" and "/")
* // comment till endl
* # comment till endl
* Note that "/" is not allowed elsewhere in the stream.
*/
class InConfig : public InText
{
public:
  /** 
  * Default constructor
  */
  InConfig() : readSection(false) {};
  
protected:
  /**
  * Creates the reader.
  * @param sectionName If given the section is searched
  * @param stream The medium that should be read from.
  */
  void create(const std::string& sectionName, PhysicalInStream& stream);
  
  /**
  * The function determines whether the current character is a whitespace.
  * In this context, the start of 
  */
  virtual bool isWhitespace();
  
  /**
  * The function skips the whitespace.
  */
  virtual void skipWhitespace(PhysicalInStream& stream);
  
  /**
  * The function reads the next character from the stream.
  */
  virtual void nextChar(PhysicalInStream& stream);

private:
  /** Are we reading a section? */
  bool readSection; 
  
  /**
  * The functions skip all characters to the end of the line.
  */
  void skipLine(PhysicalInStream& stream);
  
  /**
  * The functions skip all characters to the end of the comment.
  */
  void skipComment(PhysicalInStream& stream);
};

/**
* @class InBinary
*
* Formated reading of binary data to be used in streams. 
* The physical reading is done by PhysicalInStream derivates.
*/
class InBinary : public StreamReader
{
protected:
  /**
  * The function returns whether the end of stream has been reached.
  * @return End of stream reached?
  */
  virtual bool isEof(const PhysicalInStream& stream) const {return stream.getEof();}

  /**
  * The function reads a char from the stream.
  * @param d The value that is read.
  * @param stream A Stream to read from.
  */
  virtual void readChar(char& d, PhysicalInStream& stream)
  {stream.readFromStream(&d,sizeof(d));}
  
  /**
  * The function reads an unsigned char from the stream.
  * @param d The value that is read.
  * @param stream A Stream to read from.
  */
  virtual void readUChar(unsigned char& d, PhysicalInStream& stream)
  {stream.readFromStream(&d,sizeof(d));}
  
  /**
  * The function reads a short int from the stream.
  * @param d The value that is read.
  * @param stream A Stream to read from.
  */
  virtual void readShort(short& d, PhysicalInStream& stream) 
  {stream.readFromStream(&d,sizeof(d));}
  
  /**
  * The function reads an unsigned short int from the stream.
  * @param d The value that is read.
  * @param stream A Stream to read from.
  */
  virtual void readUShort(unsigned short& d, PhysicalInStream& stream) 
  {stream.readFromStream(&d,sizeof(d));}
  
  /**
  * The function reads an int from the stream.
  * @param d The value that is read.
  * @param stream A Stream to read from.
  */
  virtual void readInt(int& d, PhysicalInStream& stream) 
  {stream.readFromStream(&d,sizeof(d));}
  
  /**
  * The function reads an unsigned int from the stream.
  * @param d The value that is read.
  * @param stream A Stream to read from.
  */
  virtual void readUInt(unsigned int& d, PhysicalInStream& stream) 
  {stream.readFromStream(&d,sizeof(d));}
  
  /**
  * The function reads a long int from the stream.
  * @param d The value that is read.
  * @param stream A Stream to read from.
  */
  virtual void readLong(long& d, PhysicalInStream& stream) 
  {stream.readFromStream(&d,sizeof(d));}
  
  /**
  * The function reads an unsigned long int from the stream.
  * @param d The value that is read.
  * @param stream A Stream to read from.
  */
  virtual void readULong(unsigned long& d, PhysicalInStream& stream) 
  {stream.readFromStream(&d,sizeof(d));}
  
  /**
  * The function reads a float from the stream.
  * @param d The value that is read.
  * @param stream A Stream to read from.
  */
  virtual void readFloat(float& d, PhysicalInStream& stream) 
  {stream.readFromStream(&d,sizeof(d));}
  
  /**
  * The function reads a double from the stream.
  * @param d The value that is read.
  * @param stream A Stream to read from.
  */
  virtual void readDouble(double& d, PhysicalInStream& stream) 
  {stream.readFromStream(&d,sizeof(d));}
  
  /**
  * The function reads a string from the stream.
  * @param d The value that is read.
  * @param stream A Stream to read from.
  */
  virtual void readString(std::string& d, PhysicalInStream& stream) 
  {int size; stream.readFromStream(&size,sizeof(size)); d.resize(size); stream.readFromStream(&d[0],size);}
  
  /**
  * The function is intended to read an endl-symbol from the stream.
  * In fact, the function does nothing.
  * @param stream A Stream to read from.
  */
  virtual void readEndl(PhysicalInStream& stream) {};
  
  /**
  * The function reads a number of bytes from a stream.
  * @param p The address the data is written to. Note that p
  *          must point to a memory area that is at least
  *          "size" bytes large.
  * @param size The number of bytes to be read.
  * @param stream A Stream to read from.
  */
  virtual void readData(void* p,int size, PhysicalInStream& stream)
  { stream.readFromStream(p, size); }

  /**
  * The function skips a number of bytes in the file.
  * @param size The number of bytes to be skipped.
  * @param stream The stream to read from.
  */
  virtual void skipData(int size, PhysicalInStream& stream)
  { stream.skipInStream(size); }
};

/**
* @class InFile
*
* An PhysicalInStream that reads the data from a file.
*/
class InFile : public PhysicalInStream
{
private:
  File* stream; /**< Object representing the file. */
  
public:
  /** Default constructor */
  InFile() : stream(0) {};
  
  /** Destructor */
  ~InFile() { if (stream != 0) delete stream; }
  
  /**
  * The function states whether the file actually exists.
  * @return Does the file exist?
  */
  virtual bool exists() const 
  { return  (stream != 0 ? stream->exists() : false); }
  
  /**
  * The function states whether the end of the file has been reached.
  * @return End of file reached?
  */
  virtual bool getEof() const 
  { return (stream != 0 ? stream->eof() : false); }
  
protected:
  /**
  * opens the file.
  * @param name The name of the file to open. It will be interpreted
  *             as relative to the configuration directory.
  */
  void open(const std::string& name) 
  { if (stream == 0) stream = new File(name,"rb"); }
  
  /**
  * The function reads a number of bytes from the file.
  * @param p The address the data is written to. Note that p
  *          must point to a memory area that is at least
  *          "size" bytes large.
  * @param size The number of bytes to be read.
  */
  virtual void readFromStream(void* p,int size) 
  { if (stream != 0) stream->read(p,size); }
};

/**
* @class InMemory
*
* An PhysicalInStream that reads the data from a memory region.
*/
class InMemory : public PhysicalInStream
{
private:
  const char* memory, /**< Points to the next byte to read from memory. */
            * end; /**< Points to the end of the memory block. */
  
public:
  /** Default constructor */
  InMemory() : memory(0), end(0) {};
  
  /**
  * The function states whether the stream actually exists.
  * @return Does the stream exist? This is always true for memory streams.
  */
  virtual bool exists() const {return (memory != 0); }
  
  /**
  * The function states whether the end of the file has been reached.
  * It will only work if the correct size of the memory block was 
  * specified during the construction of the stream.
  * @return End of file reached?
  */
  virtual bool getEof() const 
  {return (memory != 0 ? memory >= end: false); }
  
protected:
  /**
  * Opens the stream.
  * @param mem The address of the memory block from which is read.
  * @param size The size of the memory block. It is only used to 
  *             implement the function eof(). If the size is not
  *             specified, eof() will always return true, but reading
  *             from the stream is still possible.
  */
  void open(const void* mem,unsigned size = 0) 
  { if (memory == 0) { memory = (const char*) mem; end = (const char*) mem + size; }}
  
  /**
  * The function reads a number of bytes from memory.
  * @param p The address the data is written to. Note that p
  *          must point to a memory area that is at least
  *          "size" bytes large.
  * @param size The number of bytes to be read.
  */
  virtual void readFromStream(void* p,int size);
  
  /**
  * The function skips a number of bytes.
  * @param size The number of bytes to be skipped.
  */
  virtual void skipInStream(int size) {memory += size;}
};

/**
* @class InBinaryFile
*
* A binary stream from a file.
*/
class InBinaryFile : public InStream<InFile,InBinary>
{
public:
  /**
  * Constructor.
  * @param name The name of the file to open. It will be interpreted
  *             as relative to the configuration directory.
  */
  InBinaryFile(const std::string& name) { open(name); }

  /**
  * The function returns whether this is a binary stream.
  * @return Does it output data in binary format?
  */
  virtual bool isBinary() const {return true;}
};

/**
* @class InBinaryMemory
*
* A Binary Stream from a memory region.
*/
class InBinaryMemory : public InStream<InMemory,InBinary>
{
public:
  /**
  * Constructor.
  * @param mem The address of the memory block from which is read.
  * @param size The size of the memory block. It is only used to 
  *             implement the function eof(). If the size is not
  *             specified, eof() will always return true, but reading
  *             from the stream is still possible.
  */
  InBinaryMemory(const void* mem, unsigned size = 0) { open(mem, size); }

  /**
  * The function returns whether this is a binary stream.
  * @return Does it output data in binary format?
  */
  virtual bool isBinary() const {return true;}
};

/**
* @class InTextFile
*
* A binary stream from a file.
*/
class InTextFile : public InStream<InFile,InText>
{
public:
  /**
  * Constructor.
  * @param name The name of the file to open. It will be interpreted
  *             as relative to the configuration directory.
  */
  InTextFile(const std::string& name) { open(name); initEof(*this); }
};

/**
* @class InTextMemory
*
* A Binary Stream from a memory region.
*/
class InTextMemory : public InStream<InMemory,InText>
{
public:
  /**
  * Constructor.
  * @param mem The address of the memory block from which is read.
  * @param size The size of the memory block. It is only used to 
  *             implement the function eof(). If the size is not
  *             specified, eof() will always return true, but reading
  *             from the stream is still possible.
  */
  InTextMemory(const void* mem, unsigned size = 0) { open(mem, size); initEof(*this); }
};

/**
* @class InConfigFile
*
* A config-file-style-formated text stream from a file.
*/
class InConfigFile : public InStream<InFile,InConfig>
{
public:
  /**
  * Constructor.
  * @param name The name of the file to open. It will be interpreted
  *             as relative to the configuration directory. Note that
  *             the file is treated as binary file, in order
  *             to gain the same results on all supported platforms.
  * @param sectionName If given the section is searched
  */
  InConfigFile(const std::string& name, const std::string& sectionName = "")
  { open(name); initEof(*this); create(sectionName,*this); }
};

/**
* @class InConfigMemory
*
* A config-file-style-formated text stream from a memory region.
*/
class InConfigMemory : public InStream<InMemory,InConfig>
{
public:
  /**
  * Constructor.
  * @param mem The address of the memory block from which is read.
  * @param size The size of the memory block. It is only used to 
  *             implement the function eof(). If the size is not
  *             specified, eof() will always return true, but reading
  *             from the stream is still possible.
  * @param sectionName If given the section is searched
  */
  InConfigMemory(const void* mem, unsigned size = 0, const std::string& sectionName = "")
  { open(mem, size); initEof(*this); create(sectionName,*this); }
};

#endif //__InStreams_h_
