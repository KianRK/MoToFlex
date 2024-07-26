#pragma once

#include <fstream>
#include <string>
#include <iostream>
#include <sstream>

class DebugSymbolsLog
{
public:
	DebugSymbolsLog(void);	 
	~DebugSymbolsLog(void);

	static void initialize(bool);

	// Printouts
	static void print(double toWriteString);
	static void print(std::string toWriteString);
	static void print(int toWriteString);
	static void printWithoutSeperator(std::string toWriteString);
	static void printStaticSymbol(std::string name, double value);
	static void printStaticSymbol(std::string name, int value);
	static void printStaticSymbol(std::string name, std::string value);
	static void printSeparator();
	static void printSeparatorSign(std::string toWriteChar);
	static void printTimestamp();
	static void debugPrintTimestamp();
	static void plottAndNewLine();
	static void finalize();


private:
	static std::ofstream oDebug;
	static std::ostringstream bufferStream;
	static std::string path;
	static bool shellIPrint;
	//Time-Logging
	static double tval();
	static bool firstTimeTimeCheck;
	static int frameCounter;
};


/************************************************************************/
/* This is the old DebugSymbolsLLog                                     */
/************************************************************************/
/*
class DebugSymbolsLog
{
public:
	DebugSymbolsLog(void);	 
	~DebugSymbolsLog(void);

	static void initialize(bool);

	// Printouts
	static void print(double toWriteString) 
		{if (DebugSymbolsLog::shellIPrint) oDebug << toWriteString<< ";";}
	static void print(std::string toWriteString) 
		{if (DebugSymbolsLog::shellIPrint) oDebug << toWriteString<<";";}
	static void print(int toWriteString) 
		{if (DebugSymbolsLog::shellIPrint) oDebug << toWriteString<<";";}
	static void printWithoutSeperator(std::string toWriteString) 
		{if (DebugSymbolsLog::shellIPrint) oDebug << toWriteString;}
	static void printStaticSymbol(std::string name, double value) 
		{if (DebugSymbolsLog::shellIPrint) oDebug << name<< ":"<<value<<";";}
	static void printStaticSymbol(std::string name, int value) 
		{if (DebugSymbolsLog::shellIPrint) oDebug << name<< ":"<<value<<";";}
	static void printStaticSymbol(std::string name, std::string value) 
		{if (DebugSymbolsLog::shellIPrint) oDebug << name<< ":"<<value<<";";}
	static void printSeparator() 
		{if (DebugSymbolsLog::shellIPrint) oDebug << ";"; }
	static void printSeparatorSign(std::string toWriteChar)  
		{if (DebugSymbolsLog::shellIPrint) oDebug << toWriteChar; }
	static void printTimestamp()
		{if (DebugSymbolsLog::shellIPrint) DebugSymbolsLog::oDebug<<tval()<<";";}
	static void debugPrintTimestamp()
		{if (DebugSymbolsLog::shellIPrint) DebugSymbolsLog::oDebug<<tval()<<"\n"; }
	static void finalize()
		{DebugSymbolsLog::oDebug.close();}


private:
	static std::ofstream oDebug;
	static bool shellIPrint;
	//Time-Logging
	static double tval();
	static bool firstTimeTimeCheck;
};
*/
