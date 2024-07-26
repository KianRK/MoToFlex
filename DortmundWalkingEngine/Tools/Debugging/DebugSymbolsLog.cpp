#include "DebugSymbolsLog.h"
#include <time.h>
#include "Platform/Win32/File.h"
#include <stdlib.h>

using namespace std;

std::ofstream DebugSymbolsLog::oDebug;
bool DebugSymbolsLog::shellIPrint = 0;
int DebugSymbolsLog::frameCounter = 0;
std::ostringstream DebugSymbolsLog::bufferStream(std::ostringstream::out);
std::string DebugSymbolsLog::path;

// should be called explicitly
DebugSymbolsLog::DebugSymbolsLog(void)
{
}

// A Debug will be only done, when advised to
void DebugSymbolsLog::initialize(bool shellIPrint = false)
{
	DebugSymbolsLog::shellIPrint = shellIPrint;
	if (shellIPrint)
	{		
		bool notFound=true;
		int counter = 0;
		char buf[8];

		string basicPath = File::getGTDir();
		basicPath +="/Config/Logs/";
		while (notFound){
			string path = basicPath;
			path += "SymblosLog_";
			#ifdef _WIN32
				_snprintf(buf, 8, "%d", counter);
			#else
				snprintf(buf, 8, "%d", counter);
			#endif
			path += buf;
			path += ".xlg";

			//Check, wether File exists
			ifstream inp;
			inp.open(path.c_str(), ifstream::in);
			inp.close();
			if(inp.fail())
			{
				inp.clear(ios::failbit);
				DebugSymbolsLog::oDebug.open(path.c_str(), ofstream::out);
				break;
			}
			else counter++;
			if (counter > 1000){
				{ cout << "Fehler beim Loggen. Es gibt (angeblich) mehr als 1000 Logs" << endl; 
				break;}
			}
		}
	}
}

DebugSymbolsLog::~DebugSymbolsLog(void)
{
	DebugSymbolsLog::oDebug.close();
}
/************************************************************************/
/* Write-Funcions*/
/************************************************************************/

void DebugSymbolsLog::print(double toWriteString) 
	{if (DebugSymbolsLog::shellIPrint) DebugSymbolsLog::bufferStream << toWriteString<< ";";}
void DebugSymbolsLog::print(std::string toWriteString) 
	{if (DebugSymbolsLog::shellIPrint) DebugSymbolsLog::bufferStream << toWriteString<<";";}
void DebugSymbolsLog::print(int toWriteString) 
	{if (DebugSymbolsLog::shellIPrint) DebugSymbolsLog::bufferStream << toWriteString<<";";}
void DebugSymbolsLog::printWithoutSeperator(std::string toWriteString) 
	{if (DebugSymbolsLog::shellIPrint) DebugSymbolsLog::bufferStream << toWriteString;}
void DebugSymbolsLog::printStaticSymbol(std::string name, double value) 
	{if (DebugSymbolsLog::shellIPrint) DebugSymbolsLog::bufferStream << name<< ":"<<value<<";";}
void DebugSymbolsLog::printStaticSymbol(std::string name, int value) 
	{if (DebugSymbolsLog::shellIPrint) DebugSymbolsLog::bufferStream << name<< ":"<<value<<";";}
void DebugSymbolsLog::printStaticSymbol(std::string name, std::string value) 
	{if (DebugSymbolsLog::shellIPrint) DebugSymbolsLog::bufferStream << name<< ":"<<value<<";";}
void DebugSymbolsLog::printSeparator() 
	{if (DebugSymbolsLog::shellIPrint) DebugSymbolsLog::bufferStream << ";"; }
void DebugSymbolsLog::printSeparatorSign(std::string toWriteChar)  
	{if (DebugSymbolsLog::shellIPrint) DebugSymbolsLog::bufferStream << toWriteChar; }
void DebugSymbolsLog::printTimestamp()
	{if (DebugSymbolsLog::shellIPrint) DebugSymbolsLog::bufferStream<<tval()<<";";}
void DebugSymbolsLog::debugPrintTimestamp()
	{if (DebugSymbolsLog::shellIPrint) DebugSymbolsLog::bufferStream<<tval()<<"\n"; }
void DebugSymbolsLog::plottAndNewLine()
{
	if (DebugSymbolsLog::shellIPrint)
	{
/*
		if (DebugSymbolsLog::frameCounter == 3)
		{
			DebugSymbolsLog::bufferStream << "\n";
			DebugSymbolsLog::frameCounter++;
		}
		else
		{
*/
			DebugSymbolsLog::bufferStream << "\n";
			DebugSymbolsLog::oDebug << DebugSymbolsLog::bufferStream.str();
			DebugSymbolsLog::bufferStream.str("");
			DebugSymbolsLog::bufferStream.clear();
/*
			DebugSymbolsLog::frameCounter = 0;
		}
*/
	}
}
void DebugSymbolsLog::finalize()
	{if (DebugSymbolsLog::shellIPrint) DebugSymbolsLog::oDebug.close();}

/************************************************************************/
/* TIMING                                                               */
/************************************************************************/
bool DebugSymbolsLog::firstTimeTimeCheck = true;

#ifdef _WIN32
#include <windows.h>

static LARGE_INTEGER time1;
static LARGE_INTEGER time2;
static LARGE_INTEGER tStart;
static LARGE_INTEGER freq;

double DebugSymbolsLog::tval(){
	//measure
	if(DebugSymbolsLog::firstTimeTimeCheck) {
		QueryPerformanceFrequency(&freq);
		QueryPerformanceCounter(&time1);
		time2 = time1;
		DebugSymbolsLog::firstTimeTimeCheck = false;
	}
	else QueryPerformanceCounter(&time2);

	//evaluate
	double returnvalue = ((double)time1.QuadPart -
		(double)time2.QuadPart)/((double)freq.QuadPart);

	// prepare for next
	return -returnvalue;
}
#else
#include <sys/time.h>
struct timeval time1Lin, time2Lin;
struct timezone tz;

double DebugSymbolsLog::tval()
{
	//measure
	if(DebugSymbolsLog::firstTimeTimeCheck) {
		gettimeofday(&time1Lin, &tz);
		DebugSymbolsLog::firstTimeTimeCheck = 0;
	}
	gettimeofday(&time2Lin, &tz);

	double t1, t2;


	t1 =  (double)time1Lin.tv_sec + (double)time1Lin.tv_usec/(1000*1000);
	t2 =  (double)time2Lin.tv_sec + (double)time2Lin.tv_usec/(1000*1000);

	//prepare next value and return;
	return t2-t1;
}
#endif
