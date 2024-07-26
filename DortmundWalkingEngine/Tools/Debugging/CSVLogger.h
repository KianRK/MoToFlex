#pragma once

#include <string>
#include <sstream>
#include <fstream>
#include <list>

using namespace std;

#ifdef RELEASE
#undef LOGGING
#endif

// Use this to globally disable logging
// #undef LOGGING

/*

 Other method to disable all loggings:
 Search and replace by using this regular Expression:

 /*\//#define LOGGING

 reaplaced by:

 //#define LOGGING

 */

#ifndef LOGGING
#define LOG(titel, name, data) /**/
#define MARK(titel, name) /**/
#define FLUSH /**/
#else
#define LOG(titel, name, data) CSVLogger::log(titel, name, data)
#define MARK(titel, name) CSVLogger::mark(titel, name)
#define FLUSH	CSVLogger::flush();
#endif

class CSVLogger
{
public:
	CSVLogger(void);
	~CSVLogger(void);

  static void log(string titel, string name, string data);
	static void log(string titel, string name, unsigned int data);
	static void log(string titel, string name, double data);
	static void log(string titel, string name, unsigned long data);
	static void log(string titel, string name, long data);
	static void log(string titel, string name, int data);
	static void mark(string titel, string name);
	static void flush();
private:
	struct Column
	{
		string name;
		string data;
		bool filled;
	};

	typedef list<Column *> ColumnList;

	struct Logfile
	{
		ofstream s;
		string name;
		ColumnList columns;
		bool headerWritten;
		ColumnList::iterator lastFound;
	};

	typedef list<Logfile *> LogfileList;

	static LogfileList logs;

	static bool add(string titel, string name, string data);

	static Logfile *getFile(string name);
	static Column *getColumn(Logfile *f, string name);
	static Logfile *addFile(string name);
	static Column *addColumn(Logfile *f, string name);
	static void flush(Logfile *f);
	static bool disabled;
};
