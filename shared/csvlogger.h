/*
	Copyright 2011, Oliver Urbann
	All rights reserved.

	This file is part of MoToFlex.

    MoToFlex is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    MoToFlex is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MoToFlex.  If not, see <http://www.gnu.org/licenses/>.

	Contact e-mail: oliver.urbann@tu-dortmund.de
*/

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

#ifndef LOGGING
#define LOG(titel, name, data) /**/
#define MARK(titel, name) /**/
#define FLUSH /**/
#else
#define LOG(titel, name, data) CSVLogger::log(titel, name, data)
#define MARK(titel, name) CSVLogger::mark(titel, name)
#define FLUSH CSVLogger::flush();
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

	static string basepath;
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
