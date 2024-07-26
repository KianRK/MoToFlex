
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
#include "csvlogger.h"
#include <stdio.h>
#include <string.h>
#include <iostream>


CSVLogger::CSVLogger(void)
{

}

CSVLogger::~CSVLogger(void)
{
	
}

CSVLogger::LogfileList CSVLogger::logs;
bool CSVLogger::disabled=false;

void CSVLogger::log(string titel, string name, string data)
{
	if (disabled) return;
	stringstream s;
	s << data;
	add(titel, name, s.str());
}

void CSVLogger::log(string titel, string name, unsigned int data)
{
	if (disabled) return;
	stringstream s;
	s << data;
	add(titel, name, s.str());
}

void CSVLogger::log(string titel, string name, unsigned long data)
{
	if (disabled) return;
	stringstream s;
	s << data;
	add(titel, name, s.str());
}

void CSVLogger::log(string titel, string name, int data)
{
	if (disabled) return;
	stringstream s;
	s << data;
	add(titel, name, s.str());
}

void CSVLogger::log(string titel, string name, long data)
{
	if (disabled) return;
	stringstream s;
	s << data;
	add(titel, name, s.str());
}

void CSVLogger::log(string titel, string name, double data)
{
	if (disabled) return;
	stringstream s;
	s << data;
	add(titel, name, s.str());
}

void CSVLogger::mark(string titel, string name)
{
	if (disabled) return;
	Logfile *f=getFile(titel);
	if (f==NULL)
		f=addFile(titel);

	if (f->headerWritten)
		return;

	Column *c=getColumn(f, name);
	if (c==NULL)
		c=addColumn(f, name);
}


bool CSVLogger::add(string titel, string name, string data)
{
	Logfile *f=getFile(titel);
	if (f==NULL)
		f=addFile(titel);

	Column *c=getColumn(f, name);

	if (c==NULL)
	{
		if (f->headerWritten)
			return false;
		else
			c=addColumn(f, name);
	}

	if (c->filled)
		flush(f);

	c->data=data;
	c->filled=true;
	
	return true;
}

void CSVLogger::flush()
{
	if (disabled) return;
	for (LogfileList::iterator log=logs.begin(); log!=logs.end(); ++log)
	{
		flush(*log);
	}
}

void CSVLogger::flush(Logfile *f)
{
	if (disabled) return;
	if (!f->headerWritten)
	{
		for (ColumnList::iterator el=f->columns.begin(); el!=f->columns.end(); ++el)
		{
			f->s << (*el)->name;
			if ((*el)!=f->columns.back())
				f->s << ";";
			else
				f->s << "\n";
		}
		f->headerWritten=true;
	}

	for (ColumnList::iterator el=f->columns.begin(); el!=f->columns.end(); ++el)
	{
		f->s << (*el)->data;
		if ((*el)!=f->columns.back())
			f->s << ";";
		else
			f->s << "\n";
		(*el)->filled=false;
		(*el)->data="0";
	}
	f->s.flush();
}

CSVLogger::Logfile *CSVLogger::getFile(string name)
{
	for (LogfileList::iterator log=logs.begin(); log!=logs.end(); ++log)
	{
		if ((*log)->name==name)
			return *log;
	}	
	return NULL;
}

CSVLogger::Column *CSVLogger::getColumn(Logfile *f, string name)
{
	if (f->columns.size()==0)
		return NULL;

	ColumnList::iterator el=f->lastFound;
	for (int i=0; i<(int)f->columns.size(); i++, el++) 
	{
		if (el==f->columns.end())
			el=f->columns.begin();

		if ((*el)->name==name)
		{
			f->lastFound=el;
			return *el;
		}
	}
	return NULL;
}

CSVLogger::Logfile *CSVLogger::addFile(string name)
{
	
	Logfile *neu=new Logfile;
	string path="";
	path+=basepath;
	path+=name;
	path+=".csv";
	neu->s.open(path.c_str(), ios::out);
	neu->headerWritten=false;
	neu->name=name;
	logs.push_back(neu);
	return neu;
}

CSVLogger::Column *CSVLogger::addColumn(Logfile *f, string name)
{
	Column *neu=new Column;
	neu->filled=false;
	neu->data="";
	neu->name=name;
	f->columns.push_back(neu);
	if (f->columns.size()==1)
		f->lastFound=f->columns.begin();
	return neu;
}

string CSVLogger::basepath;
