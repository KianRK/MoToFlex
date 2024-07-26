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

#include "config.h"
#include "eaconfig.h"

#include <string.h>
#include <iostream>
#include <limits>


Config::Config(void)
{
	f=NULL;
}

Config::~Config(void)
{
}

bool Config::open(const char *path, bool write)
{
// If we are started automatically, we maybe have to 
// wait till the config is finally written.
#ifdef EA_MODE
	do {
#endif
		if (!write)
			f=fopen(path, "r");	
		else
			f=fopen(path, "w");	
		if (f==NULL)
			std::cout << "Failed to open " << path << std::endl;
#ifdef EA_MODE
	} while (f==NULL);
#endif
	if (f == NULL)
		return false;
	else return true;
}

void Config::close()
{
	fclose(f);
}

string Config::readString()
{
	char str[10000], val[1000];
	val[0]=0;
	string realstr;
	fscanf(f, "%s = %s", str, val);
	realstr=val;
	return realstr;
}

void Config::writeString(string option, string value)
{
	fprintf(f, "%s = %s\n", option.c_str(), value.c_str());
}


void Config::writeDouble(string name, double val)
{
	fprintf(f, "%s = %e\n", name.c_str(), val);
}
 

double Config::readDouble()
{
	char value[200], *stopstring, varname[200];
	fscanf(f, "%s = %s", varname, value);
	if (strcmp(value, "1.#INF00e+000")==0 || strcmp(value, "inf")==0 )
		return std::numeric_limits<float>::infinity();
	return strtod(value, &stopstring);	
}
