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

#include "compat.h"
#include <stdlib.h>
#include "eaconfig.h"

void ws_sleep(int ms)
{
#ifdef WIN32
	Sleep(ms);
#else
	usleep(ms*1000);
#endif
}

string ws_execute(string command)
{
	FILE* fp;
    char result [1000];
    fp = POPEN(command.c_str(),"r");
    fread(result,1,sizeof(result),fp);
    fclose(fp);
	for (int i=0; i<sizeof(result); i++)
	{
		if (result[i]=='\r' || result[i]=='\n')
			result[i]=0;
	}
	string ret=result;
    return ret;
}

void ws_delete(string file)
{
#ifndef WIN32
	remove(file.c_str());
#endif
}

void ws_copy(string from, string to)
{
#ifndef WIN32
	string command="cp -f ";
	command+=from+" ";
	command+=to;
	ws_execute(command);
#endif
}

bool ws_exists(string file)
{
	FILE *f=fopen(file.c_str(), "r");
	if (f!=NULL)
	{
		fclose(f);
		return true;
	}
	else return false;
}
