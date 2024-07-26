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

using namespace std;

#ifdef WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#ifdef WIN32
#define OS	0
#define POPEN _popen
#else
#include <unistd.h>
#define OS	1
#define POPEN popen
#endif

void ws_sleep(int ms);

string ws_execute(string command);

void ws_delete(string file);

void ws_copy(string from, string to);

bool ws_exists(string file);
