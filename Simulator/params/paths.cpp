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
#include "paths.h"

#include <string.h>

Paths::Paths(void)
{
}

Paths::~Paths(void)
{
}


bool Paths::load(const char *path)
{
	if (!open(path))
		return false;
	paramsPath=readString();

	for (int i=0; i<NUM_OF_NAOS; i++)
		behPath[i]=readString();

	outputPath=readString();

	fitnessPath=readString();

	for (int i=0; i<NUM_OF_NAOS; i++)
		targetPath[i]=readString();

	close();
	
	return true;
}
