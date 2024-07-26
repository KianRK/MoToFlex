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

Paths::Paths(void)
{
}

Paths::~Paths(void)
{
}

void Paths::setBasePath(string path)
{
	this->path=path;
}

void Paths::setParameterFn(string fn)
{
	parmFn=fn;
	parmFn+=".gnm";
}

void Paths::setBehaviourFn(int naoNum, string fn)
{
	behFn[naoNum]=fn;
	behFn[naoNum]+=".csv";
}

void Paths::setLogFn(string fn)
{
	logFn=fn;
	logFn+=".csv";
}

void Paths::setFitnessFn(string fn)
{
	FitFn=fn;
	FitFn+=".ftn";
}

void Paths::setTarget(int naoNum, string fn)
{
	tgtFn[naoNum]=fn;
	tgtFn[naoNum]+=".csv";
}

void Paths::write(string fn)
{
	string fullpath=getFullCfgPath(fn);
	open(fullpath.c_str(), true);
	writeString("Parameters", getFullParameterPath());
	for (int i=0; i<NUM_OF_NAOS; i++)
		writeString("Behaviour", getFullBehaviourPath(i));
	writeString("Logfile", getFullLogPath());
	writeString("Fitness", getFullFitnessPath());
	for (int i=0; i<NUM_OF_NAOS; i++)
		writeString("Target", getFullTargetPath(i));
	close();
}

