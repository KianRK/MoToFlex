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
#include "anglefitness.h"
#include <limits>
#include <string.h>
#include "math.h"

AngleFitness::AngleFitness()
{				
	fitness=0;
	frames=0;
}

AngleFitness::~AngleFitness(void)
{
}

void AngleFitness::write(float fitness)
{
	if (strcmp(path, "")==0)
		return;
	do {
		f=fopen(path, "w");
	} while (f==NULL);

	fprintf(f, "fitness = %e", fitness);
	fclose(f);
}

void AngleFitness::setPath(const char *path)
{
	strcpy(this->path, path);
}

void AngleFitness::addValues(float *targetAngles, float *realAngles, int numOfAngles, float speed)
{
	frames++;
	for (int i=0; i<numOfAngles; i++)
	{
		fitness+=(targetAngles[i]-realAngles[i])*(targetAngles[i]-realAngles[i]);
	}
}

void AngleFitness::done(int curFrame)
{
	write((float)getFitness(curFrame));
}

double AngleFitness::getFitness(int curFrame)
{
	if (frames==0)
	   return (pow(10.0, 10.0)/curFrame);
	if (fitness==0)
		return std::numeric_limits<float>::infinity();
	return (fitness/pow(frames, 5.0));
}

