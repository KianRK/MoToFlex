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
/**
 * @file speedfitness.h
 * Used to calculate the fitness of an individual.
 * @author <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a>
 */

#pragma once
#include <stdlib.h>
#include <stdio.h>


class SpeedFitness
{
public:
	SpeedFitness();
	~SpeedFitness(void);
	void addValues(float *targetAngles, float *realAngles, int numOfAngles, float speed);
	void setPath(const char *path);
	void done(int curFrame);
	double getFitness(int curFrame);
private:
	void write(float fitness);
	double fitness;
	FILE *f;
	char path[1024];
	int frames;
};

