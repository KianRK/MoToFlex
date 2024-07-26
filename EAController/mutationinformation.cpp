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

#include "mutationinformation.h"
#include "generation.h"
#include <limits>

void MutationInformation::collect(Generation &gen)
{
	meanFitness=0;
	int finishedCounter=0;
	for (Generation::IndividualIterator it=gen.individuals.begin(); it!=gen.individuals.end(); it++)
	{
		if ((*it)->state==EAController::Individual::finished)
		{
			double fitness=(*it)->getFitness();
			if (fitness!=std::numeric_limits<float>::infinity())
			{
				meanFitness+=fitness;
				finishedCounter++;
			}
		}
	}

	meanFitness/=finishedCounter;

	if (firstFitness==-1)
		firstFitness=meanFitness;
}

double MutationInformation::firstFitness=-1;
