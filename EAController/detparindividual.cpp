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

#include "detparindividual.h"
#include "generation.h"
#include <limits>

#define MUTATE(name) mutate(genom.name, variances.name, varianceFactor);
#define POS_MUTATE(name) mutate(genom.name, variances.name, varianceFactor, true);

#include "mutate_macro.h"

void DetParIndividual::mutate(double &value, double variance, double varianceFactor, bool positive)
{
	SimpleIndividual::mutate(value, variance*varianceFactor, positive);
}

void DetParIndividual::mutate(MutationInformation *mutInf)
{
	double varianceFactor=1;

	if (mutInf!=NULL)
	{
		varianceFactor=mutInf->meanFitness/MutationInformation::firstFitness;
	}

	MUTATE_ALL;
}

