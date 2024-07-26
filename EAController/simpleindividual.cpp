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

#include "simpleindividual.h"

using namespace EAController;

#define POS_MUTATE(name) mutate(genom.name, variances.name, true)
#define MUTATE(name) mutate(genom.name, variances.name)
#define COPY(param,index)  \
	genom.param index=parents[getPrntNum(parents.size())]->genom.param index;

#include "mutate_macro.h"
#include "copy_macro.h"

void SimpleIndividual::mutate(double &value, double variance, bool positive)
{
	double oldv=value;
	if (variance==0)
		return;

	if (!positive)
		value+=getNormalDistribution(0, variance);
	else
	{
		//value*=getLogNormalDistribution(0, variance);
		//do 
		{
			value=oldv;
			value+=getNormalDistribution(0, variance); 
		}
		//while (value<0);
	}

	if (value!=value)
		value=oldv;
	
}


void SimpleIndividual::mutate(MutationInformation *mutInf)
{
	MUTATE_ALL
}


void SimpleIndividual::combine(IndividualList &parents)
{
	COPY_ALL
}
