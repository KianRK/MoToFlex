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

#include "selfadaptindividual.h"
#include "eaconfig.h"
#include <cmath>


extern EAParams eaParams;

#define POS_MUTATE(name) mutate(genom.name, variances.name, true); 
#define MUTATE(name) mutate(genom.name, variances.name); 
#define COPY(param,index)  \
{int param##Source=getPrntNum(parents.size()); \
	genom.param index=parents[param##Source]->genom.param index;	\
	variances.param index=parents[param##Source]->variances.param index;}

#include "mutate_macro.h"
#include "copy_macro.h"

void SelfAdaptIndividual::mutate(double &value, double &variance, bool positive)
{
	if (variance==0)
		return;
	double r1=1/sqrt((float)2*NUM_OF_PARAMS);
	double r2=1/sqrt(2*sqrt((float)NUM_OF_PARAMS));
	variance*=exp(r1*Individual::globalRandomNumber+getNormalDistribution(0, r2));
	SimpleIndividual::mutate(value, variance, positive);
	if (positive && value==0)
		value=numeric_limits<double>::denorm_min();
}

void SelfAdaptIndividual::mutate(MutationInformation *mutInf)
{
	MUTATE_ALL;
}

void SelfAdaptIndividual::combine(IndividualList &parents)
{
	COPY_ALL;
}