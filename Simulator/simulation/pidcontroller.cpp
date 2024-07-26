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
#include "pidcontroller.h"

PIDController::PIDController()
{
}

void PIDController::setParams(PIDParams &params)
{
	previousError=0;
	errorSum=0;
	first=true;
	p=params.p;
	i=params.i;
	d=params.d;
	frameLen=params.frameLen;
}

double PIDController::doIt(double target, double actual)
{
	double error = target - actual;
	double pControlValue = p * error;
	errorSum += error;
	double iControlValue = i * frameLen * errorSum;

	double dControlValue=0;
	if (!first)
		dControlValue = (d * (error - previousError));
	// d should be d/frameLen, but that doesnt matter
	first=false;

	previousError = error;
	return pControlValue + iControlValue + dControlValue;
}
