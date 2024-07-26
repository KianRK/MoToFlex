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
 * @file anglesfromcsv.cpp
 * Used by the simulation to retrieve the target angles for the robot.
 * @author <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a>
 */

#include "anglefromcsv.h"

void AngleFromCSV::getAngles(float angles[], int numOfAngles, Robot &robot)
{
	reader->getAngles(angles, numOfAngles);
}

void AngleFromCSV::setSettingsPath(string path)
{
	if (reader!=NULL)
		delete reader;

	reader=new AngleReader(path);
}