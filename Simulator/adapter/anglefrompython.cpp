/*
	Copyright 2024, Oliver Urbann
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
 * @file anglefrompython.cpp
 * Used by the simulation to retrieve the target angles for the robot.
 * @author <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a>
 */

#include "anglefrompython.h"

void AngleFromPython::getAngles(float angles[], int numOfAngles, Robot &robot)
{   
    if (numOfAngles == NUM_OF_JOINTS)
        memcpy(angles, _angles, NUM_OF_JOINTS * sizeof(float));
    else
        printf("Error: Incorrect number of angles! Expected %d, got %d", NUM_OF_JOINTS, numOfAngles);
}

void AngleFromPython::setAngles(float *angles, int numOfAngles)
{
    memcpy(_angles, angles, NUM_OF_JOINTS * sizeof(float));
}