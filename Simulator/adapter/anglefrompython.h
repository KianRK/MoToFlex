/*
	Copyright 2023, Oliver Urbann
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
 * @file anglesfrompython.h
 * Used by the simulation to retrieve target angles from a python program, that sent
 * the angles via step in this frame.
 * @author <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a>
 */

#pragma once	
#include "robot.h"
#include <string>
#include "tools/anglereader.h"
#include "adapter/angleadapter.h"
#include "nao/nao.h"

/**
 * Used by the simulation to retrieve the target angles for the robot.
 * This class is an adapter to provide an uniform interface for all
 * possible ways to retrieve target angles. This class uses the AngleReader
 * to read the angles from a csv file.
 */
class AngleFromPython : public AngleAdapter
{
public:

	/** 
	 * Constructor.
	 */
	AngleFromPython() {}; 

	/** 
	 * Called by simulation.cpp to retrieve the target 
	 * angles for the next frame.
	 * \param angles array filled by getAngles with the target angles
	 * \param numOfAngles
	 */
	void getAngles(float angles[], int numOfAngles, Robot &robot);
	void setAngles(float angles[], int numOfAngles);

private:
	float _angles[NUM_OF_JOINTS];
};