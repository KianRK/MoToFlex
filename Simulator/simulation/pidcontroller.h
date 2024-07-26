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
 * @file pidcontroller.h
 * A pid controller.
 * @author <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a>
 */

#pragma once

/**
 * @class PIDController
 * A pid controller.
 */
class PIDController
{
public:
	struct PIDParams
	{
		/** Length of one simulation step */
		float frameLen;

		float p;

		float i;

		float d;
	};

	/** 
	 * Constructor.
	 */
	PIDController();

	/**
	 * Set p i and d value of the controller.
	 * \param params The pid values.
	 */
	void setParams(PIDParams &params);

	/**
	 * Execute the controller.
	 * \param target Target angle.
	 * \param actual Measured angle.
	 */
	double doIt(double target, double actual);

private:
	double previousError, errorSum;
	float p, i, d, frameLen;
	bool first;
};
