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
 * @file FIRFilter.h
 * Implementation of a FIR filter.
 * @author <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a>
 */

#pragma once

#ifndef WALKING_SIMULATOR
#include "../../DynamicRingBuffer.h"
#else
#include "filter/DynamicRingBuffer.h"
#endif

#include "Filter.h"
#include <string>
#include <vector>

using namespace std;

/**
* @class FastFilter
* Implementation of a FIR filter.
*/
class FIRFilter :
	public Filter
{
public:

	/** 
	 * Constructor.
	 */
	FIRFilter(void);

	/** 
	 * Destructor.
	 */
	~FIRFilter(void) 
	{
		delete buffer;
	};

	/**
	 * Add a value.
	 * \param v The value.
	 * \return Actual filtered value
	 */
	double nextValue(double v);

	/**
	 * Reads the coefficients of the FIR filter from a file.
	 * \param path Path to the file.
	 * \return True, if operation was successfull
	 */
	bool readCoefficients(string path);

	/**
	 * Set the coefficients of the FIR filter.
	 * \param coefficients Array of the coefficients
	 * \param n Number of coefficients.
	 */
	void setCoefficients(double *coefficients, int n);

private:
	unsigned int n;
	DynamicRingBuffer<double> *buffer;
	vector<double> coefficients;
};
