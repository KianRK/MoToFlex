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
 * @file fastfilter.h
 * Simple filter implementation.
 * @author <a href="mailto:oliver.urbann@tu-dortmund.de">Oliver Urbann</a>
 */

#pragma once

#include "DynamicRingBuffer.h"
#include "Filter.h"
#include <string>
#include <vector>

using namespace std;

/**
* @class FastFilter
* Simple filter implementation.
*/
class FastFilter :
	public Filter
{
public:
	/** 
	 * Constructor.
	 */
	FastFilter(void);

	/**
	 * Constructor for a filter of constant order.
	 * \param size Order of the filter.
	 */
	FastFilter(const unsigned int size);

	/** 
	 * Destructor.
	 */
	~FastFilter(void) { delete buffer; };

	/**
	 * Add a value.
	 * \param v The value.
	 * \return Actual filtered value
	 */
	double nextValue(double v);

	/**
	 * Create a buffer for a filter of dynamic size.
	 * \param size Order of the filter.
	 * \return true, if creation was successfull.
	 */
	bool createBuffer(const unsigned int size);

private:
	DynamicRingBuffer<double> *buffer;
};
