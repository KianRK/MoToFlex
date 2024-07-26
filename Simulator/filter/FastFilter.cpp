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
#include "FastFilter.h"
#include <fstream>

FastFilter::FastFilter(void)
{
	buffer=NULL;
}


FastFilter::FastFilter(unsigned int size)
{
	buffer=new DynamicRingBuffer<double>(size);
}

bool FastFilter::createBuffer(const unsigned int size)
{
	if (buffer==NULL)
		buffer=new DynamicRingBuffer<double>(size);
	else
		buffer->init(size);

	return true;
}

double FastFilter::nextValue(double v)
{
	if (buffer==NULL)
		return 0;

	buffer->add(v);
	return buffer->getAverage();
}
