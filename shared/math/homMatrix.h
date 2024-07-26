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

#pragma once

#include "Matrix_mxn.h"
#include "Matrix.h"

class HomMatrix : public Matrix_mxn<4, 4>
{
public:
	HomMatrix()
	{
		setToIdentity();
		setMember(3, 3, 1);
	}

	HomMatrix(const RotationMatrix &rot)
	{
		setToIdentity();
		setMember(3, 3, 1);
		setRotation(rot);
	}

	HomMatrix(const Vector3<double> &trans)
	{
		setToIdentity();
		setMember(3, 3, 1);
		setTranslation(trans);
	}

	HomMatrix(const Vector3<float> &trans)
	{
		setToIdentity();
		setMember(3, 3, 1);
		setTranslation(trans);
	}

	void setRotation(const RotationMatrix &rot)
	{
		for (int i=0; i<3; i++)
			for (int j=0; j<3; j++)
				setMember(i, j, rot[j][i]);
	}

	void setTranslation(const Vector3<double> &trans)
	{
		for (int i=0; i<3; i++)
			setMember(i, 3, trans[i]);
	}

	
	void setTranslation(const Vector3<float> &trans)
	{
		for (int i=0; i<3; i++)
			setMember(i, 3, trans[i]);
	}

	 
	HomMatrix& operator=(const Matrix_mxn<4, 4>& source)
	{    
		memcpy(content, source.content, memSize);
		return *this;
	}

	Vector3<double> getTranslation()
	{
		return Vector3<double>(getMember(0, 3), getMember(1, 3), getMember(2, 3));
	}
};