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

	HomMatrix(const Matrix_mxn<4,4> &M)
	{
		*this=M;
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
		RotationMatrix M=rot;
		for (int i=0; i<3; i++)
			for (int j=0; j<3; j++)
				setMember(i, j, M[j][i]);
	}

	void setTranslation(const Vector3<double> &trans)
	{
		Vector3<double> v=trans;
		for (int i=0; i<3; i++)
			setMember(i, 3, v[i]);
	}

	
	void setTranslation(const Vector3<float> &trans)
	{
		Vector3<float> v=trans;
		for (int i=0; i<3; i++)
			setMember(i, 3, v[i]);
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