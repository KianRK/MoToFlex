/**
 * @file Math/hMatrix.h
 *
 * This class is an implementation for a homogenous matrix.
 * USE WITH CARE!
 * If in doubt, use Pose3D.
 *
 TODO: avoid copying in operator*=
 * @author Thorsten Humberg
 */


#ifndef __hmatrix_h__
#define __hmatrix_h__

class hMatrix{
public:
	float values[3][4]; //3 rows: last line is always 0 0 0 1
	float tempvalues[3][4]; //3 rows: last line is always 0 0 0 1

	hMatrix(){
		values[0][0]=1;
		values[0][1]=0;
		values[0][2]=0;
		values[0][3]=0;
		values[1][0]=0;
		values[1][1]=1;
		values[1][2]=0;
		values[1][3]=0;
		values[2][0]=0;
		values[2][1]=0;
		values[2][2]=1;
		values[2][3]=0;		
	}

	hMatrix(float x, float y, float z){
		values[0][0]=1;
		values[0][1]=0;
		values[0][2]=0;
		values[0][3]=x;
		values[1][0]=0;
		values[1][1]=1;
		values[1][2]=0;
		values[1][3]=y;
		values[2][0]=0;
		values[2][1]=0;
		values[2][2]=1;
		values[2][3]=z;		
	}

	void setTranslation(float x, float y, float z){
		this->values[0][3]=x;
		this->values[1][3]=y;
		this->values[2][3]=z;
	}
	
	void setRotationX(float angle){
		this->values[1][1]=this->values[2][2]=cos(angle);
		this->values[1][2]=-sin(angle);
		this->values[2][1]=sin(angle);		
	}

	void setRotationY(float angle){
		this->values[0][0]=this->values[2][2]=cos(angle);
		this->values[2][0]=-sin(angle);
		this->values[0][2]=sin(angle);		
	}

	void setRotationZ(float angle){
		this->values[1][1]=this->values[0][0]=cos(angle);
		this->values[0][1]=-sin(angle);
		this->values[1][0]=sin(angle);		
	}

	float getXAngle(){
		float h = sqrt(this->values[1][2] * this->values[1][2] + this->values[2][2] * this->values[2][2]);
		return h ? acos(this->values[2][2] / h) * (this->values[1][2] > 0 ? -1 : 1) : 0;
	}

	float getYAngle(){
		float h = sqrt(this->values[0][0] * this->values[0][0] + this->values[2][0] * this->values[2][0]);
		return h ? acos(this->values[0][0] / h) * (this->values[2][0] > 0 ? -1 : 1) : 0;
	}

	float getZAngle(){
		float h = sqrt(this->values[0][0] * this->values[0][0] + this->values[1][0] * this->values[1][0]);
		return h ? acos(this->values[0][0] / h) * (this->values[1][0] > 0 ? -1 : 1) : 0;
	}

	
	hMatrix operator/=(const hMatrix& other){
		hMatrix temp;
		temp=other;
		temp*=(*this);
		*this=temp;
		return *this;
	}


	hMatrix operator*=(const hMatrix& other){
		tempvalues[0][0]=	
			this->values[0][0]*other.values[0][0]
		+	this->values[0][1]*other.values[1][0]
		+	this->values[0][2]*other.values[2][0];

		tempvalues[0][1]=	
			this->values[0][0]*other.values[0][1]
		+	this->values[0][1]*other.values[1][1]
		+	this->values[0][2]*other.values[2][1];

		tempvalues[0][2]=	
			this->values[0][0]*other.values[0][2]
		+	this->values[0][1]*other.values[1][2]
		+	this->values[0][2]*other.values[2][2];

		tempvalues[0][3]=	
			this->values[0][0]*other.values[0][3]
		+	this->values[0][1]*other.values[1][3]
		+	this->values[0][2]*other.values[2][3]
		+	this->values[0][3];


		tempvalues[1][0]=	
			this->values[1][0]*other.values[0][0]
		+	this->values[1][1]*other.values[1][0]
		+	this->values[1][2]*other.values[2][0];

		tempvalues[1][1]=	
			this->values[1][0]*other.values[0][1]
		+	this->values[1][1]*other.values[1][1]
		+	this->values[1][2]*other.values[2][1];

		tempvalues[1][2]=	
			this->values[1][0]*other.values[0][2]
		+	this->values[1][1]*other.values[1][2]
		+	this->values[1][2]*other.values[2][2];

		tempvalues[1][3]=	
			this->values[1][0]*other.values[0][3]
		+	this->values[1][1]*other.values[1][3]
		+	this->values[1][2]*other.values[2][3]
		+	this->values[1][3];

		tempvalues[2][0]=	
			this->values[2][0]*other.values[0][0]
		+	this->values[2][1]*other.values[1][0]
		+	this->values[2][2]*other.values[2][0];

		tempvalues[2][1]=	
			this->values[2][0]*other.values[0][1]
		+	this->values[2][1]*other.values[1][1]
		+	this->values[2][2]*other.values[2][1];

		tempvalues[2][2]=	
			this->values[2][0]*other.values[0][2]
		+	this->values[2][1]*other.values[1][2]
		+	this->values[2][2]*other.values[2][2];

		tempvalues[2][3]=	
			this->values[2][0]*other.values[0][3]
		+	this->values[2][1]*other.values[1][3]
		+	this->values[2][2]*other.values[2][3]
		+	this->values[2][3];

		for(int i=0;i<3;i++){
			for(int j=0;j<4;j++){
				this->values[i][j]=tempvalues[i][j];
			}
		}

		return *this;
	}

	hMatrix operator=(const hMatrix& other){
		for(int i=0;i<3;i++){
			for(int j=0;j<4;j++){
				this->values[i][j]=other.values[i][j];
			}
		}
		return *this;
	}

};
#endif
