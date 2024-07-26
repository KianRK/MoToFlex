/**
* @file Point.h
* This file contains the Point class.
* @author <a href="mailto:oliver.urbann@tu-dortmund.de> Oliver Urbann</a>
*/

#pragma once
		
#include <math.h>
#ifndef WALKING_SIMULATOR
#include "Tools/Math/Vector3.h"
#else
#include "math/Vector3.h"
#endif

/**
 * @class Point
 * Class representing a 3D Point.
 */
class Point
{
public:

	/**
	 * Constructor. Creates a point at coordinates (x, y, 0).
	 * @param x x coordinate.
	 * @param y y coordinate.
	 */
	Point(double x, double y)
	{
		this->x=x;
		this->y=y;
		r=z=0;
		rx=ry=0;
	}

	/**
	 * Constructor. Creates a point at coordinates (v[0], v[1], v[3]).
	 * @param v The vector to create from.
	 */
	Point(Vector3<double> v)
	{
		x=v[0];
		y=v[1];
		z=v[2];
		rx=ry=0;
	}


	/**
	 * Constructor. Creates a point at coordinates (x, y, z).
	 * @param x x coordinate.
	 * @param y y coordinate.
	 * @param z z coordinate.
	 */
	Point(double x, double y, double z)
	{
		this->x=x;
		this->y=y;
		this->z=z;
		this->r=0;
		rx=ry=0;
	}

	/**
	 * Constructor. Creates a point at coordinates (x, y, z) with rotation r around the local z axis.
	 * @param x x coordinate.
	 * @param y y coordinate.
	 * @param z z coordinate.
	 * @param r Rotation around local z axis.
	 */
	Point(double x, double y, double z, double r)
	{
		this->x=x;
		this->y=y;
		this->z=z;
		this->r=r;
		rx=ry=0;
	}

	/** Constructor. Sets all to 0. */
	Point()
	{
		x=y=z=r=rx=ry=0;
	}

	/**
	 * Copy data from another point.
	 * @param p The source point
	 * @return Copy of the instance
	 */
	Point operator = (const Point &p)
	{
		x=p.x;
		y=p.y;
		z=p.z;
		r=p.r;
		rx=p.rx;
		ry=p.ry;
		return *this;
	}

	/**
	 * Multiply operator. Multiplies every component with the corresponding component of p.
	 * @param p The point to multiply with.
	 * @return The result.
	 */
	Point operator * (const Point &p)
	{
		Point ret;
		ret.x=x*p.x;
		ret.y=y*p.y;
		ret.z=z*p.z;
		ret.rx=rx*p.rx;
		ret.ry=ry*p.ry;
		ret.r=r*p.r;
		return ret;
	}

	/**
	 * Multiply operator. Multiplies every component with the corresponding 
	 * component of p and stores the result in this instance.
	 * @param p The point to multiply with.
	 */
	void operator *= (const Point &p)
	{
		*this=*this*p;
	}

	/**
	 * Multiply with scalar. Multiplies every component with the scalar.
	 * @param f The scalar to multiply with.
	 * @return Copy of the instance.
	 */
	Point operator * (const double f)
	{
		Point ret;
		ret.x=x*f;
		ret.y=y*f;
		ret.z=z*f;
		ret.rx=rx*f;
		ret.ry=ry*f;
		ret.r=r*f;
		return ret;
	}

	/**
	 * Division by f. Divides every component by the scalar.
	 * @param f The scalar.
	 * @return Copy of the instance.
	 */
	Point operator / (const double f)
	{
		return *this*(1/f);
	}

	/**
	 * Multiply with scalar. Multiplies every component with the scalar and stores the result in this instance.
	 * @param f The scalar to multiply with.
	 * @return Copy of the instance.
	 */
	Point operator *= (const double f)
	{		
		*this=*this*f;
		return *this*f;
	}

	/**
	 * Adds another point.
	 * @param p The other point.
	 * @return Copy of the instance.
	 */
	Point operator + (const Point &p)
	{
		Point ret;
		ret.x=x+p.x;
		ret.y=y+p.y;
		ret.z=z+p.z;
		ret.rx=rx+p.rx;
		ret.ry=ry+p.ry;
		ret.r=r+p.r;
		return ret;
	}

	/**
	 * Substracts another point.
	 * @param p The other point.
	 * @return Copy of the instance.
	 */
	Point operator - (const Point &p)
	{
		Point ret;
		ret.x=x-p.x;
		ret.y=y-p.y;
		ret.z=z-p.z;
		ret.rx=rx-p.rx;
		ret.ry=ry-p.ry;
		ret.r=r-p.r;
		return ret;
	}

	/**
	 * Sets every component to the scalar.
	 * @param p The scaler.
	 */
	void operator = (double p)
	{
		r=rx=ry=x=y=z=p;
	}

	/**
	 * Checks for equality to another point.
	 * @param p The other point.
	 * @return True if equal, false otherwise.
	 */
	bool operator != (Point other)
	{
		return (x!=other.x ||
			y!=other.y ||
			z!=other.z ||
			r!=other.r ||
			rx!=other.rx ||
			ry!=other.ry);
	}

	/**
	 * Adds another point and stores the result in this instance.
	 * @param p The other point.
	 * @return Copy of the instance.
	 */
	Point operator += (const Point &p)
	{
		x+=p.x;
		y+=p.y;
		z+=p.z;
		r+=p.r;
		rx+=p.rx;
		ry+=p.ry;
		return *this;
	}

	/**
	 * Substracts another point and stores the result in this instance.
	 * @param p The other point.
	 * @return Copy of the instance.
	 */
	Point operator -= (const Point &p)
	{
		x-=p.x;
		y-=p.y;
		z-=p.z;
		r-=p.r;
		rx-=p.rx;
		ry-=p.ry;
		return *this;
	}

	/**
	 * Calculates the euklidian distance to another point within the x-y plane.
	 * @param p The other point.
	 * @return The distance.
	 */
	inline double euklidDistance2D(Point p)
	{
		return sqrt(pow(fabs(x-p.x), 2)+pow(fabs(y-p.y), 2));
	}

	/**
	 * Calculates the euklidian distance to another point in 3D space.
	 * @param p The other point.
	 * @return The distance.
	 */
	inline double euklidDistance3D(Point p)
	{
		return sqrt(pow(fabs(x-p.x), 2)+pow(fabs(y-p.y), 2)+pow(fabs(z-p.z), 2));
	}

	/**
	 * Rotates the point around the z axis. The z component is therefore constant.
	 * @param r Radians.
	 * @return Copy of the instance.
	 */
	Point rotate2D(double r)
	{
		double x, y;
		
		x=cos(r)*this->x-sin(r)*this->y;
		y=sin(r)*this->x+cos(r)*this->y;

		this->x=x;
		this->y=y;
		return *this;
	}

	/**
	 * Rotates the point around the x axis. The x component is therefore constant.
	 * @param r Radians.
	 * @return Copy of the instance.
	 */
	void rotateAroundX(double r)
	{
		double y, z;
		y=cos(r)*this->y-sin(r)*this->z;
		z=sin(r)*this->y+cos(r)*this->z;

		this->y=y;
		this->z=z;
	}

	/**
	 * Rotates the point around the y axis. The y component is therefore constant.
	 * @param r Radians.
	 * @return Copy of the instance.
	 */
	void rotateAroundY(double r)
	{
		double x, z;
		
		x=cos(r)*this->x+sin(r)*this->z;
		z=-sin(r)*this->x+cos(r)*this->z;

		this->x=x;
		this->z=z;
	}

	/**
	 * Scalar product with another point.
	 * @param p The other point.
	 * @return The result.
	 */
	double scalarProduct(Point p)
	{
		return x*p.x+y*p.y+z*p.z;
	}

	/** 
	 * Calculates the angle between the vectors 0-->this and 0-->vec both
	 * projected onto the x-y plane.
	 * @param vec The other point.
	 * @return Angle in radians.
	 */
	double angleTo2D(Point vec)
	{
		return atan2(vec.y, vec.x)-atan2(y, x);
	}

	/** 
	 * Calculates the angle between the vectors 0-->this and 0-->vec both
	 * in 3D space.
	 * @param vec The other point.
	 * @return Angle in radians.
	 */
	double angleTo3D(Point vec)
	{
		Point nullPoint;
		return ((*this).scalarProduct(vec))/((*this).euklidDistance3D(nullPoint)*vec.euklidDistance3D(nullPoint));
	}

	/** 
	 * Calculates the length of the vector 0-->this.
	 * @return The length.
	 */
	double getPositionVecLen()
	{
		return sqrt(x*x+y*y+z*z);
	}

	double 
		x, /**< x coordinate */
		y, /**< y coordinate */
		z, /**< z coordinate */
		rx,/**< Rotation around x. */
		ry,/**< Rotation around y. */
		r; /**< Rotation around z. */
};
