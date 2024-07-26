/**
 * \file ComposedNDimFS.h
 *
 * \brief A composed n-dimensional FuzzySet
 * 
 * \authors Marc Nunkesser
 */

/* $log$ */

#ifndef COMPOSEDNDIMFS_H
#define COMPOSEDNDIMFS_H

#include <Fuzzy/NDimFS.h>
#include <Fuzzy/ComposedFS.h>

/**
 * \brief A composed n-dimensional FuzzySet.
 *
 * A composed n-dimensional fuzzy set makes it possible to do some
 * calculations on n-dimensopnal fuzzy sets,
 * e.g. to connect two n-dimensional fuzzy sets using the max-function. 
 */
class ComposedNDimFS: public NDimFS {
public:

/**
 * \brief Constructor.
 *
 *@param nDimFS1 the first n-dimensional fuzzy set
 *@param nDimFS2 the secound n-dimensional fuzzy set
 *@param userFunction the function connecting both fuzzy sets
 */
	ComposedNDimFS(const RCPtr<NDimFS> & nDimFS1,
	               const RCPtr<NDimFS> & nDimFS2,
	               double (*userFunction)(double,double ) );

/**
 * \brief Membership (\f$\mu\f$) function.
 * 
 * @param v the vector of values \f$(x_1,\ldots,x_n)\f$
 * @return the value of the membership fuction at \f$(x_1,\ldots,x_n)\f$
 */
	virtual double operator()(const std::vector<double>&v) const;
	
/**
 * \brief Membership (\f$\mu\f$) function for one-dimensional fuzzy set.
 * 
 * @param a the value \f$x\f$
 * @return the value of the membership fuction at \f$x\f$
 */	
	double operator()(double a) const;
	
/**
 * \brief Membership (\f$\mu\f$) function for two-dimensional fuzzy set.
 * 
 * @param a the value \f$x_1\f$
 * @param b the value \f$x_2\f$
 * @return the value of the membership fuction at \f$(x_1,x_2)\f$
 */		
	double operator()(double a, double b) const;
	
/**
 * \brief Membership (\f$\mu\f$) function for three-dimensional fuzzy set.
 * 
 * @param a the value \f$x_1\f$
 * @param b the value \f$x_2\f$
 * @param c the value \f$x_3\f$
 * @return the value of the membership fuction at \f$(x_1,x_2,x_3)\f$
 */			
	double operator()(double a, double b, double c ) const;
	
/**
 * \brief Membership (\f$\mu\f$) function for four-dimensional fuzzy set.
 * 
 * @param a the value \f$x_1\f$
 * @param b the value \f$x_2\f$
 * @param c the value \f$x_3\f$
 * @param d the value \f$x_4\f$
 * @return the value of the membership fuction at \f$(x_1,x_2,x_3,x_4)\f$
 */			
	double operator()(double a,double b, double c, double d ) const;

	/**
	 * \brief Cast operator
	 * 
	 * Casts the ComposedNDimFS to a ComposedFS if the dimension is equal to one.
	 * 
	 * @return the ComposedFS
	 */
	operator RCPtr<ComposedFS>();

/**
 * \brief Returns the dimension of a n-dimensional fuzzy set.
 *
 * @return the dimension of the fuzzy set.
 */
	virtual unsigned  getDimension() const {
		return(leftOperand->getDimension());	
	};
	
	
private:

	RCPtr<NDimFS> leftOperand,rightOperand;
	double (*userDefinedOperator)( double, double ); //Operator supplied by the user, which can be used instead of and/or
};


#endif
