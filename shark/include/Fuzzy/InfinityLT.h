
/**
 * \file InfinityLT.h
 *
 * \brief LinguisticTerm with a step function as membership function
 * 
 * \authors Marc Nunkesser
 */

/* $log$ */
#ifndef INFINITYLT_H
#define INFINITYLT_H

#include <Fuzzy/FuzzyException.h>
#include <Fuzzy/LinguisticTerm.h>
#include <Fuzzy/InfinityFS.h>
#include <Fuzzy/LinguisticVariable.h>

/**
 * \brief LinguisticTerm with a step function as membership function.
 *
 * The support of this LinguisticTerm reaches to either positive infinity or negative
 * infinity. The corresponding menbership function is discribed by two dedicated points a and b.
 * If the support reaches positive infinity, the membership function is 0 for values smaller than a,
 * raises constantly to 1 between a and b, and stays 1 for values greater than b. If the support 
 * reaches negantive infinity, the membership function is 1 for values smaller than a, falls 
 * constantly to 0 between a and b, and is 0 for values greater than b. 
 *
 * For positive Infinity:
 * \f[
 *      \mu(x) = \left\{\begin{array}{ll} 0, & x < a \\ 
 *      \frac{1}{b-a}(x-a), & a \le x \le b \\ 
 *      1, & x > b\end{array}\right.
 * \f]
 * <img src="../images/InfinityFS.png"> 
 * 
 */
class InfinityLT: public LinguisticTerm, public InfinityFS {
public:
	
    /**
	* \brief Construnctor.
	* 
	* @param name the name
	* @param parent the associated linguistic variable 
     * @param positiveInfinity decides whether the support reaches to positive (true) or
     * negative (false) infinity
     * @param a point where membership function starts to raise/fall (positiveInfinity: true/false)
     * @param b point where membership stops to raise/fall (positiveInfinity: true/false)
	*/
	InfinityLT(const std::string name,
	           const RCPtr<LinguisticVariable>& parent,
	           bool                             positiveInfinity,
	           double                           a,
	           double                           b);


	// overloaded operator () - the mu function
	//inline double operator()(double x) const;

    /**
     * \brief Returns the lower boundary of the support.
     * 
     * @return the min. value for which the membership function is nonzero (or exceeds a
     * given threshold)
     */
	inline double         getMin() const{
		return(std::max(InfinityFS::getMin(), parent->getLowerBound()));
	};

    /**
     * \brief Returns the upper boundary of the support.
     * 
     * @return the max. value for which the membership function is nonzero (or exceeds a
     * given threshold)
     */
	inline double         getMax() const{
		return(std::min(InfinityFS::getMax(), parent->getUpperBound()));
	};

};



#endif
