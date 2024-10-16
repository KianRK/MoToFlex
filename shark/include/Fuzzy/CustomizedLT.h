
/**
 * \file CustomizedLT.h
 *
 * \brief A LinguisticTerm with an user defined mambership function
 * 
 * \authors Marc Nunkesser
 */
/* $log$ */
#ifndef CUSTOMIZEDLT_H
#define CUSTOMIZEDLT_H

#include <Fuzzy/FuzzyException.h>
#include <Fuzzy/LinguisticTerm.h>
#include <Fuzzy/CustomizedFS.h>
#include <Fuzzy/LinguisticVariable.h>
#include <Fuzzy/RCPtr.h>

/**
 * \brief A LinguisticTerm with an user defined mambership function.
 *
 * This class implements a LinguisticTerm with an user definded membership 
 * function.
 */
class CustomizedLT: public LinguisticTerm, public CustomizedFS {
public:
	
    /**
     * \brief Constructor.
     *
	* @param name the name
	* @param parent the associated linguistic variable 
     * @param userFunction membership function defined by the user
     * @param min the min. value for which the membership function is nonzero (or exceeds a
     * given threshold)	
     * @param max the max. value for which the membership function is nonzero (or exceeds a
     * given threshold)
     */
	CustomizedLT(
	    const std::string                name,
	    const RCPtr<LinguisticVariable>& parent,
	    double (*userFunction)( double),
	    double                           min,
	    double                           max);

    /**
     * \brief Returns the lower boundary of the support.
     * 
     * @return the min. value for which the membership function is nonzero (or exceeds a
     * given threshold)
     */
	inline double         getMin() const{
		return(std::max(CustomizedFS::getMin(), parent->getLowerBound()));
	};

    /**
     * \brief Returns the upper boundary of the support.
     * 
     * @return the max. value for which the membership function is nonzero (or exceeds a
     * given threshold)
     */
	inline double         getMax() const{
	return(std::min(CustomizedFS::getMax(), parent->getUpperBound()));
	};

};



#endif
