/**
* @file GaussianDistribution.h
* 
* Definition of class GaussianDistribution
*
* @author <a href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</a>
*/

#ifndef __GaussianDistribution_h_
#define __GaussianDistribution_h_

#include "Vector2.h"


/**
* @class GaussianDistribution
* Represents a 1-dimensional normal distribution
*/
class GaussianDistribution
{
public:
  /** The mean value of the distribution*/
  double mean;
  /** The variance of the distribution*/
  double variance;

  /** Empty standard constructor*/
  GaussianDistribution() {}

  /** Copy constructor
  * @param other The other distribution that is copied to this one
  */
  GaussianDistribution(const GaussianDistribution& other) {*this = other;}

  /** Computes the Mahalanobis distance to another distribution.
  *   This function is used to compare two different distributions for checking, if
  *   they describe the same phenomenon.
  * @param other Another distribution
  * @return A value without any unit
  */
  double distanceTo(const GaussianDistribution& other) const;

  /** Computes the probability density at a given position
  * @param p The position
  * @return The probability at pos.
  */
  virtual double probabilityAt(double p) const;

  /** Computes the probability density at the center of the distribution
  * @return The probability
  */
  double probabilityAtMean() const
    { return probabilityAt(mean);}

  /** Computes mean and variance of the distribution
  * @param x A list of measurements of a variable
  * @param numOfX The number of measurements
  */
  void generateDistributionFromMeasurements(double* x, int numOfX);

  /** Assignment operator
  * @param other The other distribution that is assigned to this one
  * @return A reference to this object after the assignment.
  */
  GaussianDistribution& operator=(const GaussianDistribution & other)
  {
    mean = other.mean;
    variance = other.variance;
    return *this;
  }
};

#endif //__GaussianDistribution_h_
