/**
 * @file AffinTransformation.cpp
 * Implements a class for calculations with affin Transfomations. 
 * It is basicly a re-implementation of the class Pose3D.
 * Re-Implementation is necessary, because this class is needed for numerical purposes.
 *
 */

#include "AffinTransformation.h"
#include <string.h>

AffinTransformation::AffinTransformation() {

  A[0][0] = 1;  A[0][1] = 0;  A[0][2] = 0;
  A[1][0] = 0;  A[1][1] = 1;  A[1][2] = 0;
  A[2][0] = 0;  A[2][1] = 0;  A[2][2] = 1;

  b[0] = 0;
  b[1] = 0;
  b[2] = 0;

}


AffinTransformation::AffinTransformation(float x, float y, float z, float alpha, float beta, float gamma) {

  float sinAlpha = sin(alpha);
  float cosAlpha = cos(alpha);
  float sinBeta = sin(beta);
  float cosBeta = cos(beta);
  float sinGamma = sin(gamma);
  float cosGamma = cos(gamma);

  A[0][0] = cosBeta * cosGamma;
  A[0][1] = sinAlpha * sinBeta * cosGamma - cosAlpha * sinGamma;
  A[0][2] = cosAlpha * sinBeta * cosGamma + sinAlpha * sinGamma;
  A[1][0] = cosBeta * sinGamma;
  A[1][1] = sinAlpha * sinBeta * sinGamma + cosAlpha * cosGamma;
  A[1][2] = cosAlpha * sinBeta * sinGamma - sinAlpha * cosGamma;
  A[2][0] = -sinBeta;
  A[2][1] = sinAlpha * cosBeta;
  A[2][2] = cosAlpha * cosBeta;

  b[0] = x;
  b[1] = y;
  b[2] = z;

}

void AffinTransformation::debugOut() {

  std::cout << A[0][0] << "\t" << A[0][1] << "\t" << A[0][2] << "\t\t" << b[0] << "\n";
  std::cout << A[1][0] << "\t" << A[1][1] << "\t" << A[1][2] << "\t\t" << b[1] << "\n";
  std::cout << A[2][0] << "\t" << A[2][1] << "\t" << A[2][2] << "\t\t" << b[2] << "\n";

}
