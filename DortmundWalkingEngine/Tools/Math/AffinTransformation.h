/** 
 * @file affinTransformation.h
 * Implements a class for calculations with affin Transfomations. 
 * It is basicly a re-implementation of the class Pose3D.
 * Re-Implementation is necessary, because this class is needed for numerical purposes.
 *
 */

#ifndef __AffinTransformation_h__
#define __AffinTransformation_h__

#include <string.h>
#include <math.h>
#include <iostream>
#include "Common.h"

class AffinTransformation {

  public :
  
    float A[3][3];     // The Matrix A and the vektor b that represents the affin transformation
    float b[3];
  
  AffinTransformation();  // Creates a transformation the is the identity.

  // creates a transformation that has a rotation matrix with alpha, beta and gamma as the Roll-Pitch-Yaw angles.
  AffinTransformation(float x, float y, float z, float alpha, float beta, float gamma);

  void debugOut();  // Prints the transformation to stdout
 

// Mults a rotation around the x axis from the right to this transformation.

  inline void multRotXRight(float alpha) {

    float x;
    
    float cosa = cos(alpha);
    float sina = sin(alpha);

    x       =  cosa * A[0][1] + sina * A[0][2];
    A[0][2] = -sina * A[0][1] + cosa * A[0][2];
    A[0][1] = x;

    x       =  cosa * A[1][1] + sina * A[1][2];
    A[1][2] = -sina * A[1][1] + cosa * A[1][2];
    A[1][1] = x;

    x       =  cosa * A[2][1] + sina * A[2][2];
    A[2][2] = -sina * A[2][1] + cosa * A[2][2];
    A[2][1] = x;

  }

  // Mults a rotation around the y axis from the right to this transformation.


  inline void multRotYRight(float alpha) {

    float x;
    
    float cosa = cos(alpha);
    float sina = sin(alpha);

    x       =  cosa * A[0][0] - sina * A[0][2];
    A[0][2] =  sina * A[0][0] + cosa * A[0][2];
    A[0][0] = x;

    x       =  cosa * A[1][0] - sina * A[1][2];
    A[1][2] =  sina * A[1][0] + cosa * A[1][2];
    A[1][0] = x;

    x       =  cosa * A[2][0] - sina * A[2][2];
    A[2][2] =  sina * A[2][0] + cosa * A[2][2];
    A[2][0] = x;

  }

  // Mults a rotation around the z axis from the right to this transformation.



  inline void multRotZRight(float alpha) {

    float x;
    
    float cosa = cos(alpha);
    float sina = sin(alpha);

    x       =  cosa * A[0][0] + sina * A[0][1];
    A[0][1] = -sina * A[0][0] + cosa * A[0][1];
    A[0][0] = x;

    x       =  cosa * A[1][0] + sina * A[1][1];
    A[1][1] = -sina * A[1][0] + cosa * A[1][1];
    A[1][0] = x;

    x       =  cosa * A[2][0] + sina * A[2][1];
    A[2][1] = -sina * A[2][0] + cosa * A[2][1];
    A[2][0] = x;

  }

  // Mults a rotation around the x axis from the left to this transformation.


  inline void multRotXLeft(float alpha) {

    float x;
    
    float cosa = cos(alpha);
    float sina = sin(alpha);

    x       =  cosa * A[1][0] - sina * A[2][0];
    A[2][0] =  sina * A[1][0] + cosa * A[2][0];
    A[1][0] = x;

    x       =  cosa * A[1][1] - sina * A[2][1];
    A[2][1] =  sina * A[1][1] + cosa * A[2][1];
    A[1][1] = x;

    x       =  cosa * A[1][2] - sina * A[2][2];
    A[2][2] =  sina * A[1][2] + cosa * A[2][2];
    A[1][2] = x;

    x       =  cosa * b[1] - sina * b[2];
    b[2]    =  sina * b[1] + cosa * b[2];
    b[1]    = x;

  }

 // Mults a rotation around the y axis from the left to this transformation.

  inline void multRotYLeft(float alpha) {

    float x;
    
    float cosa = cos(alpha);
    float sina = sin(alpha);

    x       =  cosa * A[0][0] + sina * A[2][0];
    A[2][0] = -sina * A[0][0] + cosa * A[2][0];
    A[0][0] = x;

    x       =  cosa * A[0][1] + sina * A[2][1];
    A[2][1] = -sina * A[0][1] + cosa * A[2][1];
    A[0][1] = x;

    x       =  cosa * A[0][2] + sina * A[2][2];
    A[2][2] = -sina * A[0][2] + cosa * A[2][2];
    A[0][2] = x;

    x       =  cosa * b[0] + sina * b[2];
    b[2]    = -sina * b[0] + cosa * b[2];
    b[0]    = x;

  }

   // Mults a rotation around the z axis from the left to this transformation.

  inline void multRotZLeft(float alpha) {

    float x;
    
    float cosa = cos(alpha);
    float sina = sin(alpha);

    x       =  cosa * A[0][0] - sina * A[1][0];
    A[1][0] =  sina * A[0][0] + cosa * A[1][0];
    A[0][0] = x;

    x       =  cosa * A[0][1] - sina * A[1][1];
    A[1][1] =  sina * A[0][1] + cosa * A[1][1];
    A[0][1] = x;

    x       =  cosa * A[0][2] - sina * A[1][2];
    A[1][2] =  sina * A[0][2] + cosa * A[1][2];
    A[0][2] = x;

    x       =  cosa * b[0] - sina * b[1];
    b[1]    =  sina * b[0] + cosa * b[1];
    b[0]    = x;

  }

   // Mults a translation along the x axis from the right to this transformation.

  inline void multTransXRight(float x) {
    
    b[0] = A[0][0] * x + b[0];
    b[1] = A[1][0] * x + b[1];
    b[2] = A[2][0] * x + b[2];

  }

   // Mults a translation along the y axis from the right to this transformation.


  inline void multTransYRight(float y) {
    
    b[0] = A[0][1] * y + b[0];
    b[1] = A[1][1] * y + b[1];
    b[2] = A[2][1] * y + b[2];

  }

    // Mults a translation along the z axis from the right to this transformation.


  inline void multTransZRight(float z) {
    
    b[0] = A[0][2] * z + b[0];
    b[1] = A[1][2] * z + b[1];
    b[2] = A[2][2] * z + b[2];

  }

    // Mults a translation from the right to this transformation.


  inline void multTransRight(float x, float y, float z) {

    b[0] = A[0][0] * x + A[0][1] * y + A[0][2] * z + b[0];
    b[1] = A[1][0] * x + A[1][1] * y + A[1][2] * z + b[1];
    b[2] = A[2][0] * x + A[2][1] * y + A[2][2] * z + b[2];

  }

    // Mults a translation along the x axis from the left to this transformation.


  inline void multTransXLeft(float x) {
    
    b[0] = b[0] + x;

   }

     // Mults a translation along the y axis from the left to this transformation.


  inline void multTransYLeft(float y) {
    
    b[1] = b[1] + y;

  }

     // Mults a translation along the z axis from the left to this transformation.

  inline void multTransZLeft(float z) {

    b[2] = b[2] + z;

  }

     // Mults a translation from the left to this transformation.


  inline void multTransLeft(float x, float y, float z) {

    b[0] = b[0] + x;
    b[1] = b[1] + y;
    b[2] = b[2] + z;

  }

  // Mults a matrix from the right to A, that represents a differential rotation around the x axis.
  // Be aware, that after this multiplikation A is no longer a rotation Matrix!! 
  // This calculation is used to calculate the gradient/jacobi matrix of kinematic functions.

  inline void pseudoMultDiffRotX(float alpha) {
    
    float x;

    float sina = sin(alpha);
    float cosa = cos(alpha);

    A[0][0] = 0;
    A[1][0] = 0;
    A[2][0] = 0;

    x       = -sina * A[0][1] + cosa * A[0][2];
    A[0][2] = -cosa * A[0][1] - sina * A[0][2];
    A[0][1] = x;

    x       = -sina * A[1][1] + cosa * A[1][2];
    A[1][2] = -cosa * A[1][1] - sina * A[1][2];
    A[1][1] = x;

    x       = -sina * A[2][1] + cosa * A[2][2];
    A[2][2] = -cosa * A[2][1] - sina * A[2][2];
    A[2][1] = x;

  }

  // Mults a matrix from the right to A, that represents a differential rotation around the y axis.
  // Be aware, that after this multiplikation A is no longer a rotation Matrix!! 
  // This calculation is used to calculate the gradient/jacobi matrix of kinematic functions.


  inline void pseudoMultDiffRotY(float alpha) {
    
    float x;

    float sina = sin(alpha);
    float cosa = cos(alpha);

    A[0][1] = 0;
    A[1][1] = 0;
    A[2][1] = 0;

    x       = -sina * A[0][0] - cosa * A[0][2];
    A[0][2] =  cosa * A[0][0] - sina * A[0][2];
    A[0][0] = x;

    x       = -sina * A[1][0] - cosa * A[1][2];
    A[1][2] =  cosa * A[1][0] - sina * A[1][2];
    A[1][0] = x;

    x       = -sina * A[2][0] - cosa * A[2][2];
    A[2][2] =  cosa * A[2][0] - sina * A[2][2];
    A[2][0] = x;

  }

  // Mults a matrix from the right to A, that represents a differential rotation around the z axis.
  // Be aware, that after this multiplikation A is no longer a rotation Matrix!! 
  // This calculation is used to calculate the gradient/jacobi matrix of kinematic functions.

  inline void pseudoMultDiffRotZ(float alpha) {
    
    float x;

    float sina = sin(alpha);
    float cosa = cos(alpha);

    A[0][2] = 0;
    A[1][2] = 0;
    A[2][2] = 0;

    x       = -sina * A[0][0] + cosa * A[0][1];
    A[0][1] = -cosa * A[0][0] - sina * A[0][1];
    A[0][0] = x;

    x       = -sina * A[1][0] + cosa * A[1][1];
    A[1][1] = -cosa * A[1][0] - sina * A[1][1];
    A[1][0] = x;

    x       = -sina * A[2][0] + cosa * A[2][1];
    A[2][1] = -cosa * A[2][0] - sina * A[2][1];
    A[2][0] = x;

  }

  // Mults a matrix from the right to A, that represents a differential rotation around the x axis,
  // but with negative sin and cos. This is necessary if you use -alpha instead of alpha. [(f(-x))' = -f'(-x)]
  // Be aware, that after this multiplikation A is no longer a rotation Matrix!! 
  // This calculation is used to calculate the gradient/jacobi matrix of kinematic functions.

  inline void pseudoMultNegativeDiffRotX(float alpha) {
    
    float x;

    float sina = -sin(alpha);
    float cosa = -cos(alpha);

    A[0][0] = 0;
    A[1][0] = 0;
    A[2][0] = 0;

    x       = -sina * A[0][1] + cosa * A[0][2];
    A[0][2] = -cosa * A[0][1] - sina * A[0][2];
    A[0][1] = x;

    x       = -sina * A[1][1] + cosa * A[1][2];
    A[1][2] = -cosa * A[1][1] - sina * A[1][2];
    A[1][1] = x;

    x       = -sina * A[2][1] + cosa * A[2][2];
    A[2][2] = -cosa * A[2][1] - sina * A[2][2];
    A[2][1] = x;

  }

  // Mults a matrix from the right to A, that represents a differential rotation around the y axis,
  // but with negative sin and cos. This is necessary if you use -alpha instead of alpha. [(f(-x))' = -f'(-x)]
  // Be aware, that after this multiplikation A is no longer a rotation Matrix!! 
  // This calculation is used to calculate the gradient/jacobi matrix of kinematic functions.


  inline void pseudoMultNegativeDiffRotY(float alpha) {
    
    float x;

    float sina = -sin(alpha);
    float cosa = -cos(alpha);

    A[0][1] = 0;
    A[1][1] = 0;
    A[2][1] = 0;

    x       = -sina * A[0][0] - cosa * A[0][2];
    A[0][2] =  cosa * A[0][0] - sina * A[0][2];
    A[0][0] = x;

    x       = -sina * A[1][0] - cosa * A[1][2];
    A[1][2] =  cosa * A[1][0] - sina * A[1][2];
    A[1][0] = x;

    x       = -sina * A[2][0] - cosa * A[2][2];
    A[2][2] =  cosa * A[2][0] - sina * A[2][2];
    A[2][0] = x;

  }

  // Mults a matrix from the right to A, that represents a differential rotation around the z axis,
  // but with negative sin and cos. This is necessary if you use -alpha instead of alpha. [(f(-x))' = -f'(-x)]
  // Be aware, that after this multiplikation A is no longer a rotation Matrix!! 
  // This calculation is used to calculate the gradient/jacobi matrix of kinematic functions.

  inline void pseudoMultNegativeDiffRotZ(float alpha) {
    
    float x;

    float sina = -sin(alpha);
    float cosa = -cos(alpha);

    A[0][2] = 0;
    A[1][2] = 0;
    A[2][2] = 0;

    x       = -sina * A[0][0] + cosa * A[0][1];
    A[0][1] = -cosa * A[0][0] - sina * A[0][1];
    A[0][0] = x;

    x       = -sina * A[1][0] + cosa * A[1][1];
    A[1][1] = -cosa * A[1][0] - sina * A[1][1];
    A[1][0] = x;

    x       = -sina * A[2][0] + cosa * A[2][1];
    A[2][1] = -cosa * A[2][0] - sina * A[2][1];
    A[2][0] = x;

  }

  // Mults 'right' from the right to this transformation

  inline void multRight(AffinTransformation* right) {

    float x,y;

    b[0] += A[0][0] * right->b[0] + A[0][1] * right->b[1] + A[0][2] * right->b[2];
    b[1] += A[1][0] * right->b[0] + A[1][1] * right->b[1] + A[1][2] * right->b[2];
    b[2] += A[2][0] * right->b[0] + A[2][1] * right->b[1] + A[2][2] * right->b[2];

    x       = A[0][0] * right->A[0][0] + A[0][1] * right->A[1][0] + A[0][2] * right->A[2][0];
    y       = A[0][0] * right->A[0][1] + A[0][1] * right->A[1][1] + A[0][2] * right->A[2][1];
    A[0][2] = A[0][0] * right->A[0][2] + A[0][1] * right->A[1][2] + A[0][2] * right->A[2][2];
    A[0][1] = y;
    A[0][0] = x;

    x       = A[1][0] * right->A[0][0] + A[1][1] * right->A[1][0] + A[1][2] * right->A[2][0];
    y       = A[1][0] * right->A[0][1] + A[1][1] * right->A[1][1] + A[1][2] * right->A[2][1];
    A[1][2] = A[1][0] * right->A[0][2] + A[1][1] * right->A[1][2] + A[1][2] * right->A[2][2];
    A[1][1] = y;
    A[1][0] = x;

    x       = A[2][0] * right->A[0][0] + A[2][1] * right->A[1][0] + A[2][2] * right->A[2][0];
    y       = A[2][0] * right->A[0][1] + A[2][1] * right->A[1][1] + A[2][2] * right->A[2][1];
    A[2][2] = A[2][0] * right->A[0][2] + A[2][1] * right->A[1][2] + A[2][2] * right->A[2][2];
    A[2][1] = y;
    A[2][0] = x;

    
  }

  // Create and returns a AffinTransformation, that is the inverted of this AffinTransformation (asuming that det A = 1)

  inline AffinTransformation* invert() {

    AffinTransformation* inverted = new AffinTransformation();

    inverted->A[0][0] = (A[1][1] * A[2][2] - A[2][1] * A[1][2]);
    inverted->A[0][1] = (A[2][1] * A[0][2] - A[0][1] * A[2][2]);
    inverted->A[0][2] = (A[0][1] * A[1][2] - A[1][1] * A[0][2]);
    inverted->A[1][0] = (A[2][0] * A[1][2] - A[1][0] * A[2][2]);
    inverted->A[1][1] = (A[0][0] * A[2][2] - A[2][0] * A[0][2]);
    inverted->A[1][2] = (A[1][0] * A[0][2] - A[0][0] * A[1][2]);
    inverted->A[2][0] = (A[1][0] * A[2][1] - A[2][0] * A[1][1]);
    inverted->A[2][1] = (A[2][0] * A[0][1] - A[0][0] * A[2][1]);
    inverted->A[2][2] = (A[0][0] * A[1][1] - A[1][0] * A[0][1]);

    inverted->b[0] = - inverted->A[0][0] * b[0] - inverted->A[0][1] * b[1] - inverted->A[0][2] * b[2];
    inverted->b[1] = - inverted->A[1][0] * b[0] - inverted->A[1][1] * b[1] - inverted->A[1][2] * b[2];
    inverted->b[2] = - inverted->A[2][0] * b[0] - inverted->A[2][1] * b[1] - inverted->A[2][2] * b[2];

    return inverted;

  }
  
};


#endif // __AffinTransformation_h__
