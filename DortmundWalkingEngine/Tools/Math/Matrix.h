/** 
 * \file Matrix.h
 * Template classes for various 3x3 matrices.
 *
 * \author <a href="mailto:martin.kallnik@gmx.de">Martin Kallnik</a>
 * \author <a href="mailto:thomas.kindler@gmx.de">Thomas Kindler</a> 
 * \author Max Risler
 */

#ifndef __Matrix_h__
#define __Matrix_h__

#include "Vector3.h"

/**
 * This class represents a 3x3-matrix 
 *
 */
template <class V = double> class Matrix3x3 
{
public:
  /** 
   * The columns of the matrix 
   */
  Vector3<V> c[3];

  /**
   * Default constructor. 
   */
  Matrix3x3<V>()
  {
    c[0]=Vector3<V>(1,0,0);
    c[1]=Vector3<V>(0,1,0);
    c[2]=Vector3<V>(0,0,1);
  }

  /**
   * Constructor.
   *
   * \param  c0  the first column of the matrix.
   * \param  c1  the second column of the matrix.
   * \param  c2  the third column of the matrix.
  */
  Matrix3x3<V>(
    const Vector3<V>& c0, 
    const Vector3<V>& c1, 
    const Vector3<V>& c2
  )
  {
    c[0] = c0;
    c[1] = c1;
    c[2] = c2;
  }

  /**
   * Anti-lateral thinking constructor.
   */
  Matrix3x3<V>(
    const V& a11, const V& a12, const V& a13,
    const V& a21, const V& a22, const V& a23,
    const V& a31, const V& a32, const V& a33)
  {
    c[0].x = a11; c[1].x = a12; c[2].x = a13;
    c[0].y = a21; c[1].y = a22; c[2].y = a23;
    c[0].z = a31; c[1].z = a32; c[2].z = a33;
  }

  /**
   * Assignment operator.
   *
   * \param  other   The other matrix that is assigned to this one
   * \return         A reference to this object after the assignment.
  */
  Matrix3x3<V>& operator=(const Matrix3x3<V>& other)
  {
    c[0] = other.c[0]; 
    c[1] = other.c[1]; 
    c[2] = other.c[2]; 
    return *this;
  }

  /**
   * Copy constructor.
   *
   * \param other The other matrix that is copied to this one
   */
  Matrix3x3<V>(const Matrix3x3<V>& other) 
  { 
    *this = other;
  }

  /**
   * Adds this matrix with another matrix.
   *
   * \param  other  The matrix this one is added to
   * \return         A new matrix containing the result
   *                 of the calculation.
  */
  Matrix3x3<V> operator+(const Matrix3x3<V>& other) const
  {
    return Matrix3x3<V>(
      (*this).c[0]+other.c[0], 
      (*this).c[1]+other.c[1], 
      (*this).c[2]+other.c[2]
    );
  }
   /**
   * Adds another matrix to this matrix.
   * 
   * \param  other  The other matrix that is added to this one
   * \return        A reference this object after the calculation.
  */
  Matrix3x3<V>& operator+=(const Matrix3x3<V>& other)
  {
    (*this).c[0]+=other.c[0]; 
    (*this).c[1]+=other.c[1]; 
    (*this).c[2]+=other.c[2];
    return *this;
  }

  /**
   * Compute difference of this matrix and another one
   *
   * \param  other  The matrix which is substracted from this one
   * \return         A new matrix containing the result
   *                 of the calculation.
  */
  Matrix3x3<V> operator-(const Matrix3x3<V>& other) const
  {
    return Matrix3x3<V>(
      (*this).c[0]-other.c[0], 
      (*this).c[1]-other.c[1], 
      (*this).c[2]-other.c[2]
    );
  }
   /**
   * Substracts another matrix from this one
   * 
   * \param  other  The other matrix that is substracted from this one
   * \return        A reference this object after the calculation.
  */
  Matrix3x3<V>& operator-=(const Matrix3x3<V>& other)
  {
    (*this).c[0]-=other.c[0]; 
    (*this).c[1]-=other.c[1]; 
    (*this).c[2]-=other.c[2];
    return *this;
  }

  /**
   * Multiplication of this matrix by vector.
   *
   * \param  vector  The vector this one is multiplied by 
   * \return         A new vector containing the result
   *                 of the calculation.
  */
  Vector3<V> operator*(const Vector3<V>& vector) const
  {
    return (c[0]*vector.x + c[1]*vector.y + c[2]*vector.z);
  }

  /**
   * Multiplication of this matrix by another matrix.
   *
   * \param  other  The other matrix this one is multiplied by 
   * \return        A new matrix containing the result
   *                of the calculation.
  */
  Matrix3x3<V> operator*(const Matrix3x3<V>& other) const
  {
    return Matrix3x3<V>(
      (*this)*other.c[0], 
      (*this)*other.c[1], 
      (*this)*other.c[2]
    );
  }

  /**
   * Multiplication of this matrix by another matrix.
   * 
   * \param  other  The other matrix this one is multiplied by 
   * \return        A reference this object after the calculation.
  */
  Matrix3x3<V>& operator*=(const Matrix3x3<V>& other)
  {
    return *this = *this * other;
  }

  /**
   * Multiplication of this matrix by a factor.
   *
   * \param  factor  The factor this matrix is multiplied by 
   * \return         A reference to this object after the calculation.
  */
  Matrix3x3<V>& operator*=(const V& factor)
  {
    c[0] *= factor;
    c[1] *= factor;
    c[2] *= factor;
    return *this;
  }

  /**
   * Division of this matrix by a factor.
   *
   * \param  factor  The factor this matrix is divided by 
   * \return         A reference to this object after the calculation.
   */
  Matrix3x3<V>& operator/=(const V& factor)
  {
    *this *= 1 / factor;
    return *this;
  }

  /**
   * Multiplication of this matrix by a factor.
   *
   * \param  factor  The factor this matrix is multiplied by 
   * \return         A new object that contains the result of the calculation.
   */
  Matrix3x3<V> operator*(const V& factor) const
  {
    return Matrix3x3<V>(*this) *= factor;
  }

  /**
   * Division of this matrix by a factor.
   *
   * \param  factor  The factor this matrix is divided by 
   * \return         A new object that contains the result of the calculation.
   */
  Matrix3x3<V> operator/(const V& factor) const
  {
    return Matrix3x3<V>(*this) /= factor;
  }

  /**
   * Comparison of another matrix with this one.
   *
   * \param  other  The other matrix that will be compared to this one
   * \return        Whether the two matrices are equal.
   */
  bool operator==(const Matrix3x3<V>& other) const
  {
    return (
      c[0] == other.c[0] && 
      c[1] == other.c[1] && 
      c[2] == other.c[2]
    );
  }

  /**
   * Comparison of another matrix with this one.
   *
   * \param  other  The other matrix that will be compared to this one
   * \return        Whether the two matrixs are unequal.
   */
  bool operator!=(const Matrix3x3<V>& other) const
  {
    return !(*this == other);
  }

  /**
   * Array-like member access.
   * \param  i index
   * \return reference to column
   */
  Vector3<V>& operator[](int i) 
  {
    return c[i];
  }
  
  /**
   * Transpose the matrix
   *
   * \return  A new object containing transposed matrix
   */
  Matrix3x3<V> transpose() const
  {
    return Matrix3x3<V>(
      Vector3<V>(c[0].x, c[1].x, c[2].x),
      Vector3<V>(c[0].y, c[1].y, c[2].y),
      Vector3<V>(c[0].z, c[1].z, c[2].z)
    );
  }
  
  /**
   * Calculation of the determinant of this matrix.
   * 
   * \return The determinant.
   */
  V det() const 
  { 
    return 
      c[0].x * (c[1].y * c[2].z - c[1].z * c[2].y) +
      c[0].y * (c[1].z * c[2].x - c[1].x * c[2].z) +
      c[0].z * (c[1].x * c[2].y - c[1].y * c[2].x);
  }
  
  /**
   * Calculate determinant of 2x2 Submatrix  
   * | a b |
   * | c d |
   *
   * \return  determinant.
   */
  static V det2(V a, V b, V c, V d)
  {
    return a*d - b*c;
  }

  /**
   * Calculate the adjoint of this matrix.
   *
   * \return the adjoint matrix.
   */
  Matrix3x3<V> adjoint() const
  {
    return Matrix3x3<V>(
      Vector3<V>(
        det2(c[1].y, c[2].y, c[1].z, c[2].z), 
        det2(c[2].x, c[1].x, c[2].z, c[1].z), 
        det2(c[1].x, c[2].x, c[1].y, c[2].y)
      ),
      Vector3<V>(
        det2(c[2].y, c[0].y, c[2].z, c[0].z), 
        det2(c[0].x, c[2].x, c[0].z, c[2].z), 
        det2(c[2].x, c[0].x, c[2].y, c[0].y)
      ),
      Vector3<V>(
        det2(c[0].y, c[1].y, c[0].z, c[1].z), 
        det2(c[1].x, c[0].x, c[1].z, c[0].z), 
        det2(c[0].x, c[1].x, c[0].y, c[1].y)      
      )
    );
  
  }  

  /**
   * Calculate the inverse of this matrix.
   *
   * \return The inverse matrix
   */
  Matrix3x3<V> invert() const
  {
    return adjoint().transpose() / det();
  }
};


/**
 * Streaming operator that reads a Matrix3x3<V> from a stream.
 *
 * \param   stream     The stream from which is read.
 * \param   matrix3x3  The Matrix3x3<V> object.
 * \return  The stream.
 */ 
template <class V> In& operator>>(In& stream, Matrix3x3<V>& matrix3x3);


/**
 * Streaming operator that writes a Matrix3x3<V> to a stream.
 *
 * \param   stream      The stream to write on.
 * \param   matrix3x3   The Matrix3x3<V> object.
 * \return  The stream.
 */ 
template <class V> Out& operator<<(Out& stream, const Matrix3x3<V>& matrix3x3);


/**
 * Representation for 3x3 RotationMatrices
 */
class RotationMatrix : public Matrix3x3<double>
{
public:
  /** 
   * Default constructor. 
   */
  RotationMatrix() {}

  /**
   * Constructor.
   *
   * \param  c0  the first column of the matrix.
   * \param  c1  the second column of the matrix.
   * \param  c2  the third column of the matrix.
   */
  RotationMatrix(const Vector3<double>& c0, const Vector3<double>& c1, 
    const Vector3<double>& c2);

  /**
   * Assignment operator.
   * 
   * \param  other  The other matrix that is assigned to this one
   * \return        A reference to this object after the assignment.
   */
  RotationMatrix& operator=(const Matrix3x3<double>& other)
  {
    c[0] = other.c[0]; 
    c[1] = other.c[1]; 
    c[2] = other.c[2]; 
    return *this;
  }

  /**
   * Copy constructor.
   *
   * \param  other  The other matrix that is copied to this one
   */
  RotationMatrix(const Matrix3x3<double>& other)
  {
    *this = other;
  }
  
  /**
   * RotationMatrix from RPY-angles.
   *   Roll  rotates along z axis,
   *   Pitch rotates along y axis,  
   *   Yaw   rotates along x axis
   *
   *   R(roll,pitch,yaw) = R(z,roll)*R(y,pitch)*R(x,yaw)
   *
   * \see  "Robotik 1 Ausgabe Sommersemester 2001" by Prof. Dr. O. von Stryk
   * \attention  RPY-angles are not clearly defined!
   */
  RotationMatrix(const double& yaw, const double& pitch, const double& roll);

  /**
   * RotationMatrix from rotation around any axis.
   * \param axis The axis.
   * \param angle The angle to rotate around the axis.
   */
  RotationMatrix(const Vector3<double>& axis, const double& angle);

  /**
   * RotationMatrix from rotation around any axis with an angle given as the length of the axis.
   * \param axis The axis.
   */
  RotationMatrix(const Vector3<double>& axis);

  /**
   * Invert the matrix.
   *
   * \note: Inverted rotation matrix is transposed matrix.
   */
  RotationMatrix invert() const
  {
    return transpose();
  }

  /** 
   * Rotation around the x-axis.
   *
   * \param   angle  The angle this pose will be rotated by
   * \return  A reference to this object after the calculation.
   */
  RotationMatrix& rotateX(const double angle);

  /** 
   * Rotation around the y-axis.
   *
   * \param   angle  The angle this pose will be rotated by
   * \return  A reference to this object after the calculation.
   */
  RotationMatrix& rotateY(const double angle);

  /** 
   * Rotation around the z-axis.
   *
   * \param   angle  The angle this pose will be rotated by
   * \return  A reference to this object after the calculation.
   */
  RotationMatrix& rotateZ(const double angle);

  /**
   * Get the x-angle of a RotationMatrix.
   *
   * \return  The angle around the x-axis between the original
   *          and the rotated z-axis projected on the y-z-plane
   */
  double getXAngle() const;

  /**
   * Get the y-angle of a RotationMatrix.
   *
   * \return  The angle around the y-axis between the original
   *          and the rotated x-axis projected on the x-z-plane
   */
  double getYAngle() const;

  /**
   * Get the z-angle of a RotationMatrix.
   *
   * \return  The angle around the z-axis between the original
   *          and the rotated x-axis projected on the x-y-plane
   */
  double getZAngle() const;

  /**
   * Create and return a RotationMatrix, rotated around x-axis
   *
   * \param   angle 
   * \return  rotated RotationMatrix
   */
  static RotationMatrix fromRotationX(const double angle)
  {
    return RotationMatrix().rotateX(angle);
  }

  /**
   * Create and return a RotationMatrix, rotated around y-axis
   *
   * \param   angle 
   * \return  rotated RotationMatrix
   */
  static RotationMatrix fromRotationY(const double angle)
  {
    return RotationMatrix().rotateY(angle);
  }

  /**
   * Create and return a RotationMatrix, rotated around z-axis
   *
   * \param   angle 
   * \return  rotated RotationMatrix
   */
  static RotationMatrix fromRotationZ(const double angle)
  {
    return RotationMatrix().rotateZ(angle);
  }

  void getAngleAxis(Vector3<double>& axis, double& angle) const;
};

/**
 * Streaming operator that reads a RotationMatrix from a stream.
 *
 * \param  stream          The stream from which is read.
 * \param  rotationMatrix  The RotationMatrix object.
 * \return The stream.
 */ 
In& operator>>(In& stream, RotationMatrix& rotationMatrix);

/**
 * Streaming operator that writes a RotationMatrix to a stream.
 *
 * \param  stream          The stream to write on.
 * \param  rotationMatrix  The RotationMatrix object.
 * \return The stream.
 */ 
Out& operator<<(Out& stream, const RotationMatrix& rotationMatrix);



#endif // __Matrix_h__
