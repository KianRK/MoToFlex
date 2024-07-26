/** 
 * @file Matrix_mxn.h
 * Implements a class for calculations with Matrixes with variable size.
 * Warning!!! Use this class with caution! As it is used for numerical calculations
 * it is optimized for fast execution. So generaly no sanitiy checks are done.
 * If you are unsure please use another class.
 *
 *    @author <A href=mailto:gregor.jochmann@uni-dortmund.de>Gregor Jochmann</A>
 *    @author <a href="mailto:stefan.czarnetzki@tu-dortmund.de">Stefan Czarnetzki</a>
 *
 * Solve and invert are taken from class Matrix_nxn.h (which was not used otherwise)
 *    @author <a href="mailto:stefanuhrig@gmx.net">Stefan Uhrig</a>
 *
 */

#ifndef __Matrix_mxn_h__
#define __Matrix_mxn_h__


#include "MVTools.h"
#include "Vector_n.h"
#include "Matrix2x2.h"
#include "Tools/Debugging/Debugging.h"
#include "Platform/GTAssert.h"


//------------------------------------------------------------------------------
/**
* @class Matrix_mxn
* Represents a mxn matrix of type double (M rows, N columns)
*
*/
template<size_t M, size_t N>
class Matrix_mxn: public Streamable
{

public:
  static const size_t componentCount = M*N;
  static const size_t memSize = M*N*sizeof(double);
  double content[componentCount];

  //constructors
  //default constructor, init with T()
  Matrix_mxn()
  {
    clear();
  }

  //Constructor from array
  Matrix_mxn(double contentArray[])
  {
    memcpy(content, contentArray, memSize);
  }

  //copy constructor
  Matrix_mxn(const Matrix_mxn<M, N>& source)
  {
    memcpy(content, source.content, memSize);
  }

  //constructor from Vector - vector multiplication (calculate vA * vB_transposed)
  Matrix_mxn(const Vector_n<double, M>& vA, const Vector_n<double, N>& vB)
  {
    for (size_t row = 0; row < M; row++)
    {
      for (size_t col = 0; col < N; col++)
      {
        setMember(row, col, vA[row] * vB[col]);
      }
    }
  }

  //destructor
  ~Matrix_mxn(){}

  virtual void serialize(In* in, Out* out)
  {
    STREAM_REGISTER_BEGIN();
    STREAM_ARRAY(content);
    STREAM_REGISTER_FINISH();
  }

  //copy operator
  Matrix_mxn<M, N>& operator=(const Matrix_mxn<M, N>& source)
  {    
    memcpy(content, source.content, memSize);
    return *this;
  }
  Matrix_mxn<2, 2>& operator=(const Matrix2x2<double>& source)
  {
    //c[0].x = a11; c[1].x = a12;
    //c[0].y = a21; c[1].y = a22;
	setMember(0, 0, source.c[0].x);
	setMember(1, 0, source.c[0].y);
	setMember(0, 1, source.c[1].x);
	setMember(1, 1, source.c[1].y);
    return *this;
  }

  size_t getRowCount() const
  {
      return M;
  }

  size_t getColumnCount() const
  {
      return N;
  }

  //cell access (zero-based)
  double getMember(size_t row, size_t col) const
  {
      return content[row * N + col];
  }

  //column access (first column is 0)
  Vector_n<double, M> getColumn(size_t col) const
  {
    Vector_n<double, M> result;
    for (unsigned int row = 0; row < M; row++)
    {
      result[row] = getMember(row, col);
    }
    return result;
  }
  //column access (first column is 0)
  void setColumn(size_t col, const Vector_n<double, M> & columnVector)
  {
    for (unsigned int row = 0; row < M; row++)
    {
      setMember(row, col, columnVector[row]);
    }
  }
  //row access (first row is 0)
  Vector_n<double, N> getRow(size_t row) const
  {
    Vector_n<double, N> result;
    for (unsigned int col = 0; col < N; col++)
    {
      result[col] = getMember(row, col);
    }
    return result;
  }
  //row access (first row is 0)
  void setRow(size_t row, const Vector_n<double, N> & rowVector)
  {
    for (unsigned int col = 0; col < N; col++)
    {
      setMember(row, col,rowVector[col]);
    }
  }
  //row access (first row is 0)
  void addToRow(size_t row, const Vector_n<double, N> & rowVector)
  {
    for (unsigned int col = 0; col < N; col++)
    {
      content[row * N + col] += rowVector[col];
    }
  }
  
  //faster add without intermediate class instantiation
  void addToThisMatrix(const Matrix_mxn<M,N> & other)
  {
    for (size_t row = 0; row < M; row++)
    {
      for (size_t col = 0; col < N; col++)
      {
        addToMember(row, col, other.getMember(row,col));
      }
    }
  }

  // You get the member at the position m,n as if you have the transposed matrix of A.

  double getMemberTransposed(size_t row, size_t col)
  {
      return content[col * N + row];
  }

  void setMember(size_t row, size_t col, double value)
  {
    content[row * N + col] = value;
  }
  void addToMember(size_t row, size_t col, double value)
  {
    content[row * N + col] += value;
  }

  //reset to zero for all members
  void clear()
  {
    memset(content, 0, memSize);
  }

  //reset a square matrix to I
  void setToIdentity()
  {
    if (M != N)
    {
      throw MVException(MVException::MatrixNotSquare);
    }
    clear();
    for (unsigned int i = 0; i < N; i++)
    {
      setMember(i,i,1.0);
    }
  }

  // for debugging: Is this matrix a valid covariance?
  bool isValidCovariance() const
  {
    if (M != N)
    {
      return false;
    }
    else
    {
      bool validCovariance = true;
      for (unsigned int i=0; i < N; i++)
      {
        validCovariance = (validCovariance && (getMember(i,i)>0));
      }
      return validCovariance;
    }
  }


  // -------------- Operators ---------------------------------------

  // +=
  Matrix_mxn<M, N>& operator+=(const Matrix_mxn<M, N>& other)
  {
    for (unsigned int i = 0; i < componentCount; i++)
    {
      content[i] = content[i] + other.content[i];
    }

    return *this;
  }

  // -=
  Matrix_mxn<M, N>& operator-=(const Matrix_mxn<M, N>& other)
  {
    for (unsigned int i = 0; i < componentCount; i++)
    {
      content[i] = content[i] - other.content[i];
    }

    return *this;
  }

  // /=
  Matrix_mxn<M, N>& operator*=(const double& factor)
  {
    for (unsigned int i = 0; i < componentCount; i++)
    {
      content[i] = content[i] * factor;
    }

    return *this;
  }

  // /=
  Matrix_mxn<M, N>& operator/=(const double& factor)
  {
    for (unsigned int i = 0; i < componentCount; i++)
    {
      content[i] = content[i] / factor;
    }

    return *this;
  }

  
  


  //---------------------- other functions -------------------
    //transposes this matrix
  Matrix_mxn<N, M> transpose()
  {
    Matrix_mxn<N, M> result;
    
    for (unsigned int row = 0; row < N; row++)
    {
      for (unsigned int col = 0; col < M; col++)
      {
        result.setMember(row, col, getMember(col, row));
      }
    }
    return result;
  }

  //----------------------------------------------------------------------------
  /**
  * Calculates the square root of this matrix using cholesky decomposition
  * pseudocode from http://de.wikipedia.org/wiki/Cholesky-Zerlegung
  *
  * requires (this) to be a positive definite symmetric matrix
  *
  * Sqrt-interpretation: (this) = G * G_transposed
  * 
  * when successful, returns contains G (lower triangular matrix)
  */
  Matrix_mxn<M, N> choleskySqrt()
  {
    if (M != N)
    {
      throw MVException(MVException::MatrixNotSquare);
    }

    Matrix_mxn<M, N> result;

    //in general: A = LL_transposed --
    //            for j = 1..N, i>j (== i = j+1 .. N, calculate 
    //            L_ii = sqrt( a_ii - sum_k=1^i-1(L_ik^2)
    //            L_ij = (A_ij - sum_k=1^j-1(L_ik * L_jk))/L_jj
    //            Note: use L_ji = ... for upper triangular matrix
    //row by row calculation (Cholesky-Banachiewicz algorithm)
    
    double tempSum;

    for (unsigned int i = 0; i < N; i++) // i : row index
    {
      for (unsigned int j = 0; j <= i ; j++) // j: column index
      {
        //L_ij
        tempSum = getMember(i, j);
        for (unsigned int k = 0; k < j; k++)
        {
          tempSum -= result.getMember(i,k) * result.getMember(j,k);
        }

        if ( i == j)
        {
          if (tempSum <= 0)
          {
            result.setMember(i, i, 0.0001);
          }
          else
          {
            result.setMember(i, i, sqrt(tempSum));
          }
        }
        else
        {
          result.setMember(i, j, tempSum / result.getMember(j,j));
        }
        
      }//end for columns

    }// end for rows/i
      
    return result;
  }

  // returns the euclidic norm of this matrix.
  double abs()
  {
    double c = 0;

    for (int i = 0; i < componentCount; i++) c += sqr(content[i]);

    return sqrt(c);
  }

  //return sum of elements on main diagonal
  const double spurSum() const
  {
    double result = 0;
    for (unsigned int i=0; i< M; i++)
    {
      result += getMember(i,i);
    }
    return result;
  }

  // Create and returns the inverted of A. 
  // A must be sqare and must not be singular.
  Matrix_mxn<M, M> invert()
  {
    if (M != N)
    {
      throw MVException(MVException::MatrixNotSquare);
    }

    if (M==2) // hopefully the compiler is clever enough to remove the unnecessary branch...
    {
      double a = getMember(0,0);
      double b = getMember(0,1);
      double c = getMember(1,0);
      double d = getMember(1,1);
      double det = a*d - b*c;
      if (MVTools::isNearZero(det))
      {
        if (MVTools::isNearNegZero(det))
          throw MVException(MVException::DivByNegZero);
        else
          throw MVException(MVException::DivByPosZero);
      }
      double detInv = 1.0 / det;
      double invM[4] = {d*detInv,-b*detInv,-c*detInv,a*detInv};
      return Matrix_mxn<M, M>( invM );
    }
    else
    {
      
      Matrix_mxn<N, N> left(*this);
      Matrix_mxn<N, N> right;
      right.setToIdentity();
      Vector_n<int, N> ranking;

      int i;
      for (i = 0; i < (int)N; ++i) 
      {
         ranking[i] = i;
      }
      double zero = 0.0;
      int r, r2, maxrow;
      for (r = 0; r < (int)(N-1); ++r)
      {
        // find highest value
        double maxval = left.getMember(ranking[r],r);
        maxrow = r;
        if (maxval < zero)
          maxval = -maxval;
        for (r2 = r+1; r2 < (int)N; ++r2)
        {
          double val = left.getMember(ranking[r2],r);
          if (val < zero)
            val = -val;
          if (val > maxval)
          {
            maxval = val;
            maxrow = r2;
          }
        }
        // swap rows
        int temp = ranking[r];
        ranking[r] = ranking[maxrow];
        ranking[maxrow] = temp;
        
        if (MVTools::isNearZero(left.getMember(ranking[r],r)))
        {
          if (MVTools::isNearNegZero(left.getMember(ranking[r],r)))
            throw MVException(MVException::DivByNegZero);
          else
            throw MVException(MVException::DivByPosZero);
        }
        
        for (r2 = r+1; r2 < (int)N; ++r2)
        {
          // calc factor for subtracting
          double factor = left.getMember(ranking[r2],r) / left.getMember(ranking[r],r);
          if (MVTools::isNearInf(factor))
          {
            if (MVTools::isNearPosInf(factor))
              throw MVException(MVException::PosInfValue);
            else
              throw MVException(MVException::NegInfValue);
          }
          
          // change left matrix
          left.addToRow(ranking[r2], (-factor)*left.getRow(ranking[r]));
          
          // change right matrix
          right.addToRow(ranking[r2], (-factor)*right.getRow(ranking[r]));
        }
      }
      // matrix has triangle form
      // bring to diagonal form
      for (r = (int)(N-1); r > 0; --r)
      {
        if (MVTools::isNearZero(left.getMember(ranking[r],r)))
        {
          if (MVTools::isNearNegZero(left.getMember(ranking[r],r)))
            throw MVException(MVException::DivByNegZero);
          else
            throw MVException(MVException::DivByPosZero);
        }
        for (r2 = r-1; r2 >= 0; --r2)
        {
          double factor = left.getMember(ranking[r2],r) / left.getMember(ranking[r],r);
          if (MVTools::isNearInf(factor))
          {
            if (MVTools::isNearPosInf(factor))
              throw MVException(MVException::PosInfValue);
            else
              throw MVException(MVException::NegInfValue);
          }

          // change left matrix
          left.addToRow(ranking[r2], (-factor)*left.getRow(ranking[r]));
          
          // change right matrix
          right.addToRow(ranking[r2], (-factor)*right.getRow(ranking[r]));
        }
      }
      // matrix has diagonal form
      // set entries of left matrix to 1 and apply multiplication to right
      Matrix_mxn<M, M> res;
      for (r = 0; r < (int)N; ++r)
      {
        res.setRow(r, right.getRow(ranking[r]));
        
        if (MVTools::isNearZero(left.getMember(ranking[r],r)))
        {
          if (MVTools::isNearNegZero(left.getMember(ranking[r],r)))
            throw MVException(MVException::DivByNegZero);
          else
            throw MVException(MVException::DivByPosZero);
        }
        res.setRow(r, res.getRow(r) / left.getMember(ranking[r],r));
      }
      return res;
    }
  }
  



  
  //----------------------------------------------------------------------------
  /**
  * Solves the system A*x=b where A is the actual matrix (which must be square!)
  * @param b Vector b
  * @return Solution x
  *
  * Complexity: n^3
  *
  */
  Vector_n<double, N> solve(Vector_n<double, N> b) const
  {
    // create copy of actual matrix
    Matrix_mxn<N, N> m(*this);
    
    // initialize ranking vector
    Vector_n<int, N> ranking;
    size_t i;
    for (i = 0; i < N; ++i)
      ranking[i] = i;
    
    double z = 0.0;
    int c;
    int r;
    for (c = 0; c < (int)N-1; ++c)
    {
      // find row containing highest value
      int maxRow = c;
      double maxValue = m.getMember(ranking[maxRow],c);
      if (maxValue < z)
        maxValue = -maxValue;
      for (r = c+1; r < (int)N; ++r)
      {
        double value = m.getMember(ranking[r],c);
        if (value < z)
          value = -value;
        if (value > maxValue)
        {
          maxRow = r;
          maxValue = value;
        }
      }
      
      // if maximum value zero --> matrix is singular
      if (MVTools::isNearZero(maxValue))
      {
        if (MVTools::isNearNegZero(maxValue))
          throw MVException(MVException::DivByNegZero);
        else
          throw MVException(MVException::DivByPosZero);
      }
      /*
      if (maxValue == z)
      return Vector_n<T, N>();
      */
      
      // swap rows in ranking
      int temp = ranking[c];
      ranking[c] = ranking[maxRow];
      ranking[maxRow] = temp;
      
      // process all following rows
      for (r = c+1; r < (int)N; ++r)
      {
        // calc factor for subtracting
        double factor = m.getMember(ranking[r],c) / m.getMember(ranking[c],c);
        if (MVTools::isNearInf(factor))
        {
          if (MVTools::isNearPosInf(factor))
            throw MVException(MVException::PosInfValue);
          else
            throw MVException(MVException::NegInfValue);
        }
        
        double sub;
        sub = factor*b[ranking[c]];
        if (MVTools::isNearInf(sub))
        {
          if (MVTools::isNearPosInf(sub))
            throw MVException(MVException::PosInfValue);
          else
            throw MVException(MVException::NegInfValue);
        }
        
        // change vector b
        b[ranking[r]] -= sub;
        
        // change matrix
        m.setMember(ranking[r],c, 0.0);
        for (int c2 = c+1; c2 < (int)N; ++c2)
        {
          sub = factor*m.getMember(ranking[c],c2);
          if (MVTools::isNearInf(sub))
          {
            if (MVTools::isNearPosInf(sub))
              throw MVException(MVException::PosInfValue);
            else
              throw MVException(MVException::NegInfValue);
          }
          //m.setMember(ranking[r],c2, m.getMember(ranking[r],c2) - sub);
          m.addToMember(ranking[r],c2, -sub);
        }
      }
    }
    
    // if last entry of matrix zero --> matrix is singular
    if (MVTools::isNearZero(m.getMember(ranking[N-1],N-1)))
    {
      if (MVTools::isNearNegZero(m.getMember(ranking[N-1],N-1)))
        throw MVException(MVException::DivByNegZero);
      else
        throw MVException(MVException::DivByPosZero);
    }
    /*
    if (m[ranking[N-1]][N-1] == z)
    return Vector_n<T, N>();
    */
    
    // matrix has triangle form
    // calculate solutions
    b[ranking[N-1]] /= m.getMember(ranking[N-1],N-1);
    for (r = N-2; r >= 0; --r)
    {
      double sum = 0.0;
      for (c = r+1; c < (int)N; ++c)
        sum += m.getMember(ranking[r],c) * b[ranking[c]];
      if (MVTools::isNearInf(sum))
      {
        if (MVTools::isNearPosInf(sum))
          throw MVException(MVException::PosInfValue);
        else
          throw MVException(MVException::NegInfValue);
      }
      
      if (MVTools::isNearZero(m.getMember(ranking[r],r)))
      {
        if (MVTools::isNearNegZero(m.getMember(ranking[r],r)))
          throw MVException(MVException::DivByNegZero);
        else
          throw MVException(MVException::DivByPosZero);
      }
      b[ranking[r]] = (b[ranking[r]] - sum) / m.getMember(ranking[r],r);
      
      if (MVTools::isNearInf(b[ranking[r]]))
      {
        if (MVTools::isNearPosInf(b[ranking[r]]))
          throw MVException(MVException::PosInfValue);
        else
          throw MVException(MVException::NegInfValue);
      }
    }
    
    // create vector with correct order
    Vector_n<double, N> x;
    for (r = 0; r < (int)N; ++r)
      x[r] = b[ranking[r]];
    
    return x;
  }

  /**
   * returns the upper left entries as 3x3 Matrix
   */
  Matrix_mxn<3, 3> upperLeft3x3() const
  {
    ASSERT(M >= 3 && N >= 3);
    
    Matrix_mxn<3, 3> result;
    for (size_t i = 0; i<3; i++)
    {
      for (size_t j = 0; j<3; j++)
      {
        result.setMember(i, j, getMember(i, j));
      }
    }
    return result;
  }

  bool isSymmetric() const
  {
    if (M != N) return false;

    double tolerance = 0.0001;
    for (size_t i = 0; i<M; i++)
    {
      for (size_t j = 0; j<M, j != i; j++)
      {
        if (fabs(getMember(i,j) - getMember(j,i)) > tolerance)
        {
          return false;
        }
      }
    }

    return true;
  }

  //print matrix to debug output
  void print(std::string matrixName)
  {
    OUTPUT(idText, text, "------" << matrixName << " ( "<< M << "x"<<N<<" Matrix)----\n");
    for (size_t row = 0; row < M; row++)
    {
      std::string rowAsText;
      for (size_t col = 0; col < N; col++)
      {
        rowAsText << getMember(row, col) << ",\t";
      }
      OUTPUT(idText, text, rowAsText);
    }
  }

};

//------------- Binary operators -------------------------

//Matrix multiplication
template<size_t A, size_t B, size_t C, size_t D>
Matrix_mxn<A, D>  operator*(const Matrix_mxn<A, B>& left,
                            const Matrix_mxn<C, D>& right)
{
  if (B != C) {
    throw MVException(MVException::MatrixDimensionMismatch);
  }
  Matrix_mxn<A,D> result;

  unsigned int i,j,k;
  double c;

  for (i = 0; i < A; i++) {
    for (j = 0; j < D; j++) {
      c = 0;
      for (k = 0; k < B; k++) {
        //double leftCell = left.getMember(i, k);
        //double rightCell =  right.getMember(k, j);
        c += left.getMember(i, k) * right.getMember(k, j);
      }
      result.setMember(i, j, c);  
    }
  }

  return result;
}
//vector(transposed) * Matrix
//returns vector to be treated as row-vector
template<size_t A, size_t B, size_t N>
Vector_n<double, A>  operator*(const Vector_n<double, N>& left,
                               const Matrix_mxn<A, B>& right
                                )
{
  if (B != N) {
    throw MVException(MVException::MatrixDimensionMismatch);
  }
  Vector_n<double, A> result;
  //Like Matrix-matrix multiplication
  // with C = N, D = 1

  unsigned int i,k;
  double c;

  for (i = 0; i < A; i++) {
    //j == 0;
    c = 0;
    for (k = 0; k < B; k++) {
      c += left[k] * right.getMember(k, i);
    }
    result[i] = c;  
  }

  return result;
}

//Matrix - vector multiplication
template<size_t A, size_t B, size_t N>
Vector_n<double, A>  operator*(const Matrix_mxn<A, B>& left,
                                const Vector_n<double, N>& right)
{
  if (B != N) {
    throw MVException(MVException::MatrixDimensionMismatch);
  }
  Vector_n<double, A> result;
  //Like Matrix-matrix multiplication
  // with C = N, D = 1

  unsigned int i,k;
  double c;

  for (i = 0; i < A; i++) {
    //j == 0;
    c = 0;
    for (k = 0; k < B; k++) c += left.getMember(i, k) * right[k];
    result[i] = c;  
  }

  return result;
}

template<size_t M, size_t N>
Matrix_mxn<M, N> operator*( const Matrix_mxn<M, N>& matrix,
                            const double& factor)
{
  Matrix_mxn<M, N> result;
  
  for (unsigned int i = 0; i < Matrix_mxn<M,N>::componentCount; i++){
    result.content[i] = matrix.content[i] * factor;
  }
  return result;
}

template<size_t M, size_t N>
Matrix_mxn<M, N> operator/( const Matrix_mxn<M, N>& matrix,
                            const double& factor)
{
  return matrix * (1.0/factor);
}

template<size_t M, size_t N>
Matrix_mxn<M, N> operator+(const Matrix_mxn<M, N>& left,
                           const Matrix_mxn<M, N>& right)
{
  Matrix_mxn<M, N> res(left);
  res += right;
  return res;
}

template<size_t M, size_t N>
Matrix_mxn<M, N> operator-(const Matrix_mxn<M, N>& left,
                           const Matrix_mxn<M, N>& right)
{
  Matrix_mxn<M, N> res(left);
  res -= right;
  return res;
}

template<size_t M, size_t N>
bool operator ==( const Matrix_mxn<M, N>& left, const Matrix_mxn<M, N>& right)
{
  double tolerance = 0.00001;

  for (unsigned int i = 0; i < Matrix_mxn<M,N>::componentCount; i++){
    if (fabs(left.content[i] - right.content[i]) > tolerance) {
      return false;
    }
  }
  return true;
}

template<size_t M, size_t N>
bool operator !=( const Matrix_mxn<M, N>& left, const Matrix_mxn<M, N>& right)
{
  return !(left == right);
}

#endif // __Matrix_mxn_h__
