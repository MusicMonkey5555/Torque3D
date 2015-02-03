//-----------------------------------------------------------------------------
// Copyright (c) 2012 GarageGames, LLC
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
//-----------------------------------------------------------------------------

#ifndef _MMATRIX_H_
#define _MMATRIX_H_

#ifndef _MPLANE_H_
#include "math/mPlane.h"
#endif

#ifndef _MBOX_H_
#include "math/mBox.h"
#endif

#ifndef _MPOINT4_H_
#include "math/mPoint4.h"
#endif

/// 4x4 Matrix Class
///
/// This runs at F32 precision.

class MatrixF
{
private:
   F32 m[16];     ///< Note: Torque uses row-major matrices

public:
   /// Create an uninitialized matrix.
   ///
   /// @param   identity    If true, initialize to the identity matrix.
   explicit MatrixF(bool identity=false);

   /// Create a matrix to rotate about origin by e.
   /// @see set
   explicit MatrixF( const EulerF &e);

   /// Create a matrix to rotate about p by e.
   /// @see set
   MatrixF( const EulerF &e, const Point3F& p);

   /// Get the index in m to element in column i, row j
   ///
   /// This is necessary as we have m as a one dimensional array.
   ///
   /// @param   i   Column desired.
   /// @param   j   Row desired.
   static U32 idx(U32 i, U32 j) { return (i + j*4); }

   /// Initialize matrix to rotate about origin by e.
   MatrixF& set( const EulerF &e);

   /// Initialize matrix to rotate about p by e.
   MatrixF& set( const EulerF &e, const Point3F& p);

   /// Initialize matrix with a cross product of p.
   MatrixF& setCrossProduct( const Point3F &p);

   /// Initialize matrix with a tensor product of p.
   MatrixF& setTensorProduct( const Point3F &p, const Point3F& q);

   operator F32*() { return (m); }              ///< Allow people to get at m.
   operator const F32*() const { return (F32*)(m); }  ///< Allow people to get at m.

   bool isAffine() const;                       ///< Check to see if this is an affine matrix.
   bool isIdentity() const;                     ///< Checks for identity matrix.

   /// Make this an identity matrix.
   MatrixF& identity();

   /// Invert m.
   MatrixF& inverse();
   /// Copy the inversion of this into out matrix.
   void invertTo( MatrixF *out );

   /// Take inverse of matrix assuming it is affine (rotation,
   /// scale, sheer, translation only).
   MatrixF& affineInverse();           

   /// Swap rows and columns.
   MatrixF& transpose();

   /// M * Matrix(p) -> M
   MatrixF& scale( const Point3F &s );            
   MatrixF& scale( F32 s ) { return scale( Point3F( s, s, s ) ); }

   /// Return scale assuming scale was applied via mat.scale(s).
   Point3F getScale() const;

   EulerF toEuler() const;

   /// Compute the inverse of the matrix.
   ///
   /// Computes inverse of full 4x4 matrix. Returns false and performs no inverse if
   /// the determinant is 0.
   ///
   /// Note: In most cases you want to use the normal inverse function.  This method should
   ///       be used if the matrix has something other than (0,0,0,1) in the bottom row.
   bool fullInverse();

   /// Swaps rows and columns into matrix.
   void transposeTo(F32 *matrix) const;

   /// Normalize the matrix.
   void normalize();

   /// Copy the requested column into a Point4F.
   void getColumn(S32 col, Point4F *cptr) const;
   Point4F getColumn4F(S32 col) const { Point4F ret; getColumn(col,&ret); return ret; }

   /// Copy the requested column into a Point3F.
   ///
   /// This drops the bottom-most row.
   void getColumn(S32 col, Point3F *cptr) const;
   Point3F getColumn3F(S32 col) const { Point3F ret; getColumn(col,&ret); return ret; }

   /// Set the specified column from a Point4F.
   void setColumn(S32 col, const Point4F& cptr);

   /// Set the specified column from a Point3F.
   ///
   /// The bottom-most row is not set.
   void setColumn(S32 col, const Point3F& cptr);

   /// Copy the specified row into a Point4F.
   void getRow(S32 row, Point4F *cptr) const;
   Point4F getRow4F(S32 row) const { Point4F ret; getRow(row,&ret); return ret; }

   /// Copy the specified row into a Point3F.
   ///
   /// Right-most item is dropped.
   void getRow(S32 row, Point3F *cptr) const;
   Point3F getRow3F(S32 row) const { Point3F ret; getRow(row,&ret); return ret; }

   /// Set the specified row from a Point4F.
   void setRow(S32 row, const Point4F& cptr);

   /// Set the specified row from a Point3F.
   ///
   /// The right-most item is not set.
   void setRow(S32 row, const Point3F& cptr);

   /// Get the position of the matrix.
   ///
   /// This is the 4th column of the matrix.
   Point3F getPosition() const;

   /// Set the position of the matrix.
   ///
   /// This is the 4th column of the matrix.
   void setPosition( const Point3F &pos ) { setColumn( 3, pos ); }

   /// Add the passed delta to the matrix position.
   void displace( const Point3F &delta );

   /// Get the x axis of the matrix.
   ///
   /// This is the 1st column of the matrix and is
   /// normally considered the right vector.
   VectorF getRightVector() const;

   /// Get the y axis of the matrix.
   ///
   /// This is the 2nd column of the matrix and is
   /// normally considered the forward vector.   
   VectorF getForwardVector() const;   

   /// Get the z axis of the matrix.
   ///
   /// This is the 3rd column of the matrix and is
   /// normally considered the up vector.   
   VectorF getUpVector() const;

   MatrixF&  mul(const MatrixF &a);                    ///< M * a -> M
   MatrixF&  mulL(const MatrixF &a);                   ///< a * M -> M
   MatrixF&  mul(const MatrixF &a, const MatrixF &b);  ///< a * b -> M

   // Scalar multiplies
   MatrixF&  mul(const F32 a);                         ///< M * a -> M
   MatrixF&  mul(const MatrixF &a, const F32 b);       ///< a * b -> M


   void mul( Point4F& p ) const;                       ///< M * p -> p (full [4x4] * [1x4])
   void mulP( Point3F& p ) const;                      ///< M * p -> p (assume w = 1.0f)
   void mulP( const Point3F &p, Point3F *d) const;     ///< M * p -> d (assume w = 1.0f)
   void mulV( VectorF& p ) const;                      ///< M * v -> v (assume w = 0.0f)
   void mulV( const VectorF &p, Point3F *d) const;     ///< M * v -> d (assume w = 0.0f)

   void mul(Box3F& b) const;                           ///< Axial box -> Axial Box
   
   MatrixF& add( const MatrixF& m );

   /// Convenience function to allow people to treat this like an array.
   F32& operator ()(S32 row, S32 col) { return m[idx(col,row)]; }
   F32 operator ()(S32 row, S32 col) const { return m[idx(col,row)]; }

   void dumpMatrix(const char *caption=NULL) const;

   // Math operator overloads
   //------------------------------------
   friend MatrixF operator * ( const MatrixF &m1, const MatrixF &m2 );
   MatrixF& operator *= ( const MatrixF &m );

   // Static identity matrix
   const static MatrixF Identity;
};


//--------------------------------------
// Inline Functions

inline MatrixF::MatrixF(bool _identity)
{
   if (_identity)
      identity();
}

inline MatrixF::MatrixF( const EulerF &e )
{
   set(e);
}

inline MatrixF::MatrixF( const EulerF &e, const Point3F& p )
{
   set(e,p);
}

inline MatrixF& MatrixF::set( const EulerF &e)
{
   m_matF_set_euler( e, *this );
   return (*this);
}


inline MatrixF& MatrixF::set( const EulerF &e, const Point3F& p)
{
   m_matF_set_euler_point( e, p, *this );
   return (*this);
}

inline MatrixF& MatrixF::setCrossProduct( const Point3F &p)
{
   m[1] = -(m[4] = p.z);
   m[8] = -(m[2] = p.y);
   m[6] = -(m[9] = p.x);
   m[0] = m[3] = m[5] = m[7] = m[10] = m[11] =
      m[12] = m[13] = m[14] = 0.0f;
   m[15] = 1;
   return (*this);
}

inline MatrixF& MatrixF::setTensorProduct( const Point3F &p, const Point3F &q)
{
   m[0] = p.x * q.x;
   m[1] = p.x * q.y;
   m[2] = p.x * q.z;
   m[4] = p.y * q.x;
   m[5] = p.y * q.y;
   m[6] = p.y * q.z;
   m[8] = p.z * q.x;
   m[9] = p.z * q.y;
   m[10] = p.z * q.z;
   m[3] = m[7] = m[11] = m[12] = m[13] = m[14] = 0.0f;
   m[15] = 1.0f;
   return (*this);
}

inline bool MatrixF::isIdentity() const
{
   return
   m[0]  == 1.0f &&
   m[1]  == 0.0f &&
   m[2]  == 0.0f &&
   m[3]  == 0.0f &&
   m[4]  == 0.0f &&
   m[5]  == 1.0f &&
   m[6]  == 0.0f &&
   m[7]  == 0.0f &&
   m[8]  == 0.0f &&
   m[9]  == 0.0f &&
   m[10] == 1.0f &&
   m[11] == 0.0f &&
   m[12] == 0.0f &&
   m[13] == 0.0f &&
   m[14] == 0.0f &&
   m[15] == 1.0f;
}

inline MatrixF& MatrixF::identity()
{
   m[0]  = 1.0f;
   m[1]  = 0.0f;
   m[2]  = 0.0f;
   m[3]  = 0.0f;
   m[4]  = 0.0f;
   m[5]  = 1.0f;
   m[6]  = 0.0f;
   m[7]  = 0.0f;
   m[8]  = 0.0f;
   m[9]  = 0.0f;
   m[10] = 1.0f;
   m[11] = 0.0f;
   m[12] = 0.0f;
   m[13] = 0.0f;
   m[14] = 0.0f;
   m[15] = 1.0f;
   return (*this);
}


inline MatrixF& MatrixF::inverse()
{
   m_matF_inverse(m);
   return (*this);
}

inline void MatrixF::invertTo( MatrixF *out )
{
   m_matF_invert_to(m,*out);
}

inline MatrixF& MatrixF::affineInverse()
{
//   AssertFatal(isAffine() == true, "Error, this matrix is not an affine transform");
   m_matF_affineInverse(m);
   return (*this);
}

inline MatrixF& MatrixF::transpose()
{
   m_matF_transpose(m);
   return (*this);
}

inline MatrixF& MatrixF::scale(const Point3F& p)
{
   m_matF_scale(m,p);
   return *this;
}

inline Point3F MatrixF::getScale() const
{
   Point3F scale;
   scale.x = mSqrt(m[0]*m[0] + m[4] * m[4] + m[8] * m[8]);
   scale.y = mSqrt(m[1]*m[1] + m[5] * m[5] + m[9] * m[9]);
   scale.z = mSqrt(m[2]*m[2] + m[6] * m[6] + m[10] * m[10]);
   return scale;
}

inline void MatrixF::normalize()
{
   m_matF_normalize(m);
}

inline MatrixF& MatrixF::mul( const MatrixF &a )
{  // M * a -> M
   AssertFatal(&a != this, "MatrixF::mul - a.mul(a) is invalid!");

   MatrixF tempThis(*this);
   m_matF_x_matF(tempThis, a, *this);
   return (*this);
}

inline MatrixF& MatrixF::mulL( const MatrixF &a )
{  // a * M -> M
   AssertFatal(&a != this, "MatrixF::mulL - a.mul(a) is invalid!");

   MatrixF tempThis(*this);
   m_matF_x_matF(a, tempThis, *this);
   return (*this);
}

inline MatrixF& MatrixF::mul( const MatrixF &a, const MatrixF &b )
{  // a * b -> M
   AssertFatal((&a != this) && (&b != this), "MatrixF::mul - a.mul(a, b) a.mul(b, a) a.mul(a, a) is invalid!");

   m_matF_x_matF(a, b, *this);
   return (*this);
}


inline MatrixF& MatrixF::mul(const F32 a)
{
   for (U32 i = 0; i < 16; i++)
      m[i] *= a;

   return *this;
}


inline MatrixF& MatrixF::mul(const MatrixF &a, const F32 b)
{
   *this = a;
   mul(b);

   return *this;
}

inline void MatrixF::mul( Point4F& p ) const
{
   Point4F temp;
   m_matF_x_point4F(*this, &p.x, &temp.x);
   p = temp;
}

inline void MatrixF::mulP( Point3F& p) const
{
   // M * p -> d
   Point3F d;
   m_matF_x_point3F(*this, &p.x, &d.x);
   p = d;
}

inline void MatrixF::mulP( const Point3F &p, Point3F *d) const
{
   // M * p -> d
   m_matF_x_point3F(*this, &p.x, &d->x);
}

inline void MatrixF::mulV( VectorF& v) const
{
   // M * v -> v
   VectorF temp;
   m_matF_x_vectorF(*this, &v.x, &temp.x);
   v = temp;
}

inline void MatrixF::mulV( const VectorF &v, Point3F *d) const
{
   // M * v -> d
   m_matF_x_vectorF(*this, &v.x, &d->x);
}

inline void MatrixF::mul(Box3F& b) const
{
   m_matF_x_box3F(*this, &b.minExtents.x, &b.maxExtents.x);
}

inline MatrixF& MatrixF::add( const MatrixF& a )
{
   for( U32 i = 0; i < 16; ++ i )
      m[ i ] += a.m[ i ];
      
   return *this;
}

inline void MatrixF::getColumn(S32 col, Point4F *cptr) const
{
   cptr->x = m[col];
   cptr->y = m[col+4];
   cptr->z = m[col+8];
   cptr->w = m[col+12];
}

inline void MatrixF::getColumn(S32 col, Point3F *cptr) const
{
   cptr->x = m[col];
   cptr->y = m[col+4];
   cptr->z = m[col+8];
}

inline void MatrixF::setColumn(S32 col, const Point4F &cptr)
{
   m[col]   = cptr.x;
   m[col+4] = cptr.y;
   m[col+8] = cptr.z;
   m[col+12]= cptr.w;
}

inline void MatrixF::setColumn(S32 col, const Point3F &cptr)
{
   m[col]   = cptr.x;
   m[col+4] = cptr.y;
   m[col+8] = cptr.z;
}


inline void MatrixF::getRow(S32 col, Point4F *cptr) const
{
   col *= 4;
   cptr->x = m[col++];
   cptr->y = m[col++];
   cptr->z = m[col++];
   cptr->w = m[col];
}

inline void MatrixF::getRow(S32 col, Point3F *cptr) const
{
   col *= 4;
   cptr->x = m[col++];
   cptr->y = m[col++];
   cptr->z = m[col];
}

inline void MatrixF::setRow(S32 col, const Point4F &cptr)
{
   col *= 4;
   m[col++] = cptr.x;
   m[col++] = cptr.y;
   m[col++] = cptr.z;
   m[col]   = cptr.w;
}

inline void MatrixF::setRow(S32 col, const Point3F &cptr)
{
   col *= 4;
   m[col++] = cptr.x;
   m[col++] = cptr.y;
   m[col]   = cptr.z;
}

inline Point3F MatrixF::getPosition() const
{
   return Point3F( m[3], m[3+4], m[3+8] );
}

inline void MatrixF::displace( const Point3F &delta )
{
   m[3]   += delta.x;
   m[3+4] += delta.y;
   m[3+8] += delta.z;
}

inline VectorF MatrixF::getForwardVector() const
{
   VectorF vec;
   getColumn( 1, &vec );
   return vec;
}

inline VectorF MatrixF::getRightVector() const
{
   VectorF vec;
   getColumn( 0, &vec );
   return vec;
}

inline VectorF MatrixF::getUpVector() const
{
   VectorF vec;
   getColumn( 2, &vec );
   return vec;
}

//------------------------------------
// Math operator overloads
//------------------------------------
inline MatrixF operator * ( const MatrixF &m1, const MatrixF &m2 )
{
   // temp = m1 * m2
   MatrixF temp;
   m_matF_x_matF(m1, m2, temp);
   return temp;
}

inline MatrixF& MatrixF::operator *= ( const MatrixF &m )
{
   MatrixF tempThis(*this);
   m_matF_x_matF(tempThis, m, *this);
   return (*this);
}

//------------------------------------
// Non-member methods
//------------------------------------

inline void mTransformPlane(const MatrixF& mat, const Point3F& scale, const PlaneF&  plane, PlaneF * result)
{
   m_matF_x_scale_x_planeF(mat, &scale.x, &plane.x, &result->x);
}

/// 4x4 Matrix Class
///
/// This runs at F64 precision.

class MatrixD
{
private:
   F64 m[16];     ///< Note: Torque uses row-major matrices

public:
   /// Create an uninitialized matrix.
   ///
   /// @param   identity    If true, initialize to the identity matrix.
   explicit MatrixD(bool identity=false);

   /// Create a matrix to rotate about origin by e.
   /// @see set
   explicit MatrixD( const EulerD &e);

   /// Create a matrix to rotate about p by e.
   /// @see set
   MatrixD( const EulerD &e, const Point3D& p);

   MatrixD( const MatrixF &mat);

   MatrixD(F64* openGLMatrix);

   /// Get the index in m to element in column i, row j
   ///
   /// This is necessary as we have m as a one dimensional array.
   ///
   /// @param   i   Column desired.
   /// @param   j   Row desired.
   static U32 idx(U32 i, U32 j) { return (i + j*4); }

   /// Initialize matrix to rotate about origin by e.
   MatrixD& set( const EulerD &e);

   /// Initialize matrix to rotate about p by e.
   MatrixD& set( const EulerD &e, const Point3D& p);

   MatrixD& set( const MatrixF &mat);

   bool equals(const MatrixD& mat);

   /// Initialize matrix with a cross product of p.
   MatrixD& setCrossProduct( const Point3D &p);

   /// Initialize matrix with a tensor product of p.
   MatrixD& setTensorProduct( const Point3D &p, const Point3D& q);

   operator F64*() { return (m); }              ///< Allow people to get at m.
   operator const F64*() const { return (F64*)(m); }  ///< Allow people to get at m.

   bool isAffine() const;                       ///< Check to see if this is an affine matrix.
   bool isIdentity() const;                     ///< Checks for identity matrix.

   /// Make this an identity matrix.
   MatrixD& identity();

   /// Invert m.
   MatrixD& inverse();
   /// Copy the inversion of this into out matrix.
   void invertTo( MatrixD *out );

   /// Take inverse of matrix assuming it is affine (rotation,
   /// scale, sheer, translation only).
   MatrixD& affineInverse();           

   /// Swap rows and columns.
   MatrixD& transpose();

   /// M * Matrix(p) -> M
   MatrixD& scale( const Point3D &s );            
   MatrixD& scale( F64 s ) { return scale( Point3D( s, s, s ) ); }

   /// Return scale assuming scale was applied via mat.scale(s).
   Point3D getScale() const;

   EulerD toEuler() const;

   /// Compute the inverse of the matrix.
   ///
   /// Computes inverse of full 4x4 matrix. Returns false and performs no inverse if
   /// the determinant is 0.
   ///
   /// Note: In most cases you want to use the normal inverse function.  This method should
   ///       be used if the matrix has something other than (0,0,0,1) in the bottom row.
   bool fullInverse();

   /// Swaps rows and columns into matrix.
   void transposeTo(F64 *matrix) const;

   /// Normalize the matrix.
   void normalize();

   /// Copy the requested column into a Point4D.
   void getColumn(S32 col, Point4D *cptr) const;
   Point4D getColumn4D(S32 col) const { Point4D ret; getColumn(col,&ret); return ret; }

   /// Copy the requested column into a Point3D.
   ///
   /// This drops the bottom-most row.
   void getColumn(S32 col, Point3D *cptr) const;
   Point3D getColumn3D(S32 col) const { Point3D ret; getColumn(col,&ret); return ret; }

   /// Set the specified column from a Point4D.
   void setColumn(S32 col, const Point4D& cptr);

   /// Set the specified column from a Point3D.
   ///
   /// The bottom-most row is not set.
   void setColumn(S32 col, const Point3D& cptr);

   /// Copy the specified row into a Point4D.
   void getRow(S32 row, Point4D *cptr) const;
   Point4D getRow4D(S32 row) const { Point4D ret; getRow(row,&ret); return ret; }

   /// Copy the specified row into a Point3D.
   ///
   /// Right-most item is dropped.
   void getRow(S32 row, Point3D *cptr) const;
   Point3D getRow3D(S32 row) const { Point3D ret; getRow(row,&ret); return ret; }

   /// Set the specified row from a Point4D.
   void setRow(S32 row, const Point4D& cptr);

   /// Set the specified row from a Point3D.
   ///
   /// The right-most item is not set.
   void setRow(S32 row, const Point3D& cptr);

   /// Get the position of the matrix.
   ///
   /// This is the 4th column of the matrix.
   Point3D getPosition() const;

   /// Set the position of the matrix.
   ///
   /// This is the 4th column of the matrix.
   void setPosition( const Point3D &pos ) { setColumn( 3, pos ); }

   /// Add the passed delta to the matrix position.
   void displace( const Point3D &delta );

   /// Get the x axis of the matrix.
   ///
   /// This is the 1st column of the matrix and is
   /// normally considered the right vector.
   VectorD getRightVector() const;

   /// Get the y axis of the matrix.
   ///
   /// This is the 2nd column of the matrix and is
   /// normally considered the forward vector.   
   VectorD getForwardVector() const;   

   /// Get the z axis of the matrix.
   ///
   /// This is the 3rd column of the matrix and is
   /// normally considered the up vector.   
   VectorD getUpVector() const;

   MatrixD&  mul(const MatrixD &a);                    ///< M * a -> M
   MatrixD&  mulL(const MatrixD &a);                   ///< a * M -> M
   MatrixD&  mul(const MatrixD &a, const MatrixD &b);  ///< a * b -> M

   // Scalar multiplies
   MatrixD&  mul(const F64 a);                         ///< M * a -> M
   MatrixD&  mul(const MatrixD &a, const F64 b);       ///< a * b -> M


   void mul( Point4D& p ) const;                       ///< M * p -> p (full [4x4] * [1x4])
   void mulP( Point3D& p ) const;                      ///< M * p -> p (assume w = 1.0f)
   void mulP( const Point3D &p, Point3D *d) const;     ///< M * p -> d (assume w = 1.0f)
   void mulV( VectorD& p ) const;                      ///< M * v -> v (assume w = 0.0f)
   void mulV( const VectorD &p, Point3D *d) const;     ///< M * v -> d (assume w = 0.0f)

   void mul(Box3D& b) const;                           ///< Axial box -> Axial Box
   
   MatrixD& add( const MatrixD& m );

   /// Convenience function to allow people to treat this like an array.
   F64& operator ()(S32 row, S32 col) { return m[idx(col,row)]; }
   F64 operator ()(S32 row, S32 col) const { return m[idx(col,row)]; }
   
   bool operator==(const MatrixD& a) const
   {
      return (a[0] == m[0] && a[1] == m[1] && a[2] == m[2] && a[3] == m[3]
         && a[4] == m[4] && a[5] == m[5] && a[6] == m[6] && a[7] == m[7]
         && a[8] == m[8] && a[9] == m[9] && a[10] == m[10] && a[11] == m[11]
         && a[12] == m[12] && a[13] == m[13] && a[14] == m[14] && a[15] == m[15]);
   }

   bool operator!=(const MatrixD& a) const
   {
      return (a[0] != m[0] || a[1] != m[1] || a[2] != m[2] || a[3] != m[3]
         || a[4] != m[4] || a[5] != m[5] || a[6] != m[6] || a[7] != m[7]
         || a[8] != m[8] || a[9] != m[9] || a[10] != m[10] || a[11] != m[11]
         || a[12] != m[12] || a[13] != m[13] || a[14] != m[14] || a[15] != m[15]);
   }
   
   void dumpMatrix(const char *caption=NULL) const;

   // Math operator overloads
   //------------------------------------
   friend MatrixD operator * ( const MatrixD &m1, const MatrixD &m2 );
   MatrixD& operator *= ( const MatrixD &m );

   // Static identity matrix
   const static MatrixD Identity;
};


//--------------------------------------
// Inline Functions

inline MatrixD::MatrixD(bool _identity)
{
   if (_identity)
      identity();
}

inline MatrixD::MatrixD( const EulerD &e )
{
   set(e);
}

inline MatrixD::MatrixD( const MatrixF &mat)
{
	set(mat);
}

inline MatrixD::MatrixD( const EulerD &e, const Point3D& p )
{
   set(e,p);
}

inline MatrixD::MatrixD(F64* openGLMatrix)
{
   m[0] = openGLMatrix[0];
   m[1] = openGLMatrix[4];
   m[2] = openGLMatrix[8];
   m[3] = openGLMatrix[12];
   m[4] = openGLMatrix[1];
   m[5] = openGLMatrix[5];
   m[6] = openGLMatrix[9];
   m[7] = openGLMatrix[13];
   m[8] = openGLMatrix[2];
   m[9] = openGLMatrix[6];
   m[10] = openGLMatrix[10];
   m[11] = openGLMatrix[14];
   m[12] = openGLMatrix[3];
   m[13] = openGLMatrix[7];
   m[14] = openGLMatrix[11];
   m[15] = openGLMatrix[15];
}

inline MatrixD& MatrixD::set( const EulerD &e)
{
   m_matD_set_euler( e, *this );
   return (*this);
}


inline MatrixD& MatrixD::set( const EulerD &e, const Point3D& p)
{
   m_matD_set_euler_point( e, p, *this );
   return (*this);
}

inline MatrixD& MatrixD::set( const MatrixF &mat)
{
   m[0] = mat[0];
   m[1] = mat[1];
   m[2] = mat[2];
   m[3] = mat[3];
   m[4] = mat[4];
   m[5] = mat[5];
   m[6] = mat[6];
   m[7] = mat[7];
   m[8] = mat[8];
   m[9] = mat[9];
   m[10] = mat[10];
   m[11] = mat[11];
   m[12] = mat[12];
   m[13] = mat[13];
   m[14] = mat[14];
   m[15] = mat[15];

   return (*this);
}

inline bool MatrixD::equals(const MatrixD& mat)
{
   return (m[0] == mat[0] && m[1] == mat[1] && m[2] == mat[2] && m[3] == mat[3] &&
      m[4] == mat[4] && m[5] == mat[5] && m[6] == mat[6] && m[7] == mat[7] &&
      m[8] == mat[8] && m[9] == mat[9] && m[10] == mat[10] && m[11] == mat[11] &&
      m[12] == mat[12] && m[13] == mat[13] && m[14] == mat[14] && m[15] == mat[15]);
}

inline MatrixD& MatrixD::setCrossProduct( const Point3D &p)
{
   m[1] = -(m[4] = p.z);
   m[8] = -(m[2] = p.y);
   m[6] = -(m[9] = p.x);
   m[0] = m[3] = m[5] = m[7] = m[10] = m[11] =
      m[12] = m[13] = m[14] = 0.0;
   m[15] = 1;
   return (*this);
}

inline MatrixD& MatrixD::setTensorProduct( const Point3D &p, const Point3D &q)
{
   m[0] = p.x * q.x;
   m[1] = p.x * q.y;
   m[2] = p.x * q.z;
   m[4] = p.y * q.x;
   m[5] = p.y * q.y;
   m[6] = p.y * q.z;
   m[8] = p.z * q.x;
   m[9] = p.z * q.y;
   m[10] = p.z * q.z;
   m[3] = m[7] = m[11] = m[12] = m[13] = m[14] = 0.0;
   m[15] = 1.0;
   return (*this);
}

inline bool MatrixD::isIdentity() const
{
   return
   m[0]  == 1.0 &&
   m[1]  == 0.0 &&
   m[2]  == 0.0 &&
   m[3]  == 0.0 &&
   m[4]  == 0.0 &&
   m[5]  == 1.0 &&
   m[6]  == 0.0 &&
   m[7]  == 0.0 &&
   m[8]  == 0.0 &&
   m[9]  == 0.0 &&
   m[10] == 1.0 &&
   m[11] == 0.0 &&
   m[12] == 0.0 &&
   m[13] == 0.0 &&
   m[14] == 0.0 &&
   m[15] == 1.0;
}

inline MatrixD& MatrixD::identity()
{
   m[0]  = 1.0;
   m[1]  = 0.0;
   m[2]  = 0.0;
   m[3]  = 0.0;
   m[4]  = 0.0;
   m[5]  = 1.0;
   m[6]  = 0.0;
   m[7]  = 0.0;
   m[8]  = 0.0;
   m[9]  = 0.0;
   m[10] = 1.0;
   m[11] = 0.0;
   m[12] = 0.0;
   m[13] = 0.0;
   m[14] = 0.0;
   m[15] = 1.0;
   return (*this);
}


inline MatrixD& MatrixD::inverse()
{
   m_matD_inverse(m);
   return (*this);
}

inline void MatrixD::invertTo( MatrixD *out )
{
   m_matD_invert_to(m,*out);
}

inline MatrixD& MatrixD::affineInverse()
{
//   AssertFatal(isAffine() == true, "Error, this matrix is not an affine transform");
   m_matD_affineInverse(m);
   return (*this);
}

inline MatrixD& MatrixD::transpose()
{
   m_matD_transpose(m);
   return (*this);
}

inline MatrixD& MatrixD::scale(const Point3D& p)
{
   m_matD_scale(m,p);
   return *this;
}

inline Point3D MatrixD::getScale() const
{
   Point3D scale;
   scale.x = mSqrt(m[0]*m[0] + m[4] * m[4] + m[8] * m[8]);
   scale.y = mSqrt(m[1]*m[1] + m[5] * m[5] + m[9] * m[9]);
   scale.z = mSqrt(m[2]*m[2] + m[6] * m[6] + m[10] * m[10]);
   return scale;
}

inline void MatrixD::normalize()
{
   m_matD_normalize(m);
}

inline MatrixD& MatrixD::mul( const MatrixD &a )
{  // M * a -> M
   AssertFatal(&a != this, "MatrixD::mul - a.mul(a) is invalid!");

   MatrixD tempThis(*this);
   m_matD_x_matD(tempThis, a, *this);
   return (*this);
}

inline MatrixD& MatrixD::mulL( const MatrixD &a )
{  // a * M -> M
   AssertFatal(&a != this, "MatrixD::mulL - a.mul(a) is invalid!");

   MatrixD tempThis(*this);
   m_matD_x_matD(a, tempThis, *this);
   return (*this);
}

inline MatrixD& MatrixD::mul( const MatrixD &a, const MatrixD &b )
{  // a * b -> M
   AssertFatal((&a != this) && (&b != this), "MatrixD::mul - a.mul(a, b) a.mul(b, a) a.mul(a, a) is invalid!");

   m_matD_x_matD(a, b, *this);
   return (*this);
}


inline MatrixD& MatrixD::mul(const F64 a)
{
   for (U32 i = 0; i < 16; i++)
      m[i] *= a;

   return *this;
}


inline MatrixD& MatrixD::mul(const MatrixD &a, const F64 b)
{
   *this = a;
   mul(b);

   return *this;
}

inline void MatrixD::mul( Point4D& p ) const
{
   Point4D temp;
   m_matD_x_point4D(*this, &p.x, &temp.x);
   p = temp;
}

inline void MatrixD::mulP( Point3D& p) const
{
   // M * p -> d
   Point3D d;
   m_matD_x_point3D(*this, &p.x, &d.x);
   p = d;
}

inline void MatrixD::mulP( const Point3D &p, Point3D *d) const
{
   // M * p -> d
   m_matD_x_point3D(*this, &p.x, &d->x);
}

inline void MatrixD::mulV( VectorD& v) const
{
   // M * v -> v
   VectorD temp;
   m_matD_x_vectorD(*this, &v.x, &temp.x);
   v = temp;
}

inline void MatrixD::mulV( const VectorD &v, Point3D *d) const
{
   // M * v -> d
   m_matD_x_vectorD(*this, &v.x, &d->x);
}

inline void MatrixD::mul(Box3D& b) const
{
   m_matD_x_box3D(*this, &b.minExtents.x, &b.maxExtents.x);
}

inline MatrixD& MatrixD::add( const MatrixD& a )
{
   for( U32 i = 0; i < 16; ++ i )
      m[ i ] += a.m[ i ];
      
   return *this;
}

inline void MatrixD::getColumn(S32 col, Point4D *cptr) const
{
   cptr->x = m[col];
   cptr->y = m[col+4];
   cptr->z = m[col+8];
   cptr->w = m[col+12];
}

inline void MatrixD::getColumn(S32 col, Point3D *cptr) const
{
   cptr->x = m[col];
   cptr->y = m[col+4];
   cptr->z = m[col+8];
}

inline void MatrixD::setColumn(S32 col, const Point4D &cptr)
{
   m[col]   = cptr.x;
   m[col+4] = cptr.y;
   m[col+8] = cptr.z;
   m[col+12]= cptr.w;
}

inline void MatrixD::setColumn(S32 col, const Point3D &cptr)
{
   m[col]   = cptr.x;
   m[col+4] = cptr.y;
   m[col+8] = cptr.z;
}


inline void MatrixD::getRow(S32 col, Point4D *cptr) const
{
   col *= 4;
   cptr->x = m[col++];
   cptr->y = m[col++];
   cptr->z = m[col++];
   cptr->w = m[col];
}

inline void MatrixD::getRow(S32 col, Point3D *cptr) const
{
   col *= 4;
   cptr->x = m[col++];
   cptr->y = m[col++];
   cptr->z = m[col];
}

inline void MatrixD::setRow(S32 col, const Point4D &cptr)
{
   col *= 4;
   m[col++] = cptr.x;
   m[col++] = cptr.y;
   m[col++] = cptr.z;
   m[col]   = cptr.w;
}

inline void MatrixD::setRow(S32 col, const Point3D &cptr)
{
   col *= 4;
   m[col++] = cptr.x;
   m[col++] = cptr.y;
   m[col]   = cptr.z;
}

inline Point3D MatrixD::getPosition() const
{
   return Point3D( m[3], m[3+4], m[3+8] );
}

inline void MatrixD::displace( const Point3D &delta )
{
   m[3]   += delta.x;
   m[3+4] += delta.y;
   m[3+8] += delta.z;
}

inline VectorD MatrixD::getForwardVector() const
{
   VectorD vec;
   getColumn( 1, &vec );
   return vec;
}

inline VectorD MatrixD::getRightVector() const
{
   VectorD vec;
   getColumn( 0, &vec );
   return vec;
}

inline VectorD MatrixD::getUpVector() const
{
   VectorD vec;
   getColumn( 2, &vec );
   return vec;
}

//------------------------------------
// Math operator overloads
//------------------------------------
inline MatrixD operator * ( const MatrixD &m1, const MatrixD &m2 )
{
   // temp = m1 * m2
   MatrixD temp;
   m_matD_x_matD(m1, m2, temp);
   return temp;
}

inline MatrixD& MatrixD::operator *= ( const MatrixD &m )
{
   MatrixD tempThis(*this);
   m_matD_x_matD(tempThis, m, *this);
   return (*this);
}

//------------------------------------
// Non-member methods
//------------------------------------

inline void mTransformPlane(const MatrixD& mat, const Point3D& scale, const PlaneD&  plane, PlaneD * result)
{
   m_matD_x_scale_x_planeD(mat, &scale.x, &plane.x, &result->x);
}

#endif //_MMATRIX_H_
