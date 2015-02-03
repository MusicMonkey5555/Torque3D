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

#ifndef _MQUAT_H_
#define _MQUAT_H_

#ifndef _MPOINT3_H_
#include "math/mPoint3.h"
#endif

class MatrixF;
class AngAxisF;
class MatrixD;
class AngAxisD;

//----------------------------------------------------------------------------
// unit quaternion class:

class QuatF
{
   //-------------------------------------- Public static constants
public:
   const static QuatF Identity;

  public:
   F32  x,y,z,w;

   QuatF() {} // no init constructor
   QuatF( F32 _x, F32 _y, F32 _z, F32 w );
   QuatF( const Point3F &axis, F32 angle );
   QuatF( const MatrixF & m );
   QuatF( const AngAxisF & a );
   QuatF( const EulerF & e );

   QuatF& set( F32 _x, F32 _y, F32 _z, F32 _w );
   QuatF& set( const Point3F &axis, F32 angle );
   QuatF& set( const MatrixF & m );
   QuatF& set( const AngAxisF & a );
   QuatF& set( const EulerF & e );

   S32 operator ==( const QuatF & c ) const;
   S32 operator !=( const QuatF & c ) const;
   QuatF& operator *=( const QuatF & c );
   QuatF& operator /=( const QuatF & c );
   QuatF& operator +=( const QuatF & c );
   QuatF& operator -=( const QuatF & c );
   QuatF& operator *=( F32 a );
   QuatF& operator /=( F32 a );

   QuatF operator-( const QuatF &c ) const;
   QuatF operator*( F32 a ) const;

   QuatF& square();
   QuatF& neg();
   F32  dot( const QuatF &q ) const;

   MatrixF* setMatrix( MatrixF * mat ) const;
   QuatF& normalize();
   QuatF& inverse();
   QuatF& identity();
   S32    isIdentity() const;
   QuatF& slerp( const QuatF & q, F32 t );
   QuatF& extrapolate( const QuatF & q1, const QuatF & q2, F32 t );
   QuatF& interpolate( const QuatF & q1, const QuatF & q2, F32 t );
   F32  angleBetween( const QuatF & q );

   Point3F& mulP(const Point3F& a, Point3F* b);   // r = p * this
   QuatF& mul(const QuatF& a, const QuatF& b);    // This = a * b

   // Vectors passed in must be normalized
   QuatF& shortestArc( const VectorF &normalizedA, const VectorF &normalizedB );
};

// a couple simple utility methods
inline F32 QuatIsEqual(F32 a,F32 b,F32 epsilon = 0.0001f) { return mFabs(a-b) < epsilon; }
inline F32 QuatIsZero(F32 a,F32 epsilon = 0.0001f) { return mFabs(a) < epsilon; }

//----------------------------------------------------------------------------
// quaternion implementation:

inline QuatF::QuatF( F32 _x, F32 _y, F32 _z, F32 _w )
{
   set( _x, _y, _z, _w );
}

inline QuatF::QuatF( const Point3F &axis, F32 angle )
{
   set( axis, angle );
}

inline QuatF::QuatF( const AngAxisF & a )
{
   set( a );
}

inline QuatF::QuatF( const EulerF & e )
{
   set( e );
}

inline QuatF::QuatF( const MatrixF & m )
{
   set( m );
}

inline QuatF& QuatF::set( F32 _x, F32 _y, F32 _z, F32 _w )
{
   x = _x;
   y = _y;
   z = _z;
   w = _w;
   return *this;
}

inline int QuatF::operator ==( const QuatF & c ) const
{
   QuatF a = *this;
   QuatF b = c;
   a.normalize();
   b.normalize();
   b.inverse();
   a *= b;
   return a.isIdentity();
}

inline int QuatF::isIdentity() const
{
   return QuatIsZero( x ) && QuatIsZero( y ) && QuatIsZero( z );
}

inline QuatF& QuatF::identity()
{
   x = 0.0f;
   y = 0.0f;
   z = 0.0f;
   w = 1.0f;
   return *this;
}

inline int QuatF::operator !=( const QuatF & c ) const
{
   return !operator==( c );
}

inline QuatF& QuatF::operator +=( const QuatF & c )
{
   x += c.x;
   y += c.y;
   z += c.z;
   w += c.w;
   return *this;
}

inline QuatF& QuatF::operator -=( const QuatF & c )
{
   x -= c.x;
   y -= c.y;
   z -= c.z;
   w -= c.w;
   return *this;
}

inline QuatF& QuatF::operator *=( F32 a )
{
   x *= a;
   y *= a;
   z *= a;
   w *= a;
   return *this;
}

inline QuatF& QuatF::operator /=( F32 a )
{
   x /= a;
   y /= a;
   z /= a;
   w /= a;
   return *this;
}

inline QuatF QuatF::operator -( const QuatF &c ) const
{
   return QuatF( x - c.x,
                 y - c.y,
                 z - c.z,
                 w - c.w );
}

inline QuatF QuatF::operator *( F32 a ) const
{
   return QuatF( x * a,
                 y * a,
                 z * a,
                 w * a );
}

inline QuatF& QuatF::neg()
{
   x = -x;
   y = -y;
   z = -z;
   w = -w;
   return *this;
}

inline F32 QuatF::dot( const QuatF &q ) const
{
   return (w*q.w + x*q.x + y*q.y + z*q.z);
}

inline F32 QuatF::angleBetween( const QuatF & q )
{
   // angle between to quaternions
   return mAcos(x * q.x + y * q.y + z * q.z + w * q.w);
}

class QuatD
{
   //-------------------------------------- Public static constants
public:
   const static QuatD Identity;

  public:
   F64  x,y,z,w;

   QuatD() {} // no init constructor
   QuatD( F64 _x, F64 _y, F64 _z, F64 w );
   QuatD( const Point3D &axis, F64 angle );
   QuatD( const MatrixD & m );
   QuatD( const AngAxisD & a );
   QuatD( const EulerD & e );

   QuatD& set( F64 _x, F64 _y, F64 _z, F64 _w );
   QuatD& set( const Point3D &axis, F64 angle );
   QuatD& set( const MatrixD & m );
   QuatD& set( const AngAxisD & a );
   QuatD& set( const EulerD & e );

   int operator ==( const QuatD & c ) const;
   int operator !=( const QuatD & c ) const;
   QuatD& operator *=( const QuatD & c );
   QuatD& operator /=( const QuatD & c );
   QuatD& operator +=( const QuatD & c );
   QuatD& operator -=( const QuatD & c );
   QuatD& operator *=( F64 a );
   QuatD& operator /=( F64 a );

   QuatD operator-( const QuatD &c ) const;
   QuatD operator*( F64 a ) const;

   QuatD& square();
   QuatD& neg();
   F64  dot( const QuatD &q ) const;

   MatrixD* setMatrix( MatrixD * mat ) const;
   QuatD& normalize();
   QuatD& inverse();
   QuatD& identity();
   int    isIdentity() const;
   QuatD& slerp( const QuatD & q, F64 t );
   QuatD& extrapolate( const QuatD & q1, const QuatD & q2, F64 t );
   QuatD& interpolate( const QuatD & q1, const QuatD & q2, F64 t );
   F64  angleBetween( const QuatD & q );

   Point3D& mulP(const Point3D& a, Point3D* b);   // r = p * this
   QuatD& mul(const QuatD& a, const QuatD& b);    // This = a * b

   // Vectors passed in must be normalized
   QuatD& shortestArc( const VectorD &normalizedA, const VectorD &normalizedB );
};

// a couple simple utility methods
inline F64 QuatIsEqual(F64 a,F64 b,F64 epsilon = 0.0001f) { return mFabs(a-b) < epsilon; }
inline F64 QuatIsZero(F64 a,F64 epsilon = 0.0001f) { return mFabs(a) < epsilon; }

//----------------------------------------------------------------------------
// quaternion implementation:

inline QuatD::QuatD( F64 _x, F64 _y, F64 _z, F64 _w )
{
   set( _x, _y, _z, _w );
}

inline QuatD::QuatD( const Point3D &axis, F64 angle )
{
   set( axis, angle );
}

inline QuatD::QuatD( const AngAxisD & a )
{
   set( a );
}

inline QuatD::QuatD( const EulerD & e )
{
   set( e );
}

inline QuatD::QuatD( const MatrixD & m )
{
   set( m );
}

inline QuatD& QuatD::set( F64 _x, F64 _y, F64 _z, F64 _w )
{
   x = _x;
   y = _y;
   z = _z;
   w = _w;
   return *this;
}

inline int QuatD::operator ==( const QuatD & c ) const
{
   QuatD a = *this;
   QuatD b = c;
   a.normalize();
   b.normalize();
   b.inverse();
   a *= b;
   return a.isIdentity();
}

inline int QuatD::isIdentity() const
{
   return QuatIsZero( x ) && QuatIsZero( y ) && QuatIsZero( z );
}

inline QuatD& QuatD::identity()
{
   x = 0.0f;
   y = 0.0f;
   z = 0.0f;
   w = 1.0f;
   return *this;
}

inline int QuatD::operator !=( const QuatD & c ) const
{
   return !operator==( c );
}

inline QuatD& QuatD::operator +=( const QuatD & c )
{
   x += c.x;
   y += c.y;
   z += c.z;
   w += c.w;
   return *this;
}

inline QuatD& QuatD::operator -=( const QuatD & c )
{
   x -= c.x;
   y -= c.y;
   z -= c.z;
   w -= c.w;
   return *this;
}

inline QuatD& QuatD::operator *=( F64 a )
{
   x *= a;
   y *= a;
   z *= a;
   w *= a;
   return *this;
}

inline QuatD& QuatD::operator /=( F64 a )
{
   x /= a;
   y /= a;
   z /= a;
   w /= a;
   return *this;
}

inline QuatD QuatD::operator -( const QuatD &c ) const
{
   return QuatD( x - c.x,
                 y - c.y,
                 z - c.z,
                 w - c.w );
}

inline QuatD QuatD::operator *( F64 a ) const
{
   return QuatD( x * a,
                 y * a,
                 z * a,
                 w * a );
}

inline QuatD& QuatD::neg()
{
   x = -x;
   y = -y;
   z = -z;
   w = -w;
   return *this;
}

inline F64 QuatD::dot( const QuatD &q ) const
{
   return (w*q.w + x*q.x + y*q.y + z*q.z);
}

inline F64 QuatD::angleBetween( const QuatD & q )
{
   // angle between to quaternions
   return mAcos(x * q.x + y * q.y + z * q.z + w * q.w);
}

#endif // _MQUAT_H_
