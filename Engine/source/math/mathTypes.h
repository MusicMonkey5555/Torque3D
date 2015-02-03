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

#ifndef _MATHTYPES_H_
#define _MATHTYPES_H_

#ifndef _DYNAMIC_CONSOLETYPES_H_
   #include "console/dynamicTypes.h"
#endif


void RegisterMathFunctions(void);


class Point2I;
class Point2F;
class Point2D;
class Point3I;
class Point3F;
class Point3D;
class Point4F;
class Point4D;
class RectI;
class RectF;
class RectD;
class MatrixF;
class MatrixD;
class Box3F;
class Box3D;
class EaseF;
class AngAxisF;
class AngAxisD;
class TransformF;


DECLARE_SCOPE( MathTypes );


DECLARE_STRUCT( Point2I );
DECLARE_STRUCT( Point2F );
DECLARE_STRUCT( Point2D );
DECLARE_STRUCT( Point3I );
DECLARE_STRUCT( Point3F );
DECLARE_STRUCT( Point3D );
DECLARE_STRUCT( Point4F );
DECLARE_STRUCT( Point4D );
DECLARE_STRUCT( RectI );
DECLARE_STRUCT( RectF );
DECLARE_STRUCT( RectD );
DECLARE_STRUCT( MatrixF );
DECLARE_STRUCT( MatrixD );
DECLARE_STRUCT( AngAxisF );
DECLARE_STRUCT( AngAxisD );
DECLARE_STRUCT( TransformF );
DECLARE_STRUCT( Box3F );
DECLARE_STRUCT( Box3D );
DECLARE_STRUCT( EaseF );


// Legacy console types.
DefineConsoleType( TypePoint2I, Point2I )
DefineConsoleType( TypePoint2F, Point2F )
DefineConsoleType( TypePoint2D, Point2D )
DefineConsoleType( TypePoint3I, Point3I )
DefineConsoleType( TypePoint3F, Point3F )
DefineConsoleType( TypePoint3D, Point3D )
DefineConsoleType( TypePoint4F, Point4F )
DefineConsoleType( TypePoint4D, Point4D )
DefineConsoleType( TypeRectI, RectI )
DefineConsoleType( TypeRectF, RectF )
DefineConsoleType( TypeRectD, RectD )
DefineConsoleType( TypeMatrixF, MatrixF )
DefineConsoleType( TypeMatrixD, MatrixD )
DefineConsoleType( TypeMatrixPosition, MatrixF)
DefineConsoleType( TypeMatrixRotation, MatrixF )
DefineConsoleType( TypeAngAxisF, AngAxisF )
DefineConsoleType( TypeAngAxisD, AngAxisD )
DefineConsoleType( TypeTransformF, TransformF )
DefineConsoleType( TypeBox3F, Box3F )
DefineConsoleType( TypeBox3D, Box3D )
DefineConsoleType( TypeEaseF, EaseF )


#endif
