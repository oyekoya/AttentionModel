/*
 *      utilmath.h
 *
 *      Copyright 2010 Wole Oyekoya <w.oyekoya@cs.ucl.ac.uk>
 *		University College London
 */
#ifndef _UTIL_MATH_H_
#define _UTIL_MATH_H_

#include <math.h>
#include <stdio.h>
#include <float.h>

#ifndef M_PI
#define M_PI 3.1415926532
#endif

#define RAD_TO_DEG(X) (X / M_PI * 180)
#define DEG_TO_RAD(X) (X / 180 * M_PI)


//////////////////////////////////////////////////////////////////////////////
//
// vector
//
//////////////////////////////////////////////////////////////////////////////
/// math helper class for vectors
class CVec3
{
public:
	// Data
	float x, y, z;

	// Ctors
	CVec3( float InX, float InY, float InZ ) : x( InX ), y( InY ), z( InZ )
	{
	}
	CVec3( ) : x(0), y(0), z(0)
	{
	}

	// Operator Overloads
	inline bool operator== (const CVec3& V2) const 
	{
		return (x == V2.x && y == V2.y && z == V2.z);
	}

	inline CVec3 operator+ (const CVec3& V2) const 
	{
		return CVec3( x + V2.x,  y + V2.y,  z + V2.z);
	}
	inline CVec3 operator- (const CVec3& V2) const
	{
		return CVec3( x - V2.x,  y - V2.y,  z - V2.z);
	}
	inline CVec3 operator- ( ) const
	{
		return CVec3(-x, -y, -z);
	}

	inline CVec3 operator/ (float S ) const
	{
		float fInv = 1.0f / S;
		return CVec3 (x * fInv , y * fInv, z * fInv);
	}
	inline CVec3 operator/ (const CVec3& V2) const
	{
		return CVec3 (x / V2.x,  y / V2.y,  z / V2.z);
	}
	inline CVec3 operator* (const CVec3& V2) const
	{
		return CVec3 (x * V2.x,  y * V2.y,  z * V2.z);
	}
	inline CVec3 operator* (float S) const
	{
		return CVec3 (x * S,  y * S,  z * S);
	}

	inline void operator+= ( const CVec3& V2 )
	{
		x += V2.x;
		y += V2.y;
		z += V2.z;
	}
	inline void operator-= ( const CVec3& V2 )
	{
		x -= V2.x;
		y -= V2.y;
		z -= V2.z;
	}

	inline float operator[] ( int i )
	{
		if ( i == 0 ) return x;
		else if ( i == 1 ) return y;
		else return z;
	}

	// Functions
	inline float Dot( const CVec3 &V1 ) const
	{
		return V1.x*x + V1.y*y + V1.z*z;
	}

	inline CVec3 Cross( const CVec3 &V2 ) const
	{
		return CVec3(
			y * V2.z  -  z * V2.y,
			z * V2.x  -  x * V2.z,
			x * V2.y  -  y * V2.x 	);
	}

	// Return vector rotated by the 3x3 portion of matrix m
	CVec3 RotByMatrix( const float m[16] ) const
	{
		return CVec3( 
			x*m[0] + y*m[4] + z*m[8],
			x*m[1] + y*m[5] + z*m[9],
			x*m[2] + y*m[6] + z*m[10] );
	}

	// These require math.h for the sqrtf function
	float Magnitude( ) const
	{
		return sqrtf( x*x + y*y + z*z );
	}

	float Distance( const CVec3 &V1 ) const
	{
		return ( *this - V1 ).Magnitude();	
	}

	inline float Normalize(void)
	{
		float mag = sqrtf( x*x + y*y + z*z );
		float oomag;

		if (mag > 0.0000001f)
		{
			oomag = 1.f/mag;
			x *= oomag;
			y *= oomag;
			z *= oomag;
		}
		else
		{
			x = 0.f;
			y = 0.f;
			z = 0.f;
			mag = 0;
		}
		return (mag);
	}
	
	/*inline void &Normalize()
	{
		float fMag = ( x*x + y*y + z*z );
		if (fMag == 0) {return;}

		float fMult = 1.0f/sqrtf(fMag);            
		x *= fMult;
		y *= fMult;
		z *= fMult;
		return;
	}*/

/*	float enclosedAngle(const CVec3 &V1 ) const
	{
		float returnValue = Dot(V1);
        returnValue /= (Magnitude() * V1.Magnitude());
        returnValue = acos(returnValue);
		return returnValue;
	}*///incorrect?

	inline float angle_between(const CVec3& a, const CVec3& b)
	{
		CVec3 an = a;
		CVec3 bn = b;
		an.Normalize();
		bn.Normalize();
		float cosine = an.Dot(bn);
		float angle = (cosine >= 1.0f) ? 0.0f :
					(cosine <= -1.0f) ? M_PI :
					(float)acos(cosine);
		return angle;
	}

	/*CVec3 enclosedAngle(const CVec3 &v2, float *angle) 
	{ 
		float ang; 
		// turn vectors into unit vectors 
		CVec3 v1 = v2;
		v1.Normalize();
		Normalize(); 
		ang = acos( v1.Dot(*this) );
		//printf("%.3f",ang);
		// if no noticable rotation is available return zero rotation
		// this way we avoid Cross product artifacts 
		if( abs(ang) < 0.0001 ) {
			*angle=0;
			return CVec3(0,0,1); 
		}
		// in this case there are 2 lines on the same axis 
		//if((abs(ang)-M_PI) < 0.001) { 
		//	v1.RotX( 0.5f ); 
			//printf(" should you be here");
			// there are an infinite number of normals 
			// in this case. Anyone of these normals will be 
			// a valid rotation (180 degrees). so I rotate the curr axis by 0.5 radians this way we get one of these normals 
		//}
		//printf("\n");
		*angle = ang;
 		CVec3 axis = v1.Cross(*this);
		return axis; 
	}*/

/*	CVec3 enclosedAngle(const CVec3 &v2, float *angle) 
	{ 
		float ang; 
		CVec3 v1 = v2;
		v1.Normalize();
		Normalize();
		CVec3 axis = v1.Cross(*this);
		ang = atan2(axis.Magnitude(),Dot(v1));
		ang = axis.Dot(CVec3(1.f,0.f,0.f)) < 0.f ? -ang : ang;
		*angle = ang;
		return axis;
	}*/


	inline void RotX(float q)
	{
		y = y*cos(q) - z*sin(q);
		z = y*sin(q) + z*cos(q);
		x = x;
	}

	inline void RotY(float q)
	{
		z = z*cos(q) - x*sin(q);
		x = z*sin(q) + x*cos(q);
		y = y;
	}

	inline void RotZ(float q)
	{
		x = x*cos(q) - y*sin(q);
		y = x*sin(q) + y*cos(q);
		z = z;	
	}

};

//////////////////////////////////////////////////////////////////////////////
//
// quaternion
//
//////////////////////////////////////////////////////////////////////////////

const float TO_HALF_RAD = 3.14159265f / 360.0f;

/// math helper class for quaternions
class CQuat
{
public:
	float x,y,z,w;

	CQuat( ) : x(0), y(0), z(0), w(1)
	{
	}

	CQuat( float fx, float fy, float fz, float fw ) : x(fx), y(fy), z(fz), w(fw)
	{
	}

	// This just took four floats initially to avoid dependence on the vector class
	// but I decided avoiding confusion with the value setting constructor was more important
	CQuat( float Angle, const CVec3& Axis )
	{
		SetAxis( Angle, Axis.x, Axis.y, Axis.z );
	}

	/// No rotation
	void Reset( )
	{
		x = 0;
		y = 0;
		z = 0;
		w = 1;
	}
	/// True if identity (x == 0.0f && y == 0.0f && z == 0.0f && w==1.0f)
	int IsIdentity( ) const
	{
		return (x == 0.0f && y == 0.0f && z == 0.0f && w==1.0f);
	}

	/// Set Quat from axis-angle
	void SetAxis( float degrees, float fX, float fY, float fZ )
	{
		float HalfAngle = degrees * TO_HALF_RAD; // Get half angle in radians from angle in degrees
		float sinA = (float)sin( HalfAngle ) ;
		w = (float)cos( HalfAngle );
		x = fX * sinA;
		y = fY * sinA;
		z = fZ * sinA;
	}
	// ------------------------------------
	/// Simple Euler Angle to Quaternion conversion (this could be made faster)
	// ------------------------------------
	void FromEuler( float rx, float ry, float rz )
	{
		CQuat qx(-rx, CVec3( 1, 0, 0 ) );
		CQuat qy(-ry, CVec3( 0, 1, 0 ) );
		CQuat qz(-rz, CVec3( 0, 0, 1 ) );
		qz = qy * qz;
		*this = qx * qz;
	}
	// ------------------------------------
	/// Set a 4x4 matrix with the rotation of this Quaternion
	// ------------------------------------
	void inline ToMatrix( float mf[16] ) const
	{
		float x2 = 2.0f * x,  y2 = 2.0f * y,  z2 = 2.0f * z;

		float xy = x2 * y,  xz = x2 * z;
		float yy = y2 * y,  yw = y2 * w;
		float zw = z2 * w,  zz = z2 * z;

		mf[ 0] = 1.0f - ( yy + zz );
		mf[ 1] = ( xy - zw );
		mf[ 2] = ( xz + yw );
		mf[ 3] = 0.0f;

		float xx = x2 * x,  xw = x2 * w,  yz = y2 * z;

		mf[ 4] = ( xy +  zw );
		mf[ 5] = 1.0f - ( xx + zz );
		mf[ 6] = ( yz - xw );
		mf[ 7] = 0.0f;

		mf[ 8] = ( xz - yw );
		mf[ 9] = ( yz + xw );
		mf[10] = 1.0f - ( xx + yy );  
		mf[11] = 0.0f;  

		mf[12] = 0.0f;  
		mf[13] = 0.0f;   
		mf[14] = 0.0f;   
		mf[15] = 1.0f;
	}
	
	/// Invert
	CQuat Invert( ) const
	{
		return CQuat( -x, -y, -z, w );
	}

	/// (You could add an epsilon to this equality test if needed)
	inline bool operator== ( const CQuat &b ) const
	{
		return (x == b.x && y == b.y && z == b.z && w == b.w);
	}
	
	/// Addition
	CQuat operator+ ( const CQuat& b ) const
	{
		return CQuat( x + b.x, y + b.y, z + b.z, w + b.w );
	}

	/// Concatenate quaternions. (Note that order matters with concatenating Quaternion rotations)
	inline CQuat operator* (const CQuat &b) const
	{
		CQuat r;

		r.w = w*b.w - x*b.x  -  y*b.y  -  z*b.z;
		r.x = w*b.x + x*b.w  +  y*b.z  -  z*b.y;
		r.y = w*b.y + y*b.w  +  z*b.x  -  x*b.z;
		r.z = w*b.z + z*b.w  +  x*b.y  -  y*b.x;

		return r;
	}

	/// Apply rotation to vector
	inline CVec3 operator* (const CVec3 &v) const
	{
		CVec3 t;
		CQuat q,r,a;
		q.x = v.x;
		q.y = v.y;
		q.z = v.z;
		q.w = 0.0f;

		a.x = x;
		a.y = y;
		a.z = z;
		a.w = w;
		
		r = q * a.Invert();
		r = a * r;

		t.x = r.x;
		t.y = r.y;
		t.z = r.z;
		
		return t;
	}
	
	/// Scalar multiplication
	CQuat operator*( float s ) const
	{
		return CQuat(x * s, y * s, z * s, w * s );
	}
	

	/// Can be used the determine Quaternion neighbourhood
	float Dot( const CQuat& a ) const
	{
		return x * a.x + y * a.y + z * a.z + w * a.w;
	}

	// ------------------------------------
	/// Quaternions store scale as well as rotation, but usually we just want rotation, so we can normalize.
	// ------------------------------------
	int Normalize( )
	{
		float lengthSq = x * x + y * y + z * z + w * w;

		if (lengthSq == 0.0 ) return -1;
		if (lengthSq != 1.0 )
		{
			float scale = ( 1.0f / sqrtf( lengthSq ) );
			x *= scale;
			y *= scale;
			z *= scale;
			w *= scale;
			return 1;
		}
		return 0;
	}

	// ------------------------------------
	/// Creates a value for this Quaternion from spherical linear interpolation
	/// t is the interpolation value from 0 to 1
	// ------------------------------------
	void Slerp(const CQuat& a, const CQuat& b, float t)
	{
		float w1, w2;
	
		float cosTheta = a.Dot(b);
		float theta    = (float)acos(cosTheta);
		float sinTheta = (float)sin(theta);
	
		if( sinTheta > 0.001f )
		{
			w1 = float( sin( (1.0f-t)*theta ) / sinTheta);
			w2 = float( sin( t*theta) / sinTheta);
		}
		else
		{
			// CQuat a ~= CQuat b
			w1 = 1.0f - t;
			w2 = t;
		}
	
		*this = a*w1 + b*w2;
	}

	// ------------------------------------
	/// linearly interpolate each component, then normalize the Quaternion
	/// Unlike spherical interpolation, this does not rotate at a constant velocity,
	/// although that's not necessarily a bad thing
	// ------------------------------------
	void NLerp( const CQuat& a, const CQuat& b, float w2)
	{
		float w1 = 1.0f - w2;
	
		*this = a*w1 + b*w2;
		Normalize();
	}


	// ------------------------------------
	/// Set this Quat to aim the Z-Axis along the vector from P1 to P2
	// ------------------------------------
	void AimZAxis( const CVec3& P1, const CVec3& P2 )
	{
		CVec3 vAim = P2 - P1;
		vAim.Normalize();

		x = vAim.y;
		y = -vAim.x;
		z = 0.0f;
		w = 1.0f + vAim.z;

		if ( x == 0.0f && y == 0.0f && z == 0.0f && w == 0.0f ) {
			*this = CQuat( 0, 1, 0, 0 ); // If we can't normalize it, just set it
		} else {
			Normalize();
		}
	}

	// calculate the shortest rotation from a to b
	void shortestArc(const CVec3 &a, const CVec3 &b)
	{
		// Make a local copy of both vectors.
		CVec3 vec_a = a;
		CVec3 vec_b = b;

		// Make sure neither vector is zero length.  Also normalize
		// the vectors while we are at it.
		float vec_a_mag = vec_a.Normalize();
		float vec_b_mag = vec_b.Normalize();
		if (vec_a_mag < 0.00001f ||
			vec_b_mag < 0.00001f)
		{
			// Can't calculate a rotation from this.
			// Just return ZERO_ROTATION instead.
			Reset();
			return;
		}

		// Create an axis to rotate around, and the cos of the angle to rotate.
		CVec3 axis = vec_a.Cross(vec_b);
		float cos_theta  = vec_a.Dot(vec_b);

		// Check the angle between the vectors to see if they are parallel or anti-parallel.
		if (cos_theta > 1.0 - 0.00001f)
		{
			// a and b are parallel.  No rotation is necessary.
			Reset();
		}
		else if (cos_theta < -1.0 + 0.00001f)
		{
			// a and b are anti-parallel.
			// Rotate 180 degrees around some orthogonal axis.
			// Find the projection of the x-axis onto a, and try
			// using the vector between the projection and the x-axis
			// as the orthogonal axis.
			CVec3 proj = vec_a * vec_a.x / (vec_a.Dot(vec_a));
			CVec3 ortho_axis(1.f, 0.f, 0.f);
			ortho_axis -= proj;
			
			// Turn this into an orthonormal axis.
			float ortho_length = ortho_axis.Normalize();
			// If the axis' length is 0, then our guess at an orthogonal axis
			// was wrong (a is parallel to the x-axis).
			if (ortho_length < 0.00001f)
			{
				// Use the z-axis instead.
				ortho_axis.x=0.f;
				ortho_axis.y=0.f;
				ortho_axis.z=1.f;
			}

			// Construct a quaternion from this orthonormal axis.
			x = ortho_axis.x;
			y = ortho_axis.y;
			z = ortho_axis.z;
			w = 0.f;
		}
		else
		{
			// a and b are NOT parallel or anti-parallel.
			// Return the rotation between these vectors.
			float theta = (float)acos(cos_theta);

			SetAxis(RAD_TO_DEG(theta), axis.x, axis.y, axis.z);
		}
	}

	inline bool isFinite() const
	{
		return (_finite(x) && _finite(y) && _finite(z) && _finite(w));
	}

	// constrains rotation to a cone angle specified in radians
	void constrain(float radians)
	{
		const float cos_angle_lim = cosf( radians/2 );//(float)cos((double)(radians/2 ));	// w limit
		const float sin_angle_lim = sinf( radians/2 );//(float)sin((double)(radians/2 ));	// rotation axis length	limit
		//printf("constrain angle = %.3f, %.3f radians=%.3f\n", cos_angle_lim, sin_angle_lim, radians);

		if (w < 0.f)
		{
			x *= -1.f;
			y *= -1.f;
			z *= -1.f;
			w *= -1.f;
		}

		// if rotation angle is greater than limit (cos is less than limit)
		if( w < cos_angle_lim )
		{
			w = cos_angle_lim;
			float axis_len = sqrtf( x*x + y*y + z*z ); // sin(theta/2)
			float axis_mult_fact = sin_angle_lim / axis_len;
			x *= axis_mult_fact;
			y *= axis_mult_fact;
			z *= axis_mult_fact;
		}

		//return *this;
	}

	// There may be a cheaper way that avoids the sqrt.
	// Does sin_a = VX*VX + VY*VY + VZ*VZ?
	// Copied from Matrix and Quaternion FAQ 1.12
	inline void getAngleAxis(float* angle, float* ax, float* ay, float* az) const
	{
		float cos_a = w;
		if (cos_a > 1.0f) cos_a = 1.0f;
		if (cos_a < -1.0f) cos_a = -1.0f;

		float sin_a = (float) sqrt( 1.0f - cos_a * cos_a );

		if ( fabs( sin_a ) < 0.0005f )
			sin_a = 1.0f;
		else
			sin_a = 1.f/sin_a;

		float temp_angle = 2.0f * (float) acos( cos_a );
		if (temp_angle > M_PI)
		{
			// The (angle,axis) pair should never have angles outside [PI, -PI]
			// since we want the _shortest_ (angle,axis) solution.
			// Since acos is defined for [0, PI], and we multiply by 2.0, we
			// can push the angle outside the acceptible range.
			// When this happens we set the angle to the other portion of a 
			// full 2PI rotation, and negate the axis, which reverses the 
			// direction of the rotation (by the right-hand rule).
			*angle = 2.f * M_PI - temp_angle;
    		*ax = - x * sin_a;
    		*ay = - y * sin_a;
    		*az = - z * sin_a;
		}
		else
		{
			*angle = temp_angle;
    		*ax = x * sin_a;
    		*ay = y * sin_a;
    		*az = z * sin_a;
		}
	}
};

//////////////////////////////////////////////////////////////////////////////
//
// matrix
//
//////////////////////////////////////////////////////////////////////////////
const float PI = 3.14159265f;
const float TO_RAD = PI / 180.0f;

/// math helper class for matrix
class CMatrix 
{
public:
// Data
	float mf[ 16 ];

// Functions
	/// Constructor (if bIdentity==true, set identity)
	CMatrix( const int bIdentity = true )
	{
		if ( bIdentity ) Identity();
	}
	/// Set identity
	void Identity( )
	{
		mf[ 0] = 1.0f;    mf[ 1] = 0.0f;      mf[ 2] = 0.0f;    mf[ 3] = 0.0f;  
		mf[ 4] = 0.0f;    mf[ 5] = 1.0f;      mf[ 6] = 0.0f;    mf[ 7] = 0.0f;  
		mf[ 8] = 0.0f;    mf[ 9] = 0.0f;      mf[10] = 1.0f;    mf[11] = 0.0f;  
		mf[12] = 0.0f;    mf[13] = 0.0f;      mf[14] = 0.0f;    mf[15] = 1.0f;
	}

	/// Concatenate 2 matrices with the * operator
	inline CMatrix operator* (const CMatrix &InM) const
	{
		CMatrix Result( 0 );
		for (int i=0;i<16;i+=4)
			{
			for (int j=0;j<4;j++)
				{
				Result.mf[i + j] = mf[ i + 0] * InM.mf[ 0 + j] + mf[ i + 1] * InM.mf[ 4 + j]
					+ mf[ i + 2] * InM.mf[ 8 + j] + mf[ i + 3] * InM.mf[ 12 + j];
				}
			}
		return Result;
	}

	/// Matrix * Vec returns a vector that can be used to transform a 3D point
	/// along the orientation of the matrix
	inline CVec3 operator* (const CVec3 &Point ) const
	{
		float x = Point.x*mf[0] + Point.y*mf[4] + Point.z*mf[8]/*  + mf[12]*/;
		float y = Point.x*mf[1] + Point.y*mf[5] + Point.z*mf[9]/*  + mf[13]*/;
		float z = Point.x*mf[2] + Point.y*mf[6] + Point.z*mf[10]/* + mf[14]*/; 
		return CVec3( x, y, z );
	}

	/// Rotate the *this matrix fDegrees counter-clockwise around a single axis( either x=1, y=1, or z=1 )
	void Rotate( float fDegrees, int x, int y, int z )
	{
		CMatrix Temp;
		if (x == 1) Temp.RotX( -fDegrees );
		if (y == 1) Temp.RotY( -fDegrees );
		if (z == 1) Temp.RotZ( -fDegrees );
		*this = Temp * (*this);
	}

	void Scale( float sx, float sy, float sz )
	{
	int x;
		for (x = 0; x <  4; x++) mf[x]*=sx;
		for (x = 4; x <  8; x++) mf[x]*=sy;
		for (x = 8; x < 12; x++) mf[x]*=sz;
	}

	void Translate( const CVec3 &Test )
	{
	for (int j=0;j<4;j++)
		{
		mf[12+j] += Test.x * mf[j] + Test.y * mf[4+j] + Test.z * mf[8+j]; 
		}	 
	}
	/// Extract translation vector of matrix
	CVec3 GetTranslate( )
	{
		return CVec3( mf[12], mf[13], mf[14] );
	}
	
	/// Zero out the translation part of the matrix
	CMatrix RotationOnly( )
	{
		CMatrix Temp = *this;
		Temp.mf[12] = 0;
		Temp.mf[13] = 0;
		Temp.mf[14] = 0;
		return Temp;
	}

	/// Create a rotation matrix for a counter-clockwise rotation of fDegrees around an arbitrary axis(x, y, z)
	void RotateMatrix( float fDegrees, float x, float y, float z)
	{
		//Identity();
		float cosA = cosf(fDegrees*TO_RAD);
		float sinA = sinf(fDegrees*TO_RAD);
		float m = 1.0f - cosA;
		mf[0] = cosA + x*x*m;
		mf[5] = cosA + y*y*m;
		mf[10]= cosA + z*z*m;
	
		float tmp1 = x*y*m;
		float tmp2 = z*sinA;
		mf[4] = tmp1 + tmp2;
		mf[1] = tmp1 - tmp2;
	
		tmp1 = x*z*m;
		tmp2 = y*sinA;
		mf[8] = tmp1 - tmp2;
		mf[2] = tmp1 + tmp2;
	
		tmp1 = y*z*m;
		tmp2 = x*sinA;
		mf[9] = tmp1 + tmp2;
		mf[6] = tmp1 - tmp2;
	}

	/// Simple but not robust matrix inversion. (Doesn't work properly if there is a scaling or skewing transformation.)
	inline CMatrix InvertSimple()
	{
		CMatrix R(0);
		R.mf[0]  = mf[0]; 		R.mf[1]  = mf[4];		R.mf[2]  = mf[8];	R.mf[3]  = 0.0f;
		R.mf[4]  = mf[1];		R.mf[5]  = mf[5];		R.mf[6]  = mf[9];	R.mf[7]  = 0.0f;
		R.mf[8]  = mf[2];		R.mf[9]  = mf[6];		R.mf[10] = mf[10];	R.mf[11] = 0.0f;
		R.mf[12] = -(mf[12]*mf[0]) - (mf[13]*mf[1]) - (mf[14]*mf[2]);
		R.mf[13] = -(mf[12]*mf[4]) - (mf[13]*mf[5]) - (mf[14]*mf[6]);
		R.mf[14] = -(mf[12]*mf[8]) - (mf[13]*mf[9]) - (mf[14]*mf[10]);
		R.mf[15] = 1.0f;
		return R;
	}
	
	/// Invert for only a rotation, any translation is zeroed out
	CMatrix InvertRot( )
	{
		CMatrix R( 0 );
		R.mf[0]  = mf[0]; 		R.mf[1]  = mf[4];		R.mf[2]  = mf[8];	R.mf[3]  = 0.0f;
		R.mf[4]  = mf[1];		R.mf[5]  = mf[5];		R.mf[6]  = mf[9];	R.mf[7]  = 0.0f;
		R.mf[8]  = mf[2];		R.mf[9]  = mf[6];		R.mf[10] = mf[10];	R.mf[11] = 0.0f;
		R.mf[12] = 0;			R.mf[13] = 0;			R.mf[14] = 0;		R.mf[15] = 1.0f;
		return R;
	}	

	void ToEuler(float *h, float *p, float *r)
	{
		if (mf[4] > 0.99998) { // singularity at north pole
				*h = atan2(mf[2],mf[10]);
				*p = M_PI/2;
				*r = 0;
		}
		else if (mf[4] < -0.99998) { // singularity at south pole
				*h = atan2(mf[2],mf[10]);
				*p = -M_PI/2;
				*r = 0;
		}else{
				*h = atan2(-mf[8],mf[0]); // heading
				*r = atan2(-mf[6],mf[5]); // bank
				*p = asin(mf[4]); // attitude
		}

		*h *= -180.0/M_PI;
		*p *= -180.0/M_PI;
		*r *= -180.0/M_PI;
	}


private:
	/// helper for Rotate
	void RotX(float angle)
	{  
		mf[5]  = cosf(angle*TO_RAD);
		mf[6]  = sinf(angle*TO_RAD);
		mf[9]  = -sinf(angle*TO_RAD);
		mf[10] = cosf(angle*TO_RAD);
	}
	/// helper for Rotate
	void RotY(float angle)
	{
		mf[0]  =  cosf(angle*TO_RAD);
		mf[2]  =  -sinf(angle*TO_RAD);
		mf[8]  =  sinf(angle*TO_RAD);
		mf[10] =  cosf(angle*TO_RAD);
	}
	/// helper for Rotate
	void RotZ(float angle)
	{
		mf[0] =  cosf(angle*TO_RAD);
		mf[1] =  sinf(angle*TO_RAD);
		mf[4] = -sinf(angle*TO_RAD);
		mf[5] =  cosf(angle*TO_RAD);
	}
};

#endif
