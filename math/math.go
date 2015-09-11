// Copyright 2015, Timothy Bogdala <tdb@animal-machine.com>
// See the LICENSE file for more details.

/*

The math module of this project defines the floating point precision to
be used and the mathematical types to be used.

From there it also defines mathematial operations on vectors, matrixes
and quaternions that operate by reference.

All matrixes will be created in column-major ordering.

*/

package math

import (
	"math"
)

// Real is the type of float used in the library.
type Real float64

// Espilon is used to test equality of the floats and represents how
// close two Reals can be and still test positive for equality.
const Epsilon Real = 1e-7

const (
	MinNormal = Real(1.1754943508222875e-38) // 1 / 2**(127 - 1)
	MinValue  = Real(math.SmallestNonzeroFloat64)
	MaxValue  = Real(math.MaxFloat64)
)

var (
	InfPos = Real(math.Inf(1))
	InfNeg = Real(math.Inf(-1))
	NaN    = Real(math.NaN())
)

// Matrix3 is a 3x3 matrix of floats in column-major order.
type Matrix3 [9]Real

// Matrix3x4 is a 3x4 matrix of floats in column-major order
// that can be used to hold a rotation and translation in 3D space
// where the 4th row would have been [0 0 0 1]
type Matrix3x4 [12]Real

// Matrix4 is a 4x4 matrix of floats in column-major order.
type Matrix4 [16]Real

// Vector3 is a vector of three floats.
type Vector3 [3]Real

// Vector4 is a vector of four floats.
type Vector4 [4]Real

// Quat is the type of a quaternion (w,x,y,z).
type Quat Vector4

// RealEqual tests the two Real numbers for equality, which really means
// that it tests whether or not the difference between the two is
// smaller than Epsilon.
func RealEqual(a, b Real) bool {
	// handle cases like inf
	if a == b {
		return true
	}

	diff := Real(math.Abs(float64(a - b)))

	// if a or b is 0 or really close to it
	if a*b == 0 || diff < MinNormal {
		return diff < Epsilon*Epsilon
	}

	return diff/Real(math.Abs(float64(a))+math.Abs(float64(b))) < Epsilon
}

// DegToRad converts degrees to radians
func DegToRad(angle Real) Real {
	return angle * math.Pi / 180.0
}

// RadToDeg converts radians to degrees
func RadToDeg(angle Real) Real {
	return angle * 180.0 / math.Pi
}

// RealAbs is an absolute value wrapper for the Real type.
func RealAbs(a Real) Real {
	return Real(math.Abs(float64(a)))
}

// RealSqrt is a square root wrapper for the Real type.
func RealSqrt(a Real) Real {
	return Real(math.Sqrt(float64(a)))
}

// RealSin is a sin function wrapper for the Real type.
func RealSin(a Real) Real {
	return Real(math.Sin(float64(a)))
}

// RealCos is a cos function wrapper for the Real type.
func RealCos(a Real) Real {
	return Real(math.Cos(float64(a)))
}

// RealIsNaN returns true if the value is Not a Number.
func RealIsNaN(a Real) bool {
	return math.IsNaN(float64(a))
}
