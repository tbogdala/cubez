// Copyright 2015, Timothy Bogdala <tdb@animal-machine.com>
// See the LICENSE file for more details.

/*

The math module of this project defines the floating point precision to
be used and the mathematical types to be used.

From there it also defines mathematial operations on vectors, matrixes
and quaternions that operate by reference.

All matrixes will be created in column-major ordering.

*/

package cubez

import (
	"math"
)

// Real is the type of float used in the library.
type Real float64

// Espilon is used to test equality of the floats and represents how
// close two Reals can be and still test positive for equality.
const Epsilon Real = 1e-30

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

/*
==================================================================================================
  VECTORS
==================================================================================================
*/

// Add adds a vector to another vector.
func (v *Vector3) Add(v2 *Vector3) {
	v[0] += v2[0]
	v[1] += v2[1]
	v[2] += v2[2]
}

// Add adds a vector, scaled by a Real, to another vector.
func (v *Vector3) AddScaled(v2 *Vector3, scale Real) {
	v[0] += v2[0] * scale
	v[1] += v2[1] * scale
	v[2] += v2[2] * scale
}

// Clear sets the vector to {0.0, 0.0, 0.0}.
func (v *Vector3) Clear() {
	v[0], v[1], v[2] = 0.0, 0.0, 0.0
}

// ComponentProduct performs a component-wise product with another vector.
func (v *Vector3) ComponentProduct(v2 *Vector3) {
	v[0] *= v2[0]
	v[1] *= v2[1]
	v[2] *= v2[2]
}

// Cross returns the cross product of this vector with another.
func (v *Vector3) Cross(v2 *Vector3) Vector3 {
	return Vector3{
		v[1]*v2[2] - v[2]*v2[1],
		v[2]*v2[0] - v[0]*v2[2],
		v[0]*v2[1] - v[1]*v2[0],
	}
}

// Dot returns the dot product of this vector with another.
func (v *Vector3) Dot(v2 *Vector3) Real {
	return v[0]*v2[0] + v[1]*v2[1] + v[2]*v2[2]
}

// Magnitude returns the magnitude of the vector.
func (v *Vector3) Magnitude() Real {
	return Real(math.Sqrt(float64(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])))
}

// MulWith multiplies a vector by a Real number.
func (v *Vector3) MulWith(r Real) {
	v[0] *= r
	v[1] *= r
	v[2] *= r
}

// Set sets the vector equal to the values of the second vector.
func (v *Vector3) Set(v2 *Vector3) {
	v[0] = v2[0]
	v[1] = v2[1]
	v[2] = v2[2]
}

// Sub subtracts a second vector from this vector.
func (v *Vector3) Sub(v2 *Vector3) {
	v[0] -= v2[0]
	v[1] -= v2[1]
	v[2] -= v2[2]
}

// MulWith multiplies a vector by a Real number.
func (v *Vector4) MulWith(r Real) {
	v[0] *= r
	v[1] *= r
	v[2] *= r
	v[3] *= r
}

/*
==================================================================================================
  MATRIXES
==================================================================================================
*/

// Sanity conversion charts between different ordering:
//
//  col major        row major     col maj 4x4    row major    col major
// 0  3  6  9       0  1  2  3    0  4  8  12     0  1  2      0  3  6
// 1  4  7  10  <=  4  5  6  7    1  5  9  13     3  4  5  =>  1  4  7
// 2  5  8  11      8  9 10 11    2  6  10 14     6  7  8      2  5  8
//                                3  7  11 15

// SetIdentity loads the matrix with its identity.
func (m *Matrix3) SetIdentity() {
	m[0], m[3], m[6] = 1.0, 0.0, 0.0
	m[1], m[4], m[7] = 0.0, 1.0, 0.0
	m[2], m[5], m[8] = 0.0, 0.0, 1.0
}

// SetIdentity loads the matrix with its identity.
func (m *Matrix3x4) SetIdentity() {
	m[0], m[3], m[6], m[9] = 1.0, 0.0, 0.0, 0.0
	m[1], m[4], m[7], m[10] = 0.0, 1.0, 0.0, 0.0
	m[2], m[5], m[8], m[11] = 0.0, 0.0, 1.0, 0.0
}

// SetIdentity loads the matrix with its identity.
func (m *Matrix4) SetIdentity() {
	m[0], m[4], m[8], m[12] = 1.0, 0.0, 0.0, 0.0
	m[1], m[5], m[9], m[13] = 0.0, 1.0, 0.0, 0.0
	m[2], m[6], m[10], m[14] = 0.0, 0.0, 1.0, 0.0
	m[3], m[7], m[11], m[15] = 0.0, 0.0, 0.0, 1.0
}

// Add adds the values of a 3x3 matrix to this matrix.
func (m *Matrix3) Add(m2 *Matrix3) {
	m[0] += m2[0]
	m[1] += m2[1]
	m[2] += m2[2]
	m[3] += m2[3]
	m[4] += m2[4]
	m[5] += m2[5]
	m[6] += m2[6]
	m[7] += m2[7]
	m[8] += m2[8]
}

// SetComponents sets the matrix vales based off of the three vectors given.
// Each vector should be arranged as a column in the matrix and should
// then be passed in order.
func (m *Matrix3) SetComponents(v1 *Vector3, v2 *Vector3, v3 *Vector3) {
	m[0], m[3], m[6] = v1[0], v2[0], v3[0]
	m[1], m[4], m[7] = v1[1], v2[1], v3[1]
	m[2], m[5], m[8] = v1[2], v2[2], v3[2]
}

// MulVector3 multiplies a 3x3 matrix by a vector.
func (m *Matrix3) MulVector3(v *Vector3) Vector3 {
	return Vector3{
		m[0]*v[0] + m[3]*v[1] + m[6]*v[2],
		m[1]*v[0] + m[4]*v[1] + m[7]*v[2],
		m[2]*v[0] + m[5]*v[1] + m[8]*v[2],
	}
}

// MulMatrix3 multiplies a 3x3 matrix by another 3x3 matrix.
func (m1 *Matrix3) MulMatrix3(m2 *Matrix3) Matrix3 {
	return Matrix3{
		m1[0]*m2[0] + m1[3]*m2[1] + m1[6]*m2[2],
		m1[1]*m2[0] + m1[4]*m2[1] + m1[7]*m2[2],
		m1[2]*m2[0] + m1[5]*m2[1] + m1[8]*m2[2],
		m1[0]*m2[3] + m1[3]*m2[4] + m1[6]*m2[5],
		m1[1]*m2[3] + m1[4]*m2[4] + m1[7]*m2[5],
		m1[2]*m2[3] + m1[5]*m2[4] + m1[8]*m2[5],
		m1[0]*m2[6] + m1[3]*m2[7] + m1[6]*m2[8],
		m1[1]*m2[6] + m1[4]*m2[7] + m1[7]*m2[8],
		m1[2]*m2[6] + m1[5]*m2[7] + m1[8]*m2[8],
	}
}

// MulWith multiplies a 3x3 matrix by a scalar value.
func (m *Matrix3) MulWith(s Real) {
	m[0] *= s
	m[1] *= s
	m[2] *= s
	m[3] *= s
	m[4] *= s
	m[5] *= s
	m[6] *= s
	m[7] *= s
	m[8] *= s
}

// Transpose produces the transpose of this matrix.
func (m *Matrix3) Transpose() Matrix3 {
	return Matrix3{
		m[0], m[3], m[6],
		m[1], m[4], m[7],
		m[2], m[5], m[8],
	}
}

// Determinant calculates the determinant of a matrix which is a measure of a square matrix's
// singularity and invertability, among other things.
func (m *Matrix3) Determinant() Real {
	return m[0]*m[4]*m[8] + m[3]*m[7]*m[2] + m[6]*m[1]*m[5] - m[6]*m[4]*m[2] - m[3]*m[1]*m[8] - m[0]*m[7]*m[5]
}

// Inv computes the inverse of a square matrix. An inverse is a square matrix such
// that when multiplied by the original, yields the identity.
func (m *Matrix3) Invert() Matrix3 {
	det := m.Determinant()
	if RealEqual(det, 0.0) {
		return Matrix3{}
	}

	retMat := Matrix3{
		m[4]*m[8] - m[5]*m[7],
		m[2]*m[7] - m[1]*m[8],
		m[1]*m[5] - m[2]*m[4],
		m[5]*m[6] - m[3]*m[8],
		m[0]*m[8] - m[2]*m[6],
		m[2]*m[3] - m[0]*m[5],
		m[3]*m[7] - m[4]*m[6],
		m[1]*m[6] - m[0]*m[7],
		m[0]*m[4] - m[1]*m[3],
	}

	retMat.MulWith(1 / det)
	return retMat
}

// SetAsTransform sets the 3x4 matrix to be a transform matrix based
// on the position and orientation passed in.
func (m *Matrix3x4) SetAsTransform(pos *Vector3, rot *Quat) {
	w, x, y, z := rot[0], rot[1], rot[2], rot[3]

	m[0] = 1 - 2*y*y - 2*z*z
	m[1] = 2*x*y + 2*w*z
	m[2] = 2*x*z - 2*w*y

	m[3] = 2*x*y - 2*w*z
	m[4] = 1 - 2*x*x - 2*z*z
	m[5] = 2*y*z + 2*w*x

	m[6] = 2*x*z + 2*w*y
	m[7] = 2*y*z - 2*w*x
	m[8] = 1 - 2*x*x - 2*y*y

	m[9] = pos[0]
	m[10] = pos[1]
	m[11] = pos[2]
}

// MulVector3 transforms the given vector by the matrix and returns the result.
func (m *Matrix3x4) MulVector3(v *Vector3) Vector3 {
	return Vector3{
		v[0]*m[0] + v[1]*m[3] + v[2]*m[6] + m[9],
		v[0]*m[1] + v[1]*m[4] + v[2]*m[7] + m[10],
		v[0]*m[2] + v[1]*m[5] + v[2]*m[8] + m[11],
	}
}

// Mul3x4 multiplies a 3x4 matrix by another 3x4 matrix. This operation is meant
// to mimic a 4x4 * 4x4 operation where the last row is {0, 0, 0, 1}.
func (m *Matrix3x4) Mul3x4(o *Matrix3x4) Matrix3x4 {
	return Matrix3x4{
		m[0]*o[0] + m[3]*o[1] + m[6]*o[2], // [0]
		m[1]*o[0] + m[4]*o[1] + m[7]*o[2], // [1]
		m[2]*o[0] + m[5]*o[1] + m[8]*o[2], // [2]

		m[0]*o[3] + m[3]*o[4] + m[6]*o[5], // [3]
		m[1]*o[3] + m[4]*o[4] + m[7]*o[5], // [4]
		m[2]*o[3] + m[5]*o[4] + m[8]*o[5], // [5]

		m[0]*o[6] + m[3]*o[7] + m[6]*o[8], // [6]
		m[1]*o[6] + m[4]*o[7] + m[7]*o[8], // [7]
		m[2]*o[6] + m[5]*o[7] + m[8]*o[8], // [8]

		m[0]*o[9] + m[3]*o[10] + m[6]*o[11] + m[9],  // [9]
		m[1]*o[9] + m[4]*o[10] + m[7]*o[11] + m[10], // [10]
		m[2]*o[9] + m[5]*o[10] + m[8]*o[11] + m[11], // [11]

	}
}

func (m *Matrix3x4) GetAxis(colNumber int) Vector3 {
	var i int
	if colNumber > 3 {
		i = 3
	} else {
		i = colNumber
	}
	return Vector3{
		m[i*3+0],
		m[i*3+1],
		m[i*3+2],
	}
}

/*
==================================================================================================
  QUATERNIONS {r, i, j, k} or {w, x, y ,z}
==================================================================================================
*/

// AddScaledVector adds the given vector to this quaternion, scaled
// by the given amount.
func (q *Quat) AddScaledVector(v *Vector3, scale Real) {
	var temp Quat
	temp[0] = 0.0
	temp[1] = v[0] * scale
	temp[2] = v[1] * scale
	temp[3] = v[2] * scale

	temp.Mul(q)

	q[0] += temp[0] * 0.5
	q[1] += temp[1] * 0.5
	q[2] += temp[2] * 0.5
	q[3] += temp[3] * 0.5
}

// SetIdentity loads the quaternion with its identity.
func (q *Quat) SetIdentity() {
	q[0], q[1], q[2], q[3] = 1.0, 0.0, 0.0, 0.0
}

// Len returns the length of the quaternion.
func (q *Quat) Len() Real {
	return Real(math.Sqrt(float64(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3])))
}

// Mul multiplies the quaternion by another quaternion.
func (q *Quat) Mul(q2 *Quat) {
	var w, x, y, z Real
	w = q[0]*q2[0] - q[1]*q2[1] - q[2]*q2[2] - q[3]*q2[3]
	x = q[0]*q2[1] + q[1]*q2[0] + q[2]*q2[3] - q[3]*q2[2]
	y = q[0]*q2[2] + q[2]*q2[0] + q[3]*q2[1] - q[1]*q2[3]
	z = q[0]*q2[3] + q[3]*q2[0] + q[1]*q2[2] - q[2]*q2[1]
	q[0], q[1], q[2], q[3] = w, x, y, z
}

// Normalize normalizes the quaternion.
func (q *Quat) Normalize() {
	length := q.Len()

	if RealEqual(1.0, length) {
		return
	}

	if length == 0.0 {
		q.SetIdentity()
		return
	}

	if length == InfPos {
		length = MaxValue
	}

	invLength := 1.0 / length
	q[0] *= invLength
	q[1] *= invLength
	q[2] *= invLength
	q[3] *= invLength
}
