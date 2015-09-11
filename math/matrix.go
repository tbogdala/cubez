// Copyright 2015, Timothy Bogdala <tdb@animal-machine.com>
// See the LICENSE file for more details.

package math

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

// SetInertiaTensorCoeffs sets the value of the matrix from inertia tensor values.
func (m *Matrix3) SetInertiaTensorCoeffs(ix, iy, iz, ixy, ixz, iyz Real) {
	m[0], m[3], m[6] = ix, -ixy, -ixz
	m[1], m[4], m[7] = -ixy, iy, -iyz
	m[2], m[5], m[8] = -ixz, -iyz, iz
}

// SetBlockInertiaTensor sets the value of the matrix as an inertia tensor
// of a rectangular block aligned with the body's coordinate system with the
// given axis half sizes and mass.
func (m *Matrix3) SetBlockInertiaTensor(halfSize *Vector3, mass Real) {
	squares := *halfSize
	squares.ComponentProduct(halfSize)
	m.SetInertiaTensorCoeffs(
		0.3 * mass * (squares[1]+squares[2]),
		0.3 * mass * (squares[0]+squares[2]),
		0.3 * mass * (squares[0]+squares[1]),
		0.0, 0.0, 0.0,
	)
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

// TransformTranspose transforms the given vector by the transpose
// of this matrix
func (m *Matrix3) TransformTranspose(v *Vector3) Vector3 {
	return Vector3{
		v[0]*m[0] + v[1]*m[1] + v[2]*m[2],
		v[0]*m[3] + v[1]*m[4] + v[2]*m[5],
		v[0]*m[6] + v[1]*m[7] + v[2]*m[8],
	}
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
func (m *Matrix3x4) MulMatrix3x4(o *Matrix3x4) Matrix3x4 {
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

// TransformInverse transforms the vector by the transformational
// inverse of this matrix.
// NOTE: will not work on matrixes with scale or shears.
func (m *Matrix3x4) TransformInverse(v *Vector3) Vector3 {
	tmp := *v
	tmp[0] -= m[9]
	tmp[1] -= m[10]
	tmp[2] -= m[11]

	return Vector3{
		tmp[0]*m[0] + tmp[1]*m[1] + tmp[2]*m[2],
		tmp[0]*m[3] + tmp[1]*m[4] + tmp[2]*m[5],
		tmp[0]*m[6] + tmp[1]*m[7] + tmp[2]*m[8],
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
