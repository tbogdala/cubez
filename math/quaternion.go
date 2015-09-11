// Copyright 2015, Timothy Bogdala <tdb@animal-machine.com>
// See the LICENSE file for more details.

package math

import "math"

// QuatFromAxis creates an quaternion from an axis and an angle.
func QuatFromAxis(angle, x, y, z Real) Quat {
	s := RealSin(angle / 2.0)
	c := RealCos(angle / 2.0)

	result := Quat{c, x * s, y * s, z * s}
	result.Normalize()
	return result
}

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
	return RealSqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3])
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

// Rotate rotates a vector by the rotation this quaternion represents.
func (q *Quat) Rotate(v *Vector3) Vector3 {
	qvec := Vector3{q[1], q[2], q[3]}
	cross := qvec.Cross(v)

	// v + 2q_w * (q_v x v) + 2q_v x (q_v x v)
	result := *v

	qvec.MulWith(2.0)
	c2 := qvec.Cross(&cross)
	result.Add(&c2)

	cross.MulWith(2.0 * q[0])
	result.Add(&cross)
	return result
}

// Conjugated returns the conjugate of a quaternion. Equivalent to Quat{w,-x,-y,-z}.
func (q *Quat) Conjugated() Quat {
	return Quat{q[0], -q[1], -q[2], -q[3]}
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

// LookAt sets the quaternion to the orientation needed to look at a 'center' from
// the 'eye' position with 'up' as a reference vector for the up direction.
// Note: this was modified from the go-gl/mathgl library.
func (q *Quat) LookAt(eye, center, up *Vector3) {
	direction := center
	direction.Sub(eye)
	direction.Normalize()

	// Find the rotation between the front of the object (that we assume towards Z-,
	// but this depends on your model) and the desired direction
	rotDir := QuatBetweenVectors(&Vector3{0, 0, -1}, direction)

	// Recompute up so that it's perpendicular to the direction
	// You can skip that part if you really want to force up
	//right := direction.Cross(up)
	//up = right.Cross(direction)

	// Because of the 1rst rotation, the up is probably completely screwed up.
	// Find the rotation between the "up" of the rotated object, and the desired up
	upCur := rotDir.Rotate(&Vector3{0, 1, 0})
	rotTarget := QuatBetweenVectors(&upCur, up)

	rotTarget.Mul(&rotDir) // remember, in reverse order.
	rotTarget.Inverse()    // camera rotation should be inversed!

	q[0] = rotTarget[0]
	q[1] = rotTarget[1]
	q[2] = rotTarget[2]
	q[3] = rotTarget[3]
}

// QuatBetweenVectors calculates the rotation between two vectors.
// Note: this was modified from the go-gl/mathgl library.
func QuatBetweenVectors(s, d *Vector3) Quat {
	start := *s
	dest := *d
	start.Normalize()
	dest.Normalize()

	cosTheta := start.Dot(&dest)
	if cosTheta < -1.0+Epsilon {
		// special case when vectors in opposite directions:
		// there is no "ideal" rotation axis
		// So guess one; any will do as long as it's perpendicular to start
		posX := Vector3{1.0, 0.0, 0.0}
		axis := posX.Cross(&start)
		if axis.Dot(&axis) < Epsilon {
			// bad luck, they were parallel, try again!
			posY := Vector3{0.0, 1.0, 0.0}
			axis = posY.Cross(&start)
		}

		axis.Normalize()
		return QuatFromAxis(math.Pi, axis[0], axis[1], axis[2])
	}

	axis := start.Cross(&dest)
	ang := RealSqrt((1.0 + cosTheta) * 2.0)
	axis.MulWith(1.0 / ang)

	return Quat{
		ang * 0.5,
		axis[0], axis[1], axis[2],
	}
}

// Inverse calculates the inverse of a quaternion. The inverse is equivalent
// to the conjugate divided by the square of the length.
//
// This method computes the square norm by directly adding the sum
// of the squares of all terms instead of actually squaring q1.Len(),
// both for performance and percision.
func (q *Quat) Inverse() {
	c := q.Conjugated()
	c.Scale(1.0 / q.Dot(q))
	q[0] = c[0]
	q[1] = c[1]
	q[2] = c[2]
	q[3] = c[3]
}

// Scale scales every element of the quaternion by some constant factor.
func (q *Quat) Scale(c Real) {
	q[0] *= c
	q[1] *= c
	q[2] *= c
	q[3] *= c
}

// Dot calculates the dot product between two quaternions, equivalent to if this was a Vector4
func (q *Quat) Dot(q2 *Quat) Real {
	return q[0]*q2[0] + q[1]*q2[1] + q[2]*q2[2] + q[3]*q2[3]
}
