// Copyright 2015, Timothy Bogdala <tdb@animal-machine.com>
// See the LICENSE file for more details.

package math

import (
	"math"
)

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
