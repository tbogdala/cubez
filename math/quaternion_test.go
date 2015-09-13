// Copyright 2015, Timothy Bogdala <tdb@animal-machine.com>
// See the LICENSE file for more details.

package math

import (
	"math"
	"testing"
)

func TestQuatMulIdentity(t *testing.T) {
	var i1, i2 Quat
	i1.SetIdentity()
	i2.SetIdentity()

	i1.Mul(&i2)

	if !RealEqual(i1[0], 1.0) || !RealEqual(i1[1], 0.0) || !RealEqual(i1[2], 0.0) || !RealEqual(i1[3], 0.0) {
		t.Errorf("Multiplication of identities does not yield identity")
	}
}

func TestQuatLen(t *testing.T) {
	q1 := Quat{0.0, 1.0, 0.0, 0.0}
	len1 := q1.Len()
	if !RealEqual(len1, 1.0) {
		t.Errorf("Quaternion calculation of length didn't yield the correct answer: %v", len1)
	}

	q2 := Quat{0.0, 0.0000000000001, 0.0, 0.0}
	len2 := q2.Len()
	if !RealEqual(len2, 1e-13) {
		t.Errorf("Quaternion calculation of length didn't yield the correct answer: %v", len2)
	}

	q3 := Quat{0.0, MaxValue, 1.0, 0.0}
	len3 := q3.Len()
	if !RealEqual(len3, InfPos) {
		t.Errorf("Quaternion calculation of length didn't yield the correct answer: %v", len3)
	}

	q4 := Quat{4.0, 1.0, 2.0, 3.0}
	len4 := q4.Len()
	if !RealEqual(len4, Real(math.Sqrt(1*1+2*2+3*3+4*4))) {
		t.Errorf("Quaternion calculation of length didn't yield the correct answer: %v", len4)
	}
}

func TestQuatNormalize(t *testing.T) {
	q1 := Quat{0.0, 0.0, 0.0, 0.0}
	q1.Normalize()
	if !RealEqual(q1[0], 1.0) || !RealEqual(q1[1], 0.0) || !RealEqual(q1[2], 0.0) || !RealEqual(q1[3], 0.0) {
		t.Errorf("Quaternion normalization didn't yield the correct quaternion: %v", q1)
	}

	q1 = Quat{0.0, 1.0, 0.0, 0.0}
	q1.Normalize()
	if !RealEqual(q1[0], 0.0) || !RealEqual(q1[1], 1.0) || !RealEqual(q1[2], 0.0) || !RealEqual(q1[3], 0.0) {
		t.Errorf("Quaternion normalization didn't yield the correct quaternion: %v", q1)
	}

	q1 = Quat{0.0, 0.0000000000001, 0.0, 0.0}
	q1.Normalize()
	if !RealEqual(q1[0], 0.0) || !RealEqual(q1[1], 1.0) || !RealEqual(q1[2], 0.0) || !RealEqual(q1[3], 0.0) {
		t.Errorf("Quaternion normalization didn't yield the correct quaternion: %v", q1)
	}

	q1 = Quat{0.0, MaxValue, 1.0, 0.0}
	q1.Normalize()
	if !RealEqual(q1[0], 0.0) || !RealEqual(q1[1], 1.0) || !RealEqual(q1[2], 0.0) || !RealEqual(q1[3], 0.0) {
		t.Errorf("Quaternion normalization didn't yield the correct quaternion: %v", q1)
	}
}

func TestQuatMul(t *testing.T) {
	q1 := Quat{1.0, 0.5, -3.0, 4.0}
	q2 := Quat{6.0, 2.0, 1.0, -9.0}

	q1.Mul(&q2)

	if !RealEqual(q1[0], 44.0) || !RealEqual(q1[1], 28.0) || !RealEqual(q1[2], -4.5) || !RealEqual(q1[3], 21.5) {
		t.Errorf("Quaternion multiplication didn't yield the correct quaternion: %v", q1)
	}
}

func TestQuatRotate(t *testing.T) {
	q := Quat{1.0, 0.0, 1.0, 0.0}
	v := Vector3{1.0, 0.0, 0.0}
	q.Normalize()
	result := q.Rotate(&v)
	if !RealEqual(result[0], 0.0) || !RealEqual(result[1], 0.0) || !RealEqual(result[2], -1.0) {
		t.Errorf("Quaternion rotation didn't yield the correct vector:\n\t(q=%v:v%v)%v", q, v, result)
	}

	// y-axis rotation
	q = QuatFromAxis(0.0, 0.0, 1.0, 0.0)
	v = Vector3{1.0, 0.0, 0.0}
	result = q.Rotate(&v)
	if !RealEqual(result[0], 1.0) || !RealEqual(result[1], 0.0) || !RealEqual(result[2], 0.0) {
		t.Errorf("Quaternion rotation didn't yield the correct vector:\n\t(q=%v:v%v)%v", q, v, result)
	}

	q = QuatFromAxis(DegToRad(90), 0.0, 1.0, 0.0)
	v = Vector3{1.0, 0.0, 0.0}
	result = q.Rotate(&v)
	if !RealEqual(result[0], 0.0) || !RealEqual(result[1], 0.0) || !RealEqual(result[2], -1.0) {
		t.Errorf("Quaternion rotation didn't yield the correct vector:\n\t(q=%v:v%v)%v", q, v, result)
	}

	q = QuatFromAxis(DegToRad(180), 0.0, 1.0, 0.0)
	v = Vector3{1.0, 0.0, 0.0}
	result = q.Rotate(&v)
	if !RealEqual(result[0], -1.0) || !RealEqual(result[1], 0.0) || !RealEqual(result[2], 0.0) {
		t.Errorf("Quaternion rotation didn't yield the correct vector:\n\t(q=%v:v%v)%v", q, v, result)
	}

	q = QuatFromAxis(DegToRad(270), 0.0, 1.0, 0.0)
	v = Vector3{0.0, 0.0, 1.0}
	result = q.Rotate(&v)
	if !RealEqual(result[0], -1.0) || !RealEqual(result[1], 0.0) || !RealEqual(result[2], 0.0) {
		t.Errorf("Quaternion rotation didn't yield the correct vector:\n\t(q=%v:v%v)%v", q, v, result)
	}

	// x-axis rotation
	q = QuatFromAxis(0.0, 0.0, 0.0, 1.0)
	v = Vector3{0.0, 1.0, 0.0}
	result = q.Rotate(&v)
	if !RealEqual(result[0], 0.0) || !RealEqual(result[1], 1.0) || !RealEqual(result[2], 0.0) {
		t.Errorf("Quaternion rotation didn't yield the correct vector:\n\t(q=%v:v%v)%v", q, v, result)
	}

	q = QuatFromAxis(DegToRad(90), 1.0, 0.0, 0.0)
	v = Vector3{0.0, 1.0, 0.0}
	result = q.Rotate(&v)
	if !RealEqual(result[0], 0.0) || !RealEqual(result[1], 0.0) || !RealEqual(result[2], 1.0) {
		t.Errorf("Quaternion rotation didn't yield the correct vector:\n\t(q=%v:v%v)%v", q, v, result)
	}

	q = QuatFromAxis(DegToRad(180), 1.0, 0.0, 0.0)
	v = Vector3{0.0, 1.0, 0.0}
	result = q.Rotate(&v)
	if !RealEqual(result[0], 0.0) || !RealEqual(result[1], -1.0) || !RealEqual(result[2], 0.0) {
		t.Errorf("Quaternion rotation didn't yield the correct vector:\n\t(q=%v:v%v)%v", q, v, result)
	}

	q = QuatFromAxis(DegToRad(270), 1.0, 0.0, 0.0)
	v = Vector3{0.0, 1.0, 0.0}
	result = q.Rotate(&v)
	if !RealEqual(result[0], 0.0) || !RealEqual(result[1], 0.0) || !RealEqual(result[2], -1.0) {
		t.Errorf("Quaternion rotation didn't yield the correct vector:\n\t(q=%v:v%v)%v", q, v, result)
	}

	// z-axis rotation
	q = QuatFromAxis(0.0, 0.0, 0.0, 1.0)
	v = Vector3{0.0, 1.0, 0.0}
	result = q.Rotate(&v)
	if !RealEqual(result[0], 0.0) || !RealEqual(result[1], 1.0) || !RealEqual(result[2], 0.0) {
		t.Errorf("Quaternion rotation didn't yield the correct vector:\n\t(q=%v:v%v)%v", q, v, result)
	}

	q = QuatFromAxis(DegToRad(90), 0.0, 0.0, 1.0)
	v = Vector3{0.0, 1.0, 0.0}
	result = q.Rotate(&v)
	if !RealEqual(result[0], -1.0) || !RealEqual(result[1], 0.0) || !RealEqual(result[2], 0.0) {
		t.Errorf("Quaternion rotation didn't yield the correct vector:\n\t(q=%v:v%v)%v", q, v, result)
	}

	q = QuatFromAxis(DegToRad(180), 0.0, 0.0, 1.0)
	v = Vector3{0.0, 1.0, 0.0}
	result = q.Rotate(&v)
	if !RealEqual(result[0], 0.0) || !RealEqual(result[1], -1.0) || !RealEqual(result[2], 0.0) {
		t.Errorf("Quaternion rotation didn't yield the correct vector:\n\t(q=%v:v%v)%v", q, v, result)
	}

	q = QuatFromAxis(DegToRad(270), 0.0, 0.0, 1.0)
	v = Vector3{0.0, 1.0, 0.0}
	result = q.Rotate(&v)
	if !RealEqual(result[0], 1.0) || !RealEqual(result[1], 0.0) || !RealEqual(result[2], 0.0) {
		t.Errorf("Quaternion rotation didn't yield the correct vector:\n\t(q=%v:v%v)%v", q, v, result)
	}
}
