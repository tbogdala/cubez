// Copyright 2015, Timothy Bogdala <tdb@animal-machine.com>
// See the LICENSE file for more details.

package math

import (
	"testing"
)

func TestMat3Identity(t *testing.T) {
  var m1, m2 Matrix3
	m1.SetIdentity()
	m2.SetIdentity()

	m3 := m1.MulMatrix3(&m2)

	if !RealEqual(m3[0], 1.0) || !RealEqual(m3[3], 0.0) || !RealEqual(m3[6], 0.0) ||
    !RealEqual(m3[1], 0.0) || !RealEqual(m3[4], 1.0) || !RealEqual(m3[7], 0.0) ||
    !RealEqual(m3[2], 0.0) || !RealEqual(m3[5], 0.0) || !RealEqual(m3[8], 1.0) {
		t.Errorf("Multiplication of identities does not yield identity:\n\t%v", m3)
	}
}

func TestMat3x4Identity(t *testing.T) {
  var m1, m2 Matrix3x4
	m1.SetIdentity()
	m2.SetIdentity()

	m3 := m1.MulMatrix3x4(&m2)

	if !RealEqual(m3[0], 1.0) || !RealEqual(m3[3], 0.0) || !RealEqual(m3[6], 0.0) || !RealEqual(m3[9], 0.0) ||
    !RealEqual(m3[1], 0.0) || !RealEqual(m3[4], 1.0) || !RealEqual(m3[7], 0.0) || !RealEqual(m3[10], 0.0) ||
    !RealEqual(m3[2], 0.0) || !RealEqual(m3[5], 0.0) || !RealEqual(m3[8], 1.0) || !RealEqual(m3[11], 0.0) {
		t.Errorf("Multiplication of identities does not yield identity:\n\t%v", m3)
	}
}

func TestMat3Multiplications(t *testing.T) {
  var m1 Matrix3 = Matrix3{
    0.6, 0.2, 0.3,
		0.2, 0.7, 0.5,
		0.3, 0.5, 0.7,
  }

  m1Invert := m1.Invert()
  m3 := m1.MulMatrix3(&m1Invert)

  if !RealEqual(m3[0], 1.0) || !RealEqual(m3[3], 0.0) || !RealEqual(m3[6], 0.0) ||
    !RealEqual(m3[1], 0.0) || !RealEqual(m3[4], 1.0) || !RealEqual(m3[7], 0.0) ||
    !RealEqual(m3[2], 0.0) || !RealEqual(m3[5], 0.0) || !RealEqual(m3[8], 1.0) {
		t.Errorf("Multiplication by it's inversion does not yield identity:\n\t%v", m3)
	}
}
