// Copyright 2015, Timothy Bogdala <tdb@animal-machine.com>
// See the LICENSE file for more details.

package math

import (
	"testing"
)

/* ================================ VECTOR 3 ================================================ */

func TestVector3Add(t *testing.T) {
	v1 := Vector3{1.0, 2.5, 3.75}
	v2 := Vector3{0.0, 1.0, 7.0}

	v1.Add(&v2)

	if !RealEqual(v2[0], 0.0) || !RealEqual(v2[1], 1.0) || !RealEqual(v2[2], 7.0) {
		t.Errorf("Add modified an original vector while calculating addition: %v", v2)
	}
	if !RealEqual(v1[0], 1.0) || !RealEqual(v1[1], 3.5) || !RealEqual(v1[2], 10.75) {
		t.Errorf("Vector Add not adding properly: %v", v1)
	}

	v2.Add(&v1)

	if !RealEqual(v1[0], 1.0) || !RealEqual(v1[1], 3.5) || !RealEqual(v1[2], 10.75) {
		t.Errorf("Add modified an original vector while calculating addition: %v", v2)
	}
	if !RealEqual(v2[0], 1.0) || !RealEqual(v2[1], 4.5) || !RealEqual(v2[2], 17.75) {
		t.Errorf("vVector Add is somehow not commutative: %v", v2)
	}
}

func TestVector3AddScaled(t *testing.T) {
	v1 := Vector3{1.0, 2.5, 3.75}
	v2 := Vector3{0.0, 1.0, 7.0}
	var scale Real = 3.0

	v1.AddScaled(&v2, scale)

	if !RealEqual(v2[0], 0.0) || !RealEqual(v2[1], 1.0) || !RealEqual(v2[2], 7.0) {
		t.Errorf("AddScaled modified an original vector while calculating AddScaled: %v", v2)
	}
	if !RealEqual(v1[0], 1.0) || !RealEqual(v1[1], 5.5) || !RealEqual(v1[2], 24.75) {
		t.Errorf("AddScaled not adding a scaled vector properly: %v", v1)
	}
}

func TestVector3Clear(t *testing.T) {
	v1 := Vector3{1.0, 2.5, 3.75}

	v1.Clear()

	if !RealEqual(v1[0], 0.0) || !RealEqual(v1[1], 0.0) || !RealEqual(v1[2], 0.0) {
		t.Errorf("Clear isn't clearing a vector properly: %v", v1)
	}
}

func TestVector3ComponentProduct(t *testing.T) {
	v1 := Vector3{1.0, 2.5, 3.5}
	v2 := Vector3{0.0, 1.0, 7.0}

	v1.ComponentProduct(&v2)

	if !RealEqual(v2[0], 0.0) || !RealEqual(v2[1], 1.0) || !RealEqual(v2[2], 7.0) {
		t.Errorf("ComponentProduct modified an original vector while calculating a cross product: %v", v2)
	}
	if !RealEqual(v1[0], 0.0) || !RealEqual(v1[1], 2.5) || !RealEqual(v1[2], 24.5) {
		t.Errorf("ComponentProduct isn't multiplying a vector properly: %v", v1)
	}
}

func TestVector3Cross(t *testing.T) {
	v1 := Vector3{1.0, 2.0, 3.0}
	v2 := Vector3{10.0, 11.0, 12.0}

	v3 := v1.Cross(&v2)

	if !RealEqual(v1[0], 1.0) || !RealEqual(v1[1], 2.0) || !RealEqual(v1[2], 3.0) {
		t.Errorf("Cross modified an original vector while calculating a cross product: %v", v1)
	}
	if !RealEqual(v2[0], 10.0) || !RealEqual(v2[1], 11.0) || !RealEqual(v2[2], 12.0) {
		t.Errorf("Cross modified an original vector while calculating a cross product: %v", v1)
	}
	if !RealEqual(v3[0], -9.0) || !RealEqual(v3[1], 18.0) || !RealEqual(v3[2], -9.0) {
		t.Errorf("Cross isn't calculating a cross product properly: %v", v3)
	}
}

func TestVector3Dot(t *testing.T) {
	v1 := Vector3{-1.0, -5.0, -7.0}
	v2 := Vector3{10.0, 20.0, 30.0}

	dot := v1.Dot(&v2)

	if !RealEqual(v1[0], -1.0) || !RealEqual(v1[1], -5.0) || !RealEqual(v1[2], -7.0) {
		t.Errorf("Dot modified an original vector while calculating a dot product: %v", v1)
	}
	if !RealEqual(v2[0], 10.0) || !RealEqual(v2[1], 20.0) || !RealEqual(v2[2], 30.0) {
		t.Errorf("Dot modified an original vector while calculating a dot product: %v", v1)
	}
	if !RealEqual(dot, -320.0) {
		t.Errorf("Dot isn't calculating a dot product properly: %v", dot)
	}
}

func TestVector3Magnitude(t *testing.T) {
	v1 := Vector3{2.0, -5.0, 4.0}

	mag := v1.Magnitude()

	if !RealEqual(v1[0], 2.0) || !RealEqual(v1[1], -5.0) || !RealEqual(v1[2], 4.0) {
		t.Errorf("Magnitude modified an original vector while calculating a magnitude: %v", v1)
	}
	if !RealEqual(mag, 6.708203932499369) {
		t.Errorf("Magnitude isn't calculating a magnitude properly: %v", mag)
	}
}

func TestVector3SquareMagnitude(t *testing.T) {
	v1 := Vector3{2.0, -5.0, 4.0}

	sqmag := v1.SquareMagnitude()

	if !RealEqual(v1[0], 2.0) || !RealEqual(v1[1], -5.0) || !RealEqual(v1[2], 4.0) {
		t.Errorf("Magnitude modified an original vector while calculating a magnitude: %v", v1)
	}
	if !RealEqual(sqmag, 4.0+25+16) {
		t.Errorf("Magnitude isn't calculating a squared magnitude properly: %v", sqmag)
	}
}

func TestVector3MulWith(t *testing.T) {
	v1 := Vector3{1.0, 2.5, 3.5}

	v1.MulWith(10.0)

	if !RealEqual(v1[0], 10.0) || !RealEqual(v1[1], 25.0) || !RealEqual(v1[2], 35.0) {
		t.Errorf("MulWith isn't multiplying a vector properly: %v", v1)
	}
}

func TestVector3Normalize(t *testing.T) {
	v1 := Vector3{1.0, 2.5, 3.5}
	v1.Normalize()
	if !RealEqual(v1.Magnitude(), 1.0) {
		t.Errorf("Normalize isn't normalizing a vector properly: %v (len:%v)", v1, v1.Magnitude())
	}
}

func TestVector3Set(t *testing.T) {
	v1 := Vector3{-1.0, -5.0, -7.0}
	v2 := Vector3{10.0, 20.0, 30.0}

	v1.Set(&v2)

	if !RealEqual(v2[0], 10.0) || !RealEqual(v2[1], 20.0) || !RealEqual(v2[2], 30.0) {
		t.Errorf("Set modified an original vector while setting another vector: %v", v2)
	}
	if !RealEqual(v1[0], 10.0) || !RealEqual(v1[1], 20.0) || !RealEqual(v1[2], 30.0) {
		t.Errorf("Set did not set vector members properly: %v", v1)
	}
}

func TestVector3Sub(t *testing.T) {
	v1 := Vector3{-1.0, -5.0, -7.0}
	v2 := Vector3{10.0, 20.0, 30.0}

	v1.Sub(&v2)

	if !RealEqual(v2[0], 10.0) || !RealEqual(v2[1], 20.0) || !RealEqual(v2[2], 30.0) {
		t.Errorf("Sub modified an original vector while subtracting another vector: %v", v2)
	}
	if !RealEqual(v1[0], -11.0) || !RealEqual(v1[1], -25.0) || !RealEqual(v1[2], -37.0) {
		t.Errorf("Sub did not subtract the vector properly: %v", v1)
	}
}

/* ================================ VECTOR 4 ================================================ */

func TestVector4MulWith(t *testing.T) {
	v1 := Vector4{1.0, 2.5, 3.5, 98.7}

	v1.MulWith(10.0)

	if !RealEqual(v1[0], 10.0) || !RealEqual(v1[1], 25.0) || !RealEqual(v1[2], 35.0) || !RealEqual(v1[3], 987.0) {
		t.Errorf("MulWith isn't multiplying a vector properly: %v", v1)
	}
}

/* ================================ HOW DOES GO WORK ============================================= */

func TestVectorGoCopies(t *testing.T) {
	var v1 Vector3 = Vector3{1.0, 2.5, 3.5}
	var vArray [2]Vector3

	// This test illustrates that Go makes a copy of the float array here and that subsequent
	// operations on the new copy do not modify the original.
	vArray[0] = v1
	vArray[0].MulWith(2.0)

	if !RealEqual(vArray[0][0], 2.0) || !RealEqual(vArray[0][1], 5.0) || !RealEqual(vArray[0][2], 7.0) {
		t.Errorf("MulWith isn't multiplying a vector properly: %v", v1)
	}

	if !RealEqual(v1[0], 1.0) || !RealEqual(v1[1], 2.5) || !RealEqual(v1[2], 3.5) {
		t.Errorf("The initial vector was modified in the multiplication after a copy: %v", v1)
	}
}
