// Copyright 2015, Timothy Bogdala <tdb@animal-machine.com>
// See the LICENSE file for more details.

package cubez

import (
	"math"

	m "github.com/tbogdala/cubez/math"
)

// CollisionPlane represents a plane in space for collisions.
type CollisionPlane struct {
	// Normal is the plane's normal vector
	Normal m.Vector3

	// Offset is the distance of the plane from the origin
	Offset m.Real
}

// CollisionCube represents a cube in space for collisions.
type CollisionCube struct {
	// Body is the RigidBody that is represented by this collision object.
	Body *RigidBody

	// Halfsize holds the cube's half-sizes along each of its local axes.
	HalfSize m.Vector3

	// Offset is the matrix that gives the offset of this primitive from Body.
	Offset m.Matrix3x4

	// transform is calculated by combining the Offset of the primitive with
	// the transform of the Body.
	// NOTE: this is calculated by calling CalculateDerivedData().
	transform m.Matrix3x4
}

/*
==================================================================================================
  COLLISION PLANE
==================================================================================================
*/

// NewCollisionPlane creates a new CollisionPlane object with the
// normal and offset specified.
func NewCollisionPlane(n m.Vector3, o m.Real) *CollisionPlane {
	plane := new(CollisionPlane)
	plane.Normal = n
	plane.Offset = o
	return plane
}

/*
==================================================================================================
  COLLISION CUBE
==================================================================================================
*/

// NewCollisionCube creates a new CollisionCube object with the dimensions specified
// for a given RigidBody. If a RigidBody is not specified, then a new RigidBody
// object is created for the CollisionCube.
func NewCollisionCube(optBody *RigidBody, halfSize m.Vector3) *CollisionCube {
	cube := new(CollisionCube)
	cube.Offset.SetIdentity()
	cube.HalfSize = halfSize
	cube.Body = optBody
	if cube.Body == nil {
		cube.Body = NewRigidBody()
	}
	return cube
}

// GetTransform returns a copy of the transform matrix for the collider object.
func (cube *CollisionCube) GetTransform() m.Matrix3x4 {
	return cube.transform
}

// CalculateDerivedData internal data from public data members.
//
// Constructs a transform matrix based on the RigidBody's transform and the
// collision object's offset.
func (cube *CollisionCube) CalculateDerivedData() {
	transform := cube.Body.GetTransform()
	cube.transform = transform.MulMatrix3x4(&cube.Offset)
}

// CheckAgainstHalfSpace does a collision test on a collision box and a plane representing
// a half-space (i.e. the normal of the plane points out of the half-space).
func (cube *CollisionCube) CheckAgainstHalfSpace(plane *CollisionPlane, existingContacts []*Contact) (bool, []*Contact) {
	// check for an intersection -- if there is none, then we can return
	if !intersectCubeAndHalfSpace(cube, plane) {
		return false, nil
	}

	// Now that we have an intersection, find the points of intersection. This can be
	// done by checking the eight vertices of the cube. If the cube is resting on a plane
	// or and edge it will be reported as four or two contact points.

	// setup an array of vertices
	var mults [8]m.Vector3
	mults[0] = m.Vector3{1.0, 1.0, 1.0}
	mults[1] = m.Vector3{-1.0, 1.0, 1.0}
	mults[2] = m.Vector3{1.0, -1.0, 1.0}
	mults[3] = m.Vector3{-1.0, -1.0, 1.0}
	mults[4] = m.Vector3{1.0, 1.0, -1.0}
	mults[5] = m.Vector3{-1.0, 1.0, -1.0}
	mults[6] = m.Vector3{1.0, -1.0, -1.0}
	mults[7] = m.Vector3{-1.0, -1.0, -1.0}

	contactDetected := false
	contacts := existingContacts
	for _, v := range mults {
		// calculate the position of the vertex
		v.ComponentProduct(&cube.HalfSize)
		vertexPos := cube.transform.MulVector3(&v)

		// calculate the distance from the plane
		vertexDistance := vertexPos.Dot(&plane.Normal)

		// compare it to the plane's distance
		if vertexDistance <= plane.Offset {
			// we have contact
			c := NewContact()

			// the contact point is halfway between the vertex and the plane --
			// we multiply the direction by half the separation distance and
			// add the vertex location.
			c.ContactPoint = plane.Normal
			c.ContactPoint.MulWith(vertexDistance - plane.Offset)
			c.ContactPoint.Add(&vertexPos)
			c.ContactNormal = plane.Normal
			c.Penetration = plane.Offset - vertexDistance
			c.Bodies[0] = cube.Body
			c.Bodies[1] = nil

			contacts = append(contacts, c)
			contactDetected = true

			// FIXME:
			// TODO: c.Friction and c.Restitution set here are test constants
			c.Friction = 0.9
			c.Restitution = 0.1
		}
	}

	return contactDetected, contacts
}

/*
==================================================================================================
  UTILITY
==================================================================================================
*/

// intersectCubeAndHalfSpace tests to see if a cube and plane intersect
func intersectCubeAndHalfSpace(cube *CollisionCube, plane *CollisionPlane) bool {
	// work out the projected radius of the cube onto the plane normal
	projectedRadius := transformToAxis(cube, &plane.Normal)

	// work out how far the box is from the origin
	axis := cube.transform.GetAxis(3)
	cubeDistance := plane.Normal.Dot(&axis) - projectedRadius

	// check for intersection
	return cubeDistance <= plane.Offset
}

func transformToAxis(cube *CollisionCube, axis *m.Vector3) m.Real {
	cubeAxisX := cube.transform.GetAxis(0)
	cubeAxisY := cube.transform.GetAxis(1)
	cubeAxisZ := cube.transform.GetAxis(2)

	return cube.HalfSize[0]*m.Real(math.Abs(float64(axis.Dot(&cubeAxisX)))) +
		cube.HalfSize[1]*m.Real(math.Abs(float64(axis.Dot(&cubeAxisY)))) +
		cube.HalfSize[2]*m.Real(math.Abs(float64(axis.Dot(&cubeAxisZ))))
}
