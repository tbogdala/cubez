// Copyright 2015, Timothy Bogdala <tdb@animal-machine.com>
// See the LICENSE file for more details.

package cubez

import (
	"math"
	m "github.com/tbogdala/cubez/math"
)

// Collider is an interface for collision primitive objects to make calculating collisions
// amongst a heterogenous set of objects easier.

type Collider interface {
	CalculateDerivedData()
	GetBody() *RigidBody
	CheckAgainstHalfSpace(plane *CollisionPlane, existingContacts []*Contact) (bool, []*Contact)
	CheckAgainstSphere(sphere *CollisionSphere, existingContacts []*Contact) (bool, []*Contact)
	CheckAgainstCube(secondCube *CollisionCube, existingContacts []*Contact) (bool, []*Contact)
}

// CollisionPlane represents a plane in space for collisions but doesn't
// have an associated rigid body and is considered to be infinite.
// It's primarily useful for rerepresenting immovable world geometry like
// a giant ground plane.
type CollisionPlane struct {
	// Normal is the plane's normal vector
	Normal m.Vector3

	// Offset is the distance of the plane from the origin
	Offset m.Real
}

// CollisionCube is a rigid body that can be considered an axis-alligned cube
// for contact collision.
type CollisionCube struct {
	// Body is the RigidBody that is represented by this collision object.
	Body *RigidBody

	// Offset is the matrix that gives the offset of this primitive from Body.
	Offset m.Matrix3x4

	// transform is calculated by combining the Offset of the primitive with
	// the transform of the Body.
	// NOTE: this is calculated by calling CalculateDerivedData().
	transform m.Matrix3x4

	// Halfsize holds the cube's half-sizes along each of its local axes.
	HalfSize m.Vector3
}

// CollisionSphere is a rigid body that can be considered a sphere
// for collision detection.
type CollisionSphere struct {
	// Body is the RigidBody that is represented by this collision object.
	Body *RigidBody

	// Offset is the matrix that gives the offset of this primitive from Body.
	Offset m.Matrix3x4

	// transform is calculated by combining the Offset of the primitive with
	// the transform of the Body.
	// NOTE: this is calculated by calling CalculateDerivedData().
	transform m.Matrix3x4

	// Radius is the radius of the sphere.
	Radius m.Real
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

// CalculateDerivedData currently doesn't do anything for planes.
func (s *CollisionPlane) CalculateDerivedData() {
}

/*
==================================================================================================
  COLLISION SPHERE
==================================================================================================
*/

// NewCollisionSphere creates a new CollisionSphere object with the radius specified
// for a given RigidBody. If a RigidBody is not specified, then a new RigidBody
// object is created for the new collider object.
func NewCollisionSphere(optBody *RigidBody, radius m.Real) *CollisionSphere {
	s := new(CollisionSphere)
	s.Offset.SetIdentity()
	s.Radius = radius
	s.Body = optBody
	if s.Body == nil {
		s.Body = NewRigidBody()
	}
	return s
}

// GetTransform returns a copy of the transform matrix for the collider object.
func (s *CollisionSphere) GetTransform() m.Matrix3x4 {
	return s.transform
}

// GetBody returns the rigid body associated with the sphere.
func (s *CollisionSphere) GetBody() *RigidBody {
	return s.Body
}

// CalculateDerivedData internal data from public data members.
//
// Constructs a transform matrix based on the RigidBody's transform and the
// collision object's offset.
func (s *CollisionSphere) CalculateDerivedData() {
	transform := s.Body.GetTransform()
	s.transform = transform.MulMatrix3x4(&s.Offset)
}

// CheckAgainstHalfSpace does a collision test on a collision sphere and a plane representing
// a half-space (i.e. the normal of the plane points out of the half-space).
func (s *CollisionSphere) CheckAgainstHalfSpace(plane *CollisionPlane, existingContacts []*Contact) (bool, []*Contact) {
	// work out the distance from the origin
	positionAxis := s.transform.GetAxis(3)
	distance := plane.Normal.Dot(&positionAxis) - s.Radius

	// check for intersection
	if distance <= plane.Offset == false {
		return false, existingContacts
	}

	c := NewContact()
	c.ContactPoint = plane.Normal
	c.ContactPoint.MulWith(distance + s.Radius*-1.0)
	c.ContactPoint.Add(&positionAxis)
	c.ContactNormal = plane.Normal
	c.Penetration = -distance
	c.Bodies[0] = s.Body
	c.Bodies[1] = nil

	// FIXME:
	// TODO: c.Friction and c.Restitution set here are test constants
	c.Friction = 0.9
	c.Restitution = 0.1

	contacts := append(existingContacts, c)

	return true, contacts
}

// CheckAgainstCube checks the sphere against collision with a cube.
func (s *CollisionSphere) CheckAgainstCube(cube *CollisionCube, existingContacts []*Contact) (bool, []*Contact) {
	// FIXME: not implemented
	return false, existingContacts
}

// CheckAgainstCube checks the sphere against collision with a sphere.
func (s *CollisionSphere) CheckAgainstSphere(cube *CollisionSphere, existingContacts []*Contact) (bool, []*Contact) {
	// FIXME: not implemented
	return false, existingContacts
}

/*
==================================================================================================
  COLLISION CUBE
==================================================================================================
*/

// NewCollisionCube creates a new CollisionCube object with the dimensions specified
// for a given RigidBody. If a RigidBody is not specified, then a new RigidBody
// object is created for the new collider object.
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

// GetBody returns the rigid body associated with the cube.
func (cube *CollisionCube) GetBody() *RigidBody {
	return cube.Body
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
		return false, existingContacts
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

// CheckAgainstSphere checks the cube against a sphere to see if there's a collision.
func (cube *CollisionCube) CheckAgainstSphere(sphere *CollisionSphere, existingContacts []*Contact) (bool, []*Contact) {
	// transform the center of the sphere into cube coordinates
	position := sphere.transform.GetAxis(3)
	relCenter := cube.transform.TransformInverse(&position)

	// check to see if we can exclude contact
	if m.RealAbs(relCenter[0])-sphere.Radius > cube.HalfSize[0] ||
		m.RealAbs(relCenter[1])-sphere.Radius > cube.HalfSize[1] ||
		m.RealAbs(relCenter[2])-sphere.Radius > cube.HalfSize[2] {
		return false, existingContacts
	}

	var closestPoint m.Vector3

	// clamp the coordinates to the box
	for i := 0; i < 3; i++ {
		dist := relCenter[i]
		if dist > cube.HalfSize[i] {
			dist = cube.HalfSize[i]
		} else if dist < -cube.HalfSize[i] {
			dist = -cube.HalfSize[i]
		}
		closestPoint[i] = dist
	}

	// check to see if we're in contact
	distCheck := closestPoint
	distCheck.Sub(&relCenter)
	dist := distCheck.Dot(&distCheck)
	if dist > sphere.Radius*sphere.Radius {
		return false, existingContacts
	}

	// transform the contact point
	closestPointWorld := cube.transform.MulVector3(&closestPoint)

	// we have contact
	c := NewContact()
	c.ContactPoint = closestPointWorld

	closestPointWorld.Sub(&position)
	c.ContactNormal = closestPointWorld
	c.ContactNormal.Normalize()

	c.Penetration = sphere.Radius - m.RealSqrt(dist)
	c.Bodies[0] = cube.Body
	c.Bodies[1] = sphere.Body

	contacts := append(existingContacts, c)

	// FIXME:
	// TODO: c.Friction and c.Restitution set here are test constants
	c.Friction = 0.9
	c.Restitution = 0.1

	return true, contacts
}

// penetrationOnAxis checks if the two boxes overlap along a given axis and
// returns the amount of overlap.
func penetrationOnAxis(one *CollisionCube, two *CollisionCube, axis *m.Vector3, toCenter *m.Vector3) m.Real {
	// project the half-size of one onto axis
	oneProject := transformToAxis(one, axis)
	twoProject := transformToAxis(two, axis)

	// Project this onto the axis
	distance := m.RealAbs(toCenter.Dot(axis))

	// Return the overlap (i.e. positive indicates
	// overlap, negative indicates separation).
	return oneProject + twoProject - distance
}

func tryAxis(one *CollisionCube, two *CollisionCube, axis m.Vector3, toCenter *m.Vector3,
	index int, smallestPenetration m.Real, smallestCase int) (bool, m.Real, int) {
	// make sure we have a normalized axis, and don't check almost parallel axes
	if axis.SquareMagnitude() < m.Epsilon {
		return true, smallestPenetration, smallestCase
	}

	axis.Normalize()

	penetration := penetrationOnAxis(one, two, &axis, toCenter)
	if penetration < 0 {
		return false, smallestPenetration, smallestCase
	}

	if penetration < smallestPenetration {
		return true, penetration, index
	}

	return true, smallestPenetration, smallestCase
}

// fillPointFaceBoxBox is called when we know that a vertex from
// box two is in contact with box one.
func fillPointFaceBoxBox(one *CollisionCube, two *CollisionCube, toCenter *m.Vector3,
	best int, pen m.Real, existingContacts []*Contact) []*Contact {
	// We know which axis the collision is on (i.e. best),
	// but we need to work out which of the two faces on this axis.
	normal := one.transform.GetAxis(best)
	if normal.Dot(toCenter) > 0 {
		normal.MulWith(-1.0)
	}

	// Work out which vertex of box two we're colliding with.
	v := two.HalfSize
	if twoA0 := two.transform.GetAxis(0); twoA0.Dot(&normal) < 0 {
		v[0] = -v[0]
	}
	if twoA1 := two.transform.GetAxis(1); twoA1.Dot(&normal) < 0 {
		v[1] = -v[1]
	}
	if twoA2 := two.transform.GetAxis(2); twoA2.Dot(&normal) < 0 {
		v[2] = -v[2]
	}

	c := NewContact()
	c.ContactNormal = normal
	c.Penetration = pen
	c.ContactPoint = two.transform.MulVector3(&v)
	c.Bodies[0] = one.Body
	c.Bodies[1] = two.Body

	// FIXME:
	// TODO: c.Friction and c.Restitution set here are test constants
	c.Friction = 0.9
	c.Restitution = 0.1

	contacts := append(existingContacts, c)

	return contacts
}

func contactPoint(pOne *m.Vector3, dOne *m.Vector3, oneSize m.Real,
	pTwo *m.Vector3, dTwo *m.Vector3, twoSize m.Real, useOne bool) m.Vector3 {
	// If useOne is true, and the contact point is outside
	// the edge (in the case of an edge-face contact) then
	// we use one's midpoint, otherwise we use two's.
	//Vector3 toSt, cOne, cTwo;
	//real dpStaOne, dpStaTwo, dpOneTwo, smOne, smTwo;
	//real denom, mua, mub;

	smOne := dOne.SquareMagnitude()
	smTwo := dTwo.SquareMagnitude()
	dpOneTwo := dTwo.Dot(dOne)

	toSt := *pOne
	toSt.Sub(pTwo)
	dpStaOne := dOne.Dot(&toSt)
	dpStaTwo := dTwo.Dot(&toSt)

	denom := smOne*smTwo - dpOneTwo*dpOneTwo

	// Zero denominator indicates parrallel lines
	if m.RealAbs(denom) < m.Epsilon {
		if useOne {
			return *pOne
		} else {
			return *pTwo
		}
	}

	mua := (dpOneTwo*dpStaTwo - smTwo*dpStaOne) / denom
	mub := (smOne*dpStaTwo - dpOneTwo*dpStaOne) / denom

	// If either of the edges has the nearest point out
	// of bounds, then the edges aren't crossed, we have
	// an edge-face contact. Our point is on the edge, which
	// we know from the useOne parameter.
	if mua > oneSize || mua < -oneSize || mub > twoSize || mub < -twoSize {
		if useOne {
			return *pOne
		} else {
			return *pTwo
		}
	} else {
		cOne := *dOne
		cOne.MulWith(mua)
		cOne.Add(pOne)

		cTwo := *dTwo
		cTwo.MulWith(mub)
		cTwo.Add(pTwo)

		cOne.MulWith(0.5)
		cTwo.MulWith(0.5)
		cOne.Add(&cTwo)
		return cOne
	}
}

func (cube *CollisionCube) CheckAgainstCube(secondCube *CollisionCube, existingContacts []*Contact) (bool, []*Contact) {
	// find the vector between two vectors
	toCenter := secondCube.transform.GetAxis(3)
	oneAxis3 := cube.transform.GetAxis(3)
	toCenter.Sub(&oneAxis3)

	var ret bool
	pen := m.MaxValue
	var best int = 0xffffff

	// Now we check each axis, returning if it gives a separating axis.
	// Keep track of the smallest penetration axis.
	for i := 0; i <= 2; i++ {
		ret, pen, best = tryAxis(cube, secondCube, cube.transform.GetAxis(i), &toCenter, i, pen, best)
		if ret == false {
			return false, existingContacts
		}
	}
	for i := 0; i <= 2; i++ {
		ret, pen, best = tryAxis(cube, secondCube, secondCube.transform.GetAxis(i), &toCenter, i+3, pen, best)
		if ret == false {
			return false, existingContacts
		}
	}

	// Store the best axis-major, in case we run into almost parallel edge collisions later
	bestSingleAxis := best

	for i := 0; i <= 2; i++ {
		a1 := cube.transform.GetAxis(i)
		a2 := secondCube.transform.GetAxis(0)
		cross := a1.Cross(&a2)
		ret, pen, best = tryAxis(cube, secondCube, cross, &toCenter, (i*3)+6, pen, best)
		if ret == false {
			return false, existingContacts
		}
		a1 = cube.transform.GetAxis(i)
		a2 = secondCube.transform.GetAxis(1)
		cross = a1.Cross(&a2)
		ret, pen, best = tryAxis(cube, secondCube, cross, &toCenter, (i*3)+7, pen, best)
		if ret == false {
			return false, existingContacts
		}
		a1 = cube.transform.GetAxis(i)
		a2 = secondCube.transform.GetAxis(2)
		cross = a1.Cross(&a2)
		ret, pen, best = tryAxis(cube, secondCube, cross, &toCenter, (i*3)+8, pen, best)
		if ret == false {
			return false, existingContacts
		}
	}

	// We now know there's a collision, and we know which of the axes gave
	// the smallest penetration. We now can deal with it in different ways
	// depending on the case.
	if best < 3 {
		// We've got a vertex of box two on a face of box one.
		return true, fillPointFaceBoxBox(cube, secondCube, &toCenter, best, pen, existingContacts)
	} else if best < 6 {
		// We've got a vertex of box one on a face of box two.
		// We use the same algorithm as above, but swap around
		// one and two (and therefore also the vector between their
		// centres).
		newCenter := toCenter
		newCenter.MulWith(-1.0)
		return true, fillPointFaceBoxBox(secondCube, cube, &newCenter, best-3, pen, existingContacts)
	} else {
		// We've got an edge-edge contact. Find out which axes
		best -= 6
		oneAxisIndex := best / 3
		twoAxisIndex := best % 3
		oneAxis := cube.transform.GetAxis(oneAxisIndex)
		twoAxis := secondCube.transform.GetAxis(twoAxisIndex)
		axis := oneAxis.Cross(&twoAxis)
		axis.Normalize()

		// The axis should point from box one to box two.
		if axis.Dot(&toCenter) > 0 {
			axis.MulWith(-1.0)
		}

		// We have the axes, but not the edges: each axis has 4 edges parallel
		// to it, we need to find which of the 4 for each object. We do
		// that by finding the point in the centre of the edge. We know
		// its component in the direction of the box's collision axis is zero
		// (its a mid-point) and we determine which of the extremes in each
		// of the other axes is closest.
		ptOnOneEdge := cube.HalfSize
		ptOnTwoEdge := secondCube.HalfSize
		for i := 0; i < 3; i++ {
			if i == oneAxisIndex {
				ptOnOneEdge[i] = 0
			} else if oneAxis := cube.transform.GetAxis(i); oneAxis.Dot(&axis) > 0 {
				ptOnOneEdge[i] = -ptOnOneEdge[i]
			}

			if i == twoAxisIndex {
				ptOnTwoEdge[i] = 0
			} else if twoAxis := secondCube.transform.GetAxis(i); twoAxis.Dot(&axis) < 0 {
				ptOnTwoEdge[i] = -ptOnTwoEdge[i]
			}
		}

		// Move them into world coordinates (they are already oriented
		// correctly, since they have been derived from the axes).
		ptOnOneEdge = cube.transform.MulVector3(&ptOnOneEdge)
		ptOnTwoEdge = secondCube.transform.MulVector3(&ptOnTwoEdge)

		// So we have a point and a direction for the colliding edges.
		// We need to find out point of closest approach of the two
		// line-segments.
		useOne := false
		if bestSingleAxis > 2 {
			useOne = true
		}
		contactVertex := contactPoint(&ptOnOneEdge, &oneAxis, cube.HalfSize[oneAxisIndex],
			&ptOnTwoEdge, &twoAxis, secondCube.HalfSize[twoAxisIndex], useOne)

		// finally ... create a new contact
		c := NewContact()
		c.ContactNormal = axis
		c.Penetration = pen
		c.ContactPoint = contactVertex
		c.Bodies[0] = cube.Body
		c.Bodies[1] = secondCube.Body

		// FIXME:
		// TODO: c.Friction and c.Restitution set here are test constants
		c.Friction = 0.9
		c.Restitution = 0.1

		contacts := append(existingContacts, c)
		return true, contacts
	}
	return false, existingContacts
}

/*
==================================================================================================
  UTILITY
==================================================================================================
*/

// CheckForCollisions will check one collider primitive against another and update the contact slice
// if there were any contacts (as well as returning a bool indicating if contacts were found).
func CheckForCollisions(one Collider, two Collider, existingContacts []*Contact) (bool, []*Contact) {
	switch two.(type) {
	case *CollisionSphere:
		otherSphere, ok := two.(*CollisionSphere)
		if ok {
    	return one.CheckAgainstSphere(otherSphere, existingContacts)
		} else {
			return false, existingContacts
		}
	case *CollisionCube:
		otherCube, ok := two.(*CollisionCube)
		if ok {
    	return one.CheckAgainstCube(otherCube, existingContacts)
		} else {
			return false, existingContacts
		}
	}

	// this is reached if we dont have a supported Check* function in the interface
	// for the primitive type.
	return false, existingContacts
}

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
