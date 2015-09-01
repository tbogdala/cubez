// Copyright 2015, Timothy Bogdala <tdb@animal-machine.com>
// See the LICENSE file for more details.

package cubez

import (
	"math"

	m "github.com/tbogdala/cubez/math"
)

// a few internal epsilon values
const (
	velocityEpsilon m.Real = 0.01
	positionEpsilon m.Real = 0.01
)

// Contact holds all of the data associated with a contact between two collider primitives.
type Contact struct {
	// Bodies holds 1 or 2 bodies involved in the contact; second body can be nil
	Bodies [2]*RigidBody

	// Friction holds the lateral friction coefficient at the contact
	Friction m.Real

	// Restitution holdes the normal restitution coefficient at the contact
	Restitution m.Real

	// ContactPoint is the position of the contact in World Space
	ContactPoint m.Vector3

	// ContactNormal is the direction of the contact in World Space
	ContactNormal m.Vector3

	// Penetration is the depth of penetration at the contact point. If
	// both Bodies are set, then this should be midway between the
	// inter-penetrating points.
	Penetration m.Real

	// contactToWorld is a transform matrix that converts coordiantes in the contact's
	// frame of reference to World coordinates. The columns are orthornomal vectors.
	contactToWorld m.Matrix3

	// relativeContactPosition holds the World Space position of the contact point
	// relative to the center of each Body.
	relativeContactPosition [2]m.Vector3

	// contactVelocity holds the closing velocity at the point of contact.
	contactVelocity m.Vector3

	// desiredDeltaVelocity holds the required change in velocity for this contact to be resolved.
	desiredDeltaVelocity m.Real
}

// NewContact returns a new Contact object.
func NewContact() *Contact {
	c := new(Contact)
	return c
}

func (c *Contact) calculateInternals(duration m.Real) {
	// make sure that if there's only one body that it's in the first spot
	if c.Bodies[0] == nil {
		c.ContactNormal.MulWith(-1.0)
		c.Bodies[0] = c.Bodies[1]
		c.Bodies[1] = nil
	}

	// make the set of axis at the contact point
	c.calculateContactBasis()

	// store the relative position of the contact to each body
	c.relativeContactPosition[0].Set(&c.ContactPoint)
	c.relativeContactPosition[0].Sub(&c.Bodies[0].Position)
	c.contactVelocity = c.calculateLocalVelocity(0, duration)

	if c.Bodies[1] != nil {
		c.relativeContactPosition[1].Set(&c.ContactPoint)
		c.relativeContactPosition[1].Sub(&c.Bodies[1].Position)

		contactVelocity1 := c.calculateLocalVelocity(1, duration)
		c.contactVelocity.Sub(&contactVelocity1)
	}

	// calculate the desired change in velocity for resolution
	c.calculateDesiredDeltaVelocity(duration)
}

func (c *Contact) calculateDesiredDeltaVelocity(duration m.Real) {
	const velocityLimit m.Real = 0.25
	var velocityFromAcc m.Real

	// calculate the acceleration induced velocity accumlated this frame
	var tempVelocity m.Vector3
	if c.Bodies[0].IsAwake {
		tempVelocity = c.Bodies[0].GetLastFrameAccelleration()
		tempVelocity.MulWith(duration)
		velocityFromAcc += tempVelocity.Dot(&c.ContactNormal)
	}
	if c.Bodies[1] != nil && c.Bodies[1].IsAwake {
		tempVelocity = c.Bodies[1].GetLastFrameAccelleration()
		tempVelocity.MulWith(duration)
		velocityFromAcc -= tempVelocity.Dot(&c.ContactNormal)
	}

	// if the velocity is very slow, limit the restitution
	restitution := c.Restitution
	if m.Real(math.Abs(float64(c.contactVelocity[0]))) < velocityLimit {
		restitution = 0.0
	}

	// combine the bounce velocity with the removed acceleration velocity
	c.desiredDeltaVelocity = -c.contactVelocity[0] - restitution*(c.contactVelocity[0]-velocityFromAcc)
}

// Constructs an arbitrary orthonormal basis for the contact. It's stored
// as a 3x3 matrix where each column is a vector for an axis. The x axis
// is based off of the contact normal and the y and z axis will be generated
// so that they are at right angles to it.
func (c *Contact) calculateContactBasis() {
	var contactTangentY m.Vector3
	var contactTangentZ m.Vector3

	absContactNormalX := m.Real(math.Abs(float64(c.ContactNormal[0])))
	absContactNormalY := m.Real(math.Abs(float64(c.ContactNormal[1])))

	// check whether the z axis is nearer to the x or y axis
	if absContactNormalX > absContactNormalY {
		// generate a scaling factor to ensure results are normalized
		s := m.Real(1.0) / m.Real(math.Sqrt(float64(
			c.ContactNormal[2]*c.ContactNormal[2]+
				c.ContactNormal[0]*c.ContactNormal[0])))

		// the new x axis is at right angles to the world y axis
		contactTangentY[0] = c.ContactNormal[2] * s
		contactTangentY[1] = 0
		contactTangentY[2] = c.ContactNormal[0] * -s

		// the new y axis is at right angles to the new x and z axes
		contactTangentZ[0] = c.ContactNormal[1] * contactTangentY[0]
		contactTangentZ[1] = c.ContactNormal[2]*contactTangentY[0] - c.ContactNormal[0]*contactTangentY[2]
		contactTangentZ[2] = -c.ContactNormal[1] * contactTangentY[0]
	} else {
		// generate a scaling factor to ensure results are normalized
		s := m.Real(1.0) / m.Real(math.Sqrt(float64(
			c.ContactNormal[2]*c.ContactNormal[2]+
				c.ContactNormal[1]*c.ContactNormal[1])))

		// the new x axis is at right angles to the world y axis
		contactTangentY[0] = 0
		contactTangentY[1] = -c.ContactNormal[2] * s
		contactTangentY[2] = c.ContactNormal[1] * s

		// the new y axis is at right angles to the new x and z axes
		contactTangentZ[0] = c.ContactNormal[1]*contactTangentY[2] - c.ContactNormal[2]*contactTangentY[1]
		contactTangentZ[1] = -c.ContactNormal[0] * contactTangentY[2]
		contactTangentZ[2] = c.ContactNormal[0] * contactTangentY[1]
	}

	// now set the contactToWorld matrix based off of these three vectors
	c.contactToWorld.SetComponents(&c.ContactNormal, &contactTangentY, &contactTangentZ)
}

func (c *Contact) calculateLocalVelocity(bodyIndex int, duration m.Real) m.Vector3 {
	body := c.Bodies[bodyIndex]

	// work out the velocity of the contact point
	velocity := body.Rotation.Cross(&c.relativeContactPosition[bodyIndex])
	velocity.Add(&body.Velocity)

	// turn the velocity into contact coordinates
	contactVelocity := c.contactToWorld.MulVector3(&velocity)

	// calculate the amount of velocity that is due to forces without reactions
	accVelocity := body.GetLastFrameAccelleration()
	accVelocity.MulWith(duration)
	accVelocity = c.contactToWorld.MulVector3(&accVelocity)

	// we ignore any component of acceleration in the contact normal direction
	accVelocity[0] = 0.0

	// add the planar velocity -- if there's enough friction they will
	// be removed during the velocity resolution
	contactVelocity.Add(&accVelocity)

	return contactVelocity
}

func (c *Contact) matchAwakeState() {
	// solo body collisions don't trigger a wake-up
	if c.Bodies[1] == nil {
		return
	}

	b0Awake := c.Bodies[0].IsAwake
	b1Awake := c.Bodies[1].IsAwake

	// wake up only the sleeping one
	if (b0Awake || b1Awake) && !(b0Awake && b1Awake) {
		if b0Awake {
			c.Bodies[1].SetAwake(true)
		} else {
			c.Bodies[0].SetAwake(true)
		}
	}
}

// ResolveContacts results a set of contacts for both penetration and velocity.
//
// NOTE: Contacts that cannot interact with each other should be passed to
// separate calls of ResolveContacts for performance reasons.
func ResolveContacts(maxIterations int, contacts []*Contact, duration m.Real) {
	// start off with some sanity checks
	if duration <= 0.0 || contacts == nil || len(contacts) == 0 {
		return
	}

	// prepares the contacts for processing
	prepareContacts(contacts, duration)

	// resolve the interpenetration problems with the contacts
	adjustPositions(maxIterations, contacts, duration)

	// resolve the velocity problems with the contacts
	adjustVelocities(maxIterations, contacts, duration)
}

func prepareContacts(contacts []*Contact, duration m.Real) {
	for _, e := range contacts {
		e.calculateInternals(duration)
	}
}

func adjustPositions(maxIterations int, contacts []*Contact, duration m.Real) {
	// iteratively resolve interpenetrations in order of severity
	iterationsUsed := 0
	for iterationsUsed < maxIterations {
		// find the biggest penetration
		max := positionEpsilon
		index := len(contacts)
		for i, c := range contacts {
			if c.Penetration > max {
				max = c.Penetration
				index = i
			}
		}
		if index == len(contacts) {
			break
		}
		contact := contacts[index]

		// match the awake state at the contact
		contact.matchAwakeState()

		// resolve the penetration
		linearChange, angularChange := contact.applyPositionChange(max)

		// again this action may have changed the penetration of other bodies,
		// so we update contacts
		for _, c := range contacts {
			for b := 0; b < 2; b++ {
				if c.Bodies[b] != nil {
					// check for a match with each body in the newly resolved contact
					for d := 0; d < 2; d++ {
						if c.Bodies[b] == contact.Bodies[d] {
							deltaPosition := angularChange[d].Cross(&c.relativeContactPosition[b])
							deltaPosition.Add(&linearChange[d])

							// the sign of the change is positive if we're dealing with the second body
							// in a contact and negative otherwise (because we're subtracting the resolution).
							var sign m.Real = 1.0
							if b == 0 {
								sign = -1.0
							}
							c.Penetration += deltaPosition.Dot(&c.ContactNormal) * sign
						}
					} // d
				}
			} // b
		} // i,c

		iterationsUsed++
	}
}

func (c *Contact) applyPositionChange(penetration m.Real) (linearChange, angularChange [2]m.Vector3) {
	const angularLimit m.Real = 0.2
	var angularInertia, linearInertia, angularMove, linearMove [2]m.Real
	var totalInertia m.Real

	// we need to work out the inertia of each object in the direction
	// of the contact normal due to angular inertia only
	for i := 0; i < 2; i++ {
		body := c.Bodies[i]
		if body == nil {
			continue
		}

		inverseInertiaTensor := body.GetInverseInertiaTensorWorld()

		// use the same procedure as for calculating frictionless velocity
		// change to work out the angular inertia
		angularInertiaWorld := c.relativeContactPosition[i].Cross(&c.ContactNormal)
		angularInertiaWorld = inverseInertiaTensor.MulVector3(&angularInertiaWorld)
		angularInertiaWorld = angularInertiaWorld.Cross(&c.relativeContactPosition[i])
		angularInertia[i] = angularInertiaWorld.Dot(&c.ContactNormal)

		// the linear component is simply the inverse mass
		linearInertia[i] = body.GetInverseMass()

		// keep track of the total inertia from all component-wise
		totalInertia += linearInertia[i] + angularInertia[i]
	}

	// the logic is split into two loops so that totalInertia is completely calculated

	for i := 0; i < 2; i++ {
		body := c.Bodies[i]
		if body == nil {
			continue
		}

		// the linear and angular movements required are in proportion
		// to the two inverse inertias
		var sign m.Real = 1.0
		if i != 0 {
			sign = -1.0
		}
		angularMove[i] = sign * penetration * (angularInertia[i] / totalInertia)
		linearMove[i] = sign * penetration * (linearInertia[i] / totalInertia)

		// to avoid angular projections that are too great (when mass is large, but
		// inertia tensor is small) limit the angular move.
		projection := c.relativeContactPosition[i]
		projection.AddScaled(&c.ContactNormal, -c.relativeContactPosition[i].Dot(&c.ContactNormal))

		// use the small angle approximation for the sine of the angle (i.e. the
		// magnitude would be sin(angularLimit) * projection.magnitude
		// but we approximate sin(angularLimit) to angularLimit.
		maxMagnitude := angularLimit * projection.Magnitude()

		if angularMove[i] < -maxMagnitude {
			totalMove := angularMove[i] + linearMove[i]
			angularMove[i] = -maxMagnitude
			linearMove[i] = totalMove - angularMove[i]
		} else if angularMove[i] > maxMagnitude {
			totalMove := angularMove[i] + linearMove[i]
			angularMove[i] = maxMagnitude
			linearMove[i] = totalMove - angularMove[i]
		}

		// we have the linear amount of movement required by turning the RigidBody
		// (in angularMove[i]); we now need to calculate the desired rotation to achieve that.
		if angularMove[i] == 0.0 {
			// easy -- no movement means no rotation
			angularChange[i].Clear()
		} else {
			// work out the direction we'd like to rotate in
			targetAngularDirection := c.relativeContactPosition[i].Cross(&c.ContactNormal)
			inverseInertiaTensor := body.GetInverseInertiaTensorWorld()
			angularChange[i] = inverseInertiaTensor.MulVector3(&targetAngularDirection)
			angularChange[i].MulWith(angularMove[i] / angularInertia[i])
		}

		// velocity change is easier -- it's just the linear movement along ContactNormal
		linearChange[i] = c.ContactNormal
		linearChange[i].MulWith(linearMove[i])

		// now we can start to apply the  values we've calculated, starting with linear movement
		pos := body.Position
		pos.AddScaled(&c.ContactNormal, linearMove[i])
		body.Position = pos

		// now we do the change in orientation
		q := body.Orientation
		q.AddScaledVector(&angularChange[i], 1.0)
		body.Orientation = q

		// we need to calculate the derived data for body that is asleep, so that
		// the changes are reflected in the object's data. otherwise the resolution
		// will not change the position of the object and the next collision detection
		// round will have the same penetration.
		if body.IsAwake {
			body.CalculateDerivedData()
		}
	}

	return
}

func adjustVelocities(maxIterations int, contacts []*Contact, duration m.Real) {
	// iteratively handle impacts in order of severity
	iterationsUsed := 0
	for iterationsUsed < maxIterations {
		max := velocityEpsilon
		index := len(contacts)
		for i, c := range contacts {
			if c.desiredDeltaVelocity > max {
				max = c.desiredDeltaVelocity
				index = i
			}
		}
		if index == len(contacts) {
			break
		}
		contact := contacts[index]

		// match the awake state at the contact
		contact.matchAwakeState()

		// do the resolution on the contact that came out on top
		velocityChange, rotationChange := contact.applyVelocityChange()

		// with the change in velocity of the two bodies, the update of contact
		// velocities means that some of the relative closing velocities need recomputing.
		for _, c2 := range contacts {
			// check each body
			for b := 0; b < 2; b++ {
				// check for a match with each body in the newly resolved contact
				for d := 0; d < 2; d++ {
					if c2.Bodies[b] == contact.Bodies[d] {
						deltaVel := rotationChange[d].Cross(&c2.relativeContactPosition[b])
						deltaVel.Add(&velocityChange[d])

						// the sign of the change is negative if we're dealing with
						// the second body in a contact.
						var sign m.Real = 1.0
						if b == 0 {
							sign = -1.0
						}

						deltaVel = c2.contactToWorld.MulVector3(&deltaVel)
						deltaVel.MulWith(sign)
						c2.contactVelocity.Add(&deltaVel)
					}
				} // d
			} // b
		} // c2
		iterationsUsed++
	}
}

func (c *Contact) applyVelocityChange() (velocityChange, rotationChange [2]m.Vector3) {
	// get hold of the inverse mass and inverse inertia tensor, both in World Space
	var inverseInertiaTensors [2]m.Matrix3
	inverseInertiaTensors[0] = c.Bodies[0].GetInverseInertiaTensorWorld()
	if c.Bodies[1] != nil {
		inverseInertiaTensors[1] = c.Bodies[1].GetInverseInertiaTensorWorld()
	}

	// we will calculate the impulse for each contact axis
	var impulseContact m.Vector3

	if c.Friction == 0.0 {
		// use the short format for frictionless contacts
		impulseContact = c.calculateFrictionlessImpulse(inverseInertiaTensors)
	} else {
		// otherwise we may have impulses that aren't in the direction of the
		// contact, so we need the more complex version
		impulseContact = c.calculateFrictionImpulse(inverseInertiaTensors)
	}

	// convert impulse to world coordinates
	impulse := c.contactToWorld.MulVector3(&impulseContact)

	// split in the impulse into linear and rotation component-wise
	impulsiveTorque := c.relativeContactPosition[0].Cross(&impulse)
	rotationChange[0] = inverseInertiaTensors[0].MulVector3(&impulsiveTorque)
	velocityChange[0].Clear()
	velocityChange[0].AddScaled(&impulse, c.Bodies[0].GetInverseMass())

	// apply the changes
	c.Bodies[0].AddVelocity(&velocityChange[0])
	c.Bodies[0].AddRotation(&rotationChange[0])

	if c.Bodies[1] != nil {
		// work out the second body's linear and angular changes
		impulsiveTorque = impulse.Cross(&c.relativeContactPosition[1])
		rotationChange[1] = inverseInertiaTensors[1].MulVector3(&impulsiveTorque)
		velocityChange[1].Clear()
		velocityChange[1].AddScaled(&impulse, -c.Bodies[1].GetInverseMass())

		// apply the changes
		c.Bodies[1].AddVelocity(&velocityChange[1])
		c.Bodies[1].AddRotation(&rotationChange[1])
	}

	return
}

func (c *Contact) calculateFrictionlessImpulse(inverseInertiaTensors [2]m.Matrix3) (impulseContact m.Vector3) {
	// build a vector that shows the change in velocity in World Space for
	// a unit impulse in the direction of the contact normal
	deltaVelWorld := c.relativeContactPosition[0].Cross(&c.ContactNormal)
	deltaVelWorld = inverseInertiaTensors[0].MulVector3(&deltaVelWorld)
	deltaVelWorld = deltaVelWorld.Cross(&c.relativeContactPosition[0])

	// work out the change in velocity in contact coordinates
	deltaVelocity := deltaVelWorld.Dot(&c.ContactNormal)

	// add the linear component of velocity change
	deltaVelocity += c.Bodies[0].GetInverseMass()

	// check if we need to process the second body's data
	if c.Bodies[1] == nil {
		// go through the same transformation sequence again
		deltaVelWorld := c.relativeContactPosition[1].Cross(&c.ContactNormal)
		deltaVelWorld = inverseInertiaTensors[1].MulVector3(&deltaVelWorld)
		deltaVelWorld = deltaVelWorld.Cross(&c.relativeContactPosition[1])

		// work out the change in velocity in contact coordinates
		// NOTE: should this be a +=?
		deltaVelocity += deltaVelWorld.Dot(&c.ContactNormal)

		// add the linear component of velocity change
		deltaVelocity += c.Bodies[1].GetInverseMass()
	}

	// calculate the required size of the impulse
	impulseContact[0] = c.desiredDeltaVelocity / deltaVelocity
	impulseContact[1] = 0
	impulseContact[2] = 0
	return
}

func (c *Contact) calculateFrictionImpulse(inverseInertiaTensors [2]m.Matrix3) (impulseContact m.Vector3) {
	inverseMass := c.Bodies[0].GetInverseMass()

	// the equivalent of a cross product in matrices is multiplication
	// by a skew symmetric matrix - we build the matrix for converting
	// between linear and angular quantities.
	var impulseToTorque m.Matrix3
	setSkewSymmetric(&impulseToTorque, &c.relativeContactPosition[0])

	// build the matrix to convert contact impulse to change in velocity in
	// world coordinates
	deltaVelWorld := impulseToTorque
	deltaVelWorld = deltaVelWorld.MulMatrix3(&inverseInertiaTensors[0])
	deltaVelWorld = deltaVelWorld.MulMatrix3(&impulseToTorque)
	deltaVelWorld.MulWith(-1.0)

	// check to see if we need to add the second body's data
	if c.Bodies[1] != nil {
		// set the cross product matrix
		setSkewSymmetric(&impulseToTorque, &c.relativeContactPosition[1])

		// calculate the velocity change matrix
		deltaVelWorld2 := impulseToTorque
		deltaVelWorld2 = deltaVelWorld2.MulMatrix3(&inverseInertiaTensors[1])
		deltaVelWorld2 = deltaVelWorld2.MulMatrix3(&impulseToTorque)
		deltaVelWorld2.MulWith(-1.0)

		// add to the total delta velocity
		deltaVelWorld.Add(&deltaVelWorld2)

		// add to the inverse mass
		inverseMass += c.Bodies[1].GetInverseMass()
	}

	// do a change of basis to convert into contact coordinates
	deltaVelocity := c.contactToWorld.Transpose()
	deltaVelocity = deltaVelocity.MulMatrix3(&deltaVelWorld)
	deltaVelocity = deltaVelocity.MulMatrix3(&c.contactToWorld)

	// add in the linear velocity change
	deltaVelocity[0] += inverseMass
	deltaVelocity[4] += inverseMass
	deltaVelocity[8] += inverseMass

	// invert to get the impulse needed per unit velocity
	impulseMatrix := deltaVelocity.Invert()

	// find the target velocities to kill
	velKill := m.Vector3{
		c.desiredDeltaVelocity,
		-c.contactVelocity[1],
		-c.contactVelocity[2],
	}

	// find the impulse to kill target velocities
	impulseContact = impulseMatrix.MulVector3(&velKill)

	// check for exceeding friction
	planarImpulse := m.Real(math.Sqrt(float64(
		impulseContact[1]*impulseContact[1] +
			impulseContact[2]*impulseContact[2])))
	if planarImpulse > impulseContact[0]*c.Friction {
		// we need to use dynamic friction
		impulseContact[1] /= planarImpulse
		impulseContact[2] /= planarImpulse

		impulseContact[0] = deltaVelocity[0] +
			deltaVelocity[3]*c.Friction*impulseContact[1] +
			deltaVelocity[6]*c.Friction*impulseContact[2]
		impulseContact[0] = c.desiredDeltaVelocity / impulseContact[0]
		impulseContact[1] *= c.Friction * impulseContact[0]
		impulseContact[2] *= c.Friction * impulseContact[0]
	}

	return
}

// setSkewSymmetric sets the matrix to be a skew symmetric matrix based on
// the given vector. The skew symmetric matrix is the equivalent of a vector
// cross prodcut. So if a,b are vectors. a x b = A_s b
// where A_s is the skew symmetrix form of a.
func setSkewSymmetric(m *m.Matrix3, v *m.Vector3) {
	m[0], m[3], m[6] = 0.0, -v[2], v[1]
	m[1], m[4], m[7] = v[2], 0.0, -v[0]
	m[2], m[5], m[8] = -v[1], v[0], 0.0
}
