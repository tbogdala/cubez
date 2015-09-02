// Copyright 2015, Timothy Bogdala <tdb@animal-machine.com>
// See the LICENSE file for more details.

package cubez

import (
	"math"

	m "github.com/tbogdala/cubez/math"
)

const (
	defaultLinearDamping  = 0.95
	defaultAngularDamping = 0.8
	sleepEpsilon          = 0.3
)

var (
	defaultAcceleration = m.Vector3{0.0, -9.78, 0.0}
)

// RigidBody is the main data structure represending an object that can
// cause collisions and move around in the physics simulation.
type RigidBody struct {
	// LinearDamping holds the amount of damping applied to the linear motion
	// of the RigidBody. This is required to remove energy that might get
	// added due to the numerical instability of floating point operations.
	LinearDamping m.Real

	// AngularDamping holds the amount of damping applied to angular modtion
	// of the RigidBody. Damping is required to remove energy added through
	// numerical instability in the integrator.
	AngularDamping m.Real

	// Position is the position of the RigidBody in World Space.
	Position m.Vector3

	// Orientation is the angular orientation of the RigidBody.
	Orientation m.Quat

	// Velocity is the linear velocity of the RigidBody in World Space.
	Velocity m.Vector3

	// Acceleration is the acceleration of the RigidBody and can be
	// used to set acceleration due to gravity or any other constant
	// acceleration desired.
	Acceleration m.Vector3

	// Rotation holds the angular velocity, or rotation, of the rigid body in World Space.
	Rotation m.Vector3

	// InverseInertiaTensor holds the inverse of the rigid body's inertia tensor.
	// The inertia tnesor provided must not be degenerate (that would mean the
	// body had zero inertia for spinning along one axis). As long as the tensor
	// is finite, it will be invertible. The inverse tensor is used for similar
	// reasons to the use of the inverse mass.
	// NOTE: this is given in Body Space.
	InverseInertiaTensor m.Matrix3

	// IsAwake indicates if the RigidBody is awake and should be updated
	// upon integration.
	// Defaults to true.
	IsAwake bool

	// CanSleep indicates if the RigidBody is allowed to 'sleep' or
	// if it should always be awake.
	// Defaults to true.
	CanSleep bool

	// inverseInertiaTensorWorld holdes the inverse inertia tensor of the
	// body in World Space.
	inverseInertiaTensorWorld m.Matrix3

	// inverseMass holds the inverse of the mass of the RigidBody which
	// is used much more often in calculations than just the mass.
	inverseMass m.Real

	// mass is the stored mass of the object and is used to calculate
	// inverseMass.
	// NOTE: This variable should not be changed directly unless
	// inverseMass is also changed.
	mass m.Real

	// transform holds a transofm matrix for converting Body Space into World Space.
	transform m.Matrix3x4

	// forceAccum stores the accumulated force of the RigidBody to be applied
	// at the next integration.
	forceAccum m.Vector3

	// torqueAccum stores the accumulated torque of the RigidBody to be applied
	// at the next integration.
	torqueAccum m.Vector3

	// lastFrameAccelleration holds the linear accelleration of the RigidBody for
	// the previous frame
	lastFrameAccelleration m.Vector3

	// motion holds the amount of motion of the body and is a recently weighted
	// mean that can be used to put a body to sleep.
	motion m.Real
}

// NewRigidBody creates a new RigidBody object and returns it.
func NewRigidBody() *RigidBody {
	body := new(RigidBody)
	body.Orientation.SetIdentity()
	body.LinearDamping = defaultLinearDamping
	body.AngularDamping = defaultLinearDamping
	body.Acceleration = defaultAcceleration
	body.CanSleep = true
	body.SetAwake(true)
	return body
}

// SetMass sets the mass of the RigidBody object.
func (body *RigidBody) SetMass(mass m.Real) {
	body.mass = mass
	body.inverseMass = 1.0 / mass
}

// GetMass gets the mass of the RigidBody object.
func (body *RigidBody) GetMass() m.Real {
	if body.inverseMass == 0.0 {
		return m.MaxValue
	}
	return body.mass
}

// GetInverseMass gets the inverse mass of the RigidBody object.
func (body *RigidBody) GetInverseMass() m.Real {
	return body.inverseMass
}

// GetTransform returns a copy of the RigidBody's calculated transform matrix
func (body *RigidBody) GetTransform() m.Matrix3x4 {
	return body.transform
}

// GetLastFrameAccelleration returns a copy of the RigidBody's linear accelleration
// for the last frame.
func (body *RigidBody) GetLastFrameAccelleration() m.Vector3 {
	return body.lastFrameAccelleration
}

// GetInverseInertiaTensorWorld returns a copy of the RigidBody's inverse
// inertia tensor in World Space.
func (body *RigidBody) GetInverseInertiaTensorWorld() m.Matrix3 {
	return body.inverseInertiaTensorWorld
}

// SetAwake sets the IsAwake property of the RigidBody.
// NOTE: this function doesn't respect CanSleep.
func (body *RigidBody) SetAwake(awake bool) {
	if awake {
		body.IsAwake = true
		// add some motion to avoid it falling asleep immediately
		body.motion = sleepEpsilon * 2.0
	} else {
		body.IsAwake = false
		body.Velocity.Clear()
		body.Rotation.Clear()
	}
}

// AddVelocity adds the vector to the RigidBody's Velocity property.
func (body *RigidBody) AddVelocity(v *m.Vector3) {
	body.Velocity.Add(v)
}

// AddRotation adds the vector to the RigidBody's Rotation property.
func (body *RigidBody) AddRotation(v *m.Vector3) {
	body.Rotation.Add(v)
}

// ClearAccumulators resets all of the stored linear and torque forces
// stored in the body.
func (body *RigidBody) ClearAccumulators() {
	body.forceAccum[0], body.forceAccum[1], body.forceAccum[2] = 0.0, 0.0, 0.0
	body.torqueAccum[0], body.torqueAccum[1], body.torqueAccum[2] = 0.0, 0.0, 0.0
}

// Integrate takes all of the forces accumulated in the RigidBody and
// change the Position and Orientation of the object.
func (body *RigidBody) Integrate(duration m.Real) {
	if body.IsAwake == false {
		return
	}

	// calculate linear acceleration from force inputs.
	body.lastFrameAccelleration = body.Acceleration
	body.lastFrameAccelleration.AddScaled(&body.forceAccum, body.inverseMass)

	// calculate angular acceleration from torque inputs
	angularAcceleration := body.inverseInertiaTensorWorld.MulVector3(&body.torqueAccum)

	// adjust velocities
	// update linear velocity from both acceleration and impulse
	body.Velocity.AddScaled(&body.lastFrameAccelleration, duration)

	// update angular velocity from both acceleration and impulse
	body.Rotation.AddScaled(&angularAcceleration, duration)

	// impose drag
	body.Velocity.MulWith(m.Real(math.Pow(float64(body.LinearDamping), float64(duration))))
	body.Rotation.MulWith(m.Real(math.Pow(float64(body.AngularDamping), float64(duration))))

	// adjust positions
	// update linear positions
	body.Position.AddScaled(&body.Velocity, duration)

	//update angular position
	body.Orientation.AddScaledVector(&body.Rotation, duration)

	// normalize the orientation and update the matrixes with the new position and orientation
	body.CalculateDerivedData()
	body.ClearAccumulators()

	// update the kinetic energy store and possibly put the body to sleep
	if body.CanSleep {
		currentMotion := body.Velocity.Dot(&body.Velocity) + body.Rotation.Dot(&body.Rotation)
		bias := m.Real(math.Pow(0.5, float64(duration)))
		body.motion = bias*body.motion + (1.0-bias)*currentMotion

		if body.motion < sleepEpsilon {
			body.SetAwake(false)
		} else if body.motion > 10*sleepEpsilon {
			body.motion = 10 * sleepEpsilon
		}
	}
}

// CalculateDerivedData internal data from public data members.
//
// NOTE: This should be called after the RigidBody's state is alterted
// directly by client code; it is called automatically during integration.
//
// Particularly, call this after modifying:
//   Position, Orientation
func (body *RigidBody) CalculateDerivedData() {
	body.Orientation.Normalize()
	body.transform.SetAsTransform(&body.Position, &body.Orientation)
	transformInertiaTensor(&body.inverseInertiaTensorWorld, &body.InverseInertiaTensor, &body.transform)
}

// transformInertiaTensor is an inernal function to do an inertia tensor transform.
func transformInertiaTensor(iitWorld *m.Matrix3, iitBody *m.Matrix3, rotmat *m.Matrix3x4) {
	var t4 = rotmat[0]*iitBody[0] + rotmat[3]*iitBody[1] + rotmat[6]*iitBody[2]
	var t9 = rotmat[0]*iitBody[3] + rotmat[3]*iitBody[4] + rotmat[6]*iitBody[5]
	var t14 = rotmat[0]*iitBody[6] + rotmat[3]*iitBody[7] + rotmat[6]*iitBody[8]

	var t28 = rotmat[1]*iitBody[0] + rotmat[4]*iitBody[1] + rotmat[7]*iitBody[2]
	var t33 = rotmat[1]*iitBody[3] + rotmat[4]*iitBody[4] + rotmat[7]*iitBody[5]
	var t38 = rotmat[1]*iitBody[6] + rotmat[4]*iitBody[7] + rotmat[7]*iitBody[8]

	var t52 = rotmat[2]*iitBody[0] + rotmat[5]*iitBody[1] + rotmat[8]*iitBody[2]
	var t57 = rotmat[2]*iitBody[3] + rotmat[5]*iitBody[4] + rotmat[8]*iitBody[5]
	var t62 = rotmat[2]*iitBody[6] + rotmat[5]*iitBody[7] + rotmat[8]*iitBody[8]

	iitWorld[0] = t4*rotmat[0] + t9*rotmat[3] + t14*rotmat[6]
	iitWorld[3] = t4*rotmat[1] + t9*rotmat[4] + t14*rotmat[7]
	iitWorld[6] = t4*rotmat[2] + t9*rotmat[5] + t14*rotmat[8]

	iitWorld[1] = t28*rotmat[0] + t33*rotmat[3] + t38*rotmat[6]
	iitWorld[4] = t28*rotmat[1] + t33*rotmat[4] + t38*rotmat[7]
	iitWorld[7] = t28*rotmat[2] + t33*rotmat[5] + t38*rotmat[8]

	iitWorld[2] = t52*rotmat[0] + t57*rotmat[3] + t62*rotmat[6]
	iitWorld[5] = t52*rotmat[1] + t57*rotmat[4] + t62*rotmat[7]
	iitWorld[8] = t52*rotmat[2] + t57*rotmat[5] + t62*rotmat[8]
}
