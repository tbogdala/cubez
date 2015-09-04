// Copyright 2015, Timothy Bogdala <tdb@animal-machine.com>
// See the LICENSE file for more details.

package main

import (
	gl "github.com/go-gl/gl/v3.3-core/gl"
	glfw "github.com/go-gl/glfw/v3.1/glfw"
	mgl "github.com/go-gl/mathgl/mgl32"
	"github.com/tbogdala/cubez"
	m "github.com/tbogdala/cubez/math"
)

var (
	app *ExampleApp

	cube         *Renderable
	cubeCollider *cubez.CollisionCube

	bullet         *Renderable
	bulletCollider *cubez.CollisionSphere

	colorShader uint32
)

// update object locations
func updateObjects(delta float64) {
	// for now there's only one box to update
	cubeCollider.Body.Integrate(m.Real(delta))
	cubeCollider.CalculateDerivedData()

	// for now we hack in the position and rotation of the collider into the renderable
	SetGlVector3(&cube.Location, &cubeCollider.Body.Position)
	SetGlQuat(&cube.LocalRotation, &cubeCollider.Body.Orientation)

	if bulletCollider != nil {
		bulletCollider.Body.Integrate(m.Real(delta))
		bulletCollider.CalculateDerivedData()
		if bullet != nil {
			SetGlVector3(&bullet.Location, &bulletCollider.Body.Position)
			SetGlQuat(&bullet.LocalRotation, &bulletCollider.Body.Orientation)
		}
	}
}

// see if any of the rigid bodys contact
func generateContacts(delta float64) (bool, []*cubez.Contact) {
	// create the ground plane
	groundPlane := cubez.NewCollisionPlane(m.Vector3{0.0, 1.0, 0.0}, 0.0)

	// see if we have a collision with the ground
	found, contacts := cubeCollider.CheckAgainstHalfSpace(groundPlane, nil)

	// run collision checks on bullets
	if bulletCollider != nil {
		f, c := bulletCollider.CheckAgainstHalfSpace(groundPlane, contacts)
		f2, c2 := cubeCollider.CheckAgainstSphere(bulletCollider, c)
		found = found || f || f2
		contacts = c2
	}

	return found, contacts
}

func updateCallback(delta float64) {
	updateObjects(delta)
	foundContacts, contacts := generateContacts(delta)
	if foundContacts {
		cubez.ResolveContacts(len(contacts)*8, contacts, m.Real(delta))
	}
}

func renderCallback(delta float64) {
	gl.Viewport(0, 0, int32(app.Width), int32(app.Height))
	gl.ClearColor(0.05, 0.05, 0.05, 1.0)
	gl.Clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT)

	// make the projection and view matrixes
	projection := mgl.Perspective(mgl.DegToRad(60.0), float32(app.Width)/float32(app.Height), 1.0, 200.0)
	view := app.CameraRotation.Mat4()
	view = view.Mul4(mgl.Translate3D(-app.CameraPos[0], -app.CameraPos[1], -app.CameraPos[2]))

	cube.Draw(projection, view)
	if bullet != nil {
		bullet.Draw(projection, view)
	}
}

func main() {
	app = NewApp()
	app.InitGraphics("Ballistic", 800, 600)
	app.SetKeyCallback(keyCallback)
	app.OnRender = renderCallback
	app.OnUpdate = updateCallback
	defer app.Terminate()

	// compile the shaders
	var err error
	colorShader, err = LoadShaderProgram(UnlitColorVertShader, UnlitColorFragShader)
	if err != nil {
		panic("Failed to compile the vertex shader! " + err.Error())
	}

	// create a test cube to render
	cube = CreateCube(-1.0, -1.0, -1.0, 1.0, 1.0, 1.0)
	cube.Shader = colorShader
	cube.Color = mgl.Vec4{1.0, 0.0, 0.0, 1.0}

	// create the collision box for the the cube
	cubeCollider = cubez.NewCollisionCube(nil, m.Vector3{1.0, 1.0, 1.0})
	cubeCollider.Body.Position = m.Vector3{0.0, 10.0, 0.0}
	cubeCollider.Body.SetMass(8.0)
	cubeCollider.Body.CalculateDerivedData()
	cubeCollider.CalculateDerivedData()

	// setup the camera
	app.CameraPos = mgl.Vec3{-3.0, 3.0, 15.0}
	app.CameraRotation = mgl.QuatLookAtV(
		mgl.Vec3{-3.0, 3.0, 15.0},
		mgl.Vec3{0.0, 1.0, 0.0},
		mgl.Vec3{0.0, 1.0, 0.0})

	gl.Enable(gl.DEPTH_TEST)
	app.RenderLoop()
}

func fire() {
	// create a test sphere to render
	bullet = CreateSphere(0.25, 16, 16)
	bullet.Shader = colorShader
	bullet.Color = mgl.Vec4{0.0, 0.0, 1.0, 1.0}

	// create the collision box for the the bullet
	bulletCollider = cubez.NewCollisionSphere(nil, 0.25)
	bulletCollider.Body.Position = m.Vector3{0.75, 1.0, 10.0}
	bulletCollider.Body.SetMass(1.0)
	bulletCollider.Body.Velocity = m.Vector3{0.0, 0.0, -50.0}

	bulletCollider.Body.CalculateDerivedData()
	bulletCollider.CalculateDerivedData()
}

func keyCallback(w *glfw.Window, key glfw.Key, scancode int, action glfw.Action, mods glfw.ModifierKey) {
	// Key W == close app
	if key == glfw.KeyEscape && action == glfw.Press {
		w.SetShouldClose(true)
	}
	if key == glfw.KeySpace && action == glfw.Press {
		fire()
	}
}
