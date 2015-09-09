// Copyright 2015, Timothy Bogdala <tdb@animal-machine.com>
// See the LICENSE file for more details.

package main

import (
  "fmt"
	gl "github.com/go-gl/gl/v3.3-core/gl"
	glfw "github.com/go-gl/glfw/v3.1/glfw"
	mgl "github.com/go-gl/mathgl/mgl32"
	"github.com/tbogdala/cubez"
	m "github.com/tbogdala/cubez/math"
)

var (
  colorShader uint32

	app *ExampleApp
  cubes []*Entity
  groundPlane *cubez.CollisionPlane
)

// update object locations
func updateObjects(delta float64) {
  for _, cube := range(cubes) {
    body := cube.Collider.GetBody()
    body.Integrate(m.Real(delta))
    cube.Collider.CalculateDerivedData()

    // for now we hack in the position and rotation of the collider into the renderable
  	SetGlVector3(&cube.Node.Location, &body.Position)
  	SetGlQuat(&cube.Node.LocalRotation, &body.Orientation)
  }
}

// see if any of the rigid bodys contact
func generateContacts(delta float64) (bool, []*cubez.Contact) {
  fmt.Printf("\n\n\n=======================\nDEBUG generateContacts\n")
  var returnFound bool
  var found bool
  var contacts []*cubez.Contact

  for _, cube := range(cubes) {
    // see if we have a collision with the ground
    found, contacts = cube.Collider.CheckAgainstHalfSpace(groundPlane, contacts)
    if found == true {
      returnFound = true
    }

    // check it against the other cubes. yes this is O(n^2) and not good practice
    for _, otherCube := range(cubes) {
      if cube == otherCube {
        continue
      }
      found, contacts = cubez.CheckForCollisions(cube.Collider, otherCube.Collider, contacts)
      if found == true {
        fmt.Printf("Found a collision between cubes\n")
        returnFound = true
      }
    }
  }


  // DEBUG PRINT
  if returnFound {
    for i,c := range(contacts) {
      if c.Bodies[1] != nil {
        fmt.Printf("Collision %d: %v vs %v | velocity %v & %v\n", i, c.Bodies[0].Position, c.Bodies[1].Position, c.Bodies[0].Velocity, c.Bodies[1].Velocity)
      }
    }
  }

	return returnFound, contacts
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

  for _,cube := range(cubes) {
    cube.Node.Draw(projection, view)
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

  // setup the slice of cubes to render
  cubes = make([]*Entity, 0, 128)

  // create the ground plane
	groundPlane = cubez.NewCollisionPlane(m.Vector3{0.0, 1.0, 0.0}, 0.0)

	// setup the camera
	app.CameraPos = mgl.Vec3{0.0, 5.0, 10.0}
	app.CameraRotation = mgl.QuatLookAtV(
		mgl.Vec3{0.0, 5.0, 10.0},
		mgl.Vec3{0.0, 0.0, 0.0},
		mgl.Vec3{0.0, 1.0, 0.0})

	gl.Enable(gl.DEPTH_TEST)
	app.RenderLoop()
}

func fire() {
  const cubesToMake = 4
  var offset float32 = 0.0
  if len(cubes) > 0 && (len(cubes)/cubesToMake) % 2 >= 1 {
    offset = 0.75
  }

  for i:=0; i<cubesToMake; i++ {
    e := new(Entity)
    e.Node = CreateCube(-0.5, -0.5, -0.5, 0.5, 0.5, 0.5)
    e.Node.Shader = colorShader
    e.Node.Color = mgl.Vec4{1.0, 0.0, 0.0, 1.0}
    e.Node.Location = mgl.Vec3{float32(i*2.0)+offset, 10.0, 0.0}

    // create the collision box for the the cube
	  cubeCollider := cubez.NewCollisionCube(nil, m.Vector3{0.5, 0.5, 0.5})
    cubeCollider.Body.Position = m.Vector3{m.Real(i*2.0)+m.Real(offset), 10.0, 0.0}
    cubeCollider.Body.SetMass(8.0)
    cubeCollider.Body.CanSleep = true
    cubeCollider.Body.CalculateDerivedData()
    cubeCollider.CalculateDerivedData()
    e.Collider = cubeCollider

    cubes = append(cubes, e)
  }
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
