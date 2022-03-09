# Getting Started

## Introduction

Bounce is a C++ 3D physics engine for games. The library can be used for convex hull generation, collision detection, plausible physics simulation of rigid bodies.

First of all, for using Bounce, you’ll need to be experienced with C++ programming, and have a good background in 3D mathematics. Besides that, it is important to be familiar with essential physics concepts such as time, position, velocity, force, and impulse. Fortunately there are tons of explanations for these concepts available in physics books and in the net. 

## Help

By definition, the present Getting Started guide is not a complete user manual. Therefore, if you want to report bugs, need to ask questions, or want collaborate with the project you can open an issue in the GitHub issue tracker at https://github.com/irlanrobson/bounce. Installation instructions are available in this repository as well. 

**Note**: Bounce is open source, but is not open contribution.

## Basic Concepts

### Convex Hull

A convex hull is a geometrical object. For any two given points on a convex hull the line between these two points is also contained inside the convex hull. Examples of convex hulls are spheres, capsules, and boxes.

### Collision Shape

Thus, a geometrical object which contains only geometrical and topological information of that object. A convex hull, for example, thus is a collision shape.

### Fixture

Extends a collision shape by containing physics related parameters.

### Rigid Body

A rigid body is a solid body. The distance between any two given points on a rigid body is constant over time independently of how much force is exerted on it. Think of a rock. Each 3D rigid body has 6 degrees of freedom. 3 linear degrees of freedom and 3 angular degrees of freedom.

### Constraint

A constraint removes the relative degrees of freedom between two or more rigid bodies. 

### Contact

A contact is a constraint between two fixtures that exists when the shapes are overlapping with each other. A sphere slipping over a plane is constrained to move only tangentially to that plane.

### Joint

Constraint between two or more rigid bodies. A door attached to a wall by a hinge is constrained to rotate about the axis of rotation of the hinge. 

### World

A world is a set of physics objects such as rigid bodies and joints. Each body can be affected by the world gravitational force.

## System of Units

Bounce uses the MKS system of units.  The MKS system has **metre**, **kilogram**, and **seconds** as its base units. 

**Note**: Angles have units of radians, not degrees. 

## Hello World!

Probably the easiest route to take in order to learn how to use a library is creating a program that uses some of its basic features. Let us begin by writing a very simple program in C++ which runs 1 second of simulation. For simplicity, we will be using printf for rendering the simulation results. This program is available in the examples folder inside the project repository.

```cpp
#include <bounce/bounce.h>
#include <stdio.h>

// This example shows how to setup and run a simple simulation 
// using Bounce. 
int main(int argc, char** argv)
{
	// The world. We allocate it using the heap but you can to it 
	// on the stack if the stack is sufficiently large.
	b3World* world = new b3World();

	// The world gravity.
	const b3Vec3 gravity(0.0f, -9.8f, 0.0f);
	
	world->SetGravity(gravity);
	
	// The fixed time step size.
	const scalar timeStep = 1.0f / 60.0f;
	
	// Number of iterations for the velocity constraint solver.
	const uint32 velocityIterations = 8;

	// Number of iterations for the position constraint solver.
	const uint32 positionIterations = 2;

	// Create a static ground body at the world origin.
	b3BodyDef groundDef;
	b3Body* ground = world->CreateBody(groundDef);

	// Create a box positioned at the world origin and 
	// aligned with the world frame.
	b3BoxHull groundBox(10.0f, 1.0f, 10.0f);

	// Create the box collision shape.
	b3HullShape groundShape;
	groundShape.m_hull = &groundBox;

	// Add the box to the ground body. This creates the box physics wrapper.
	b3FixtureDef groundBoxDef;
	groundBoxDef.shape = &groundShape;
	ground->CreateFixture(groundBoxDef);

	// Create a dynamic body.
	b3BodyDef bodyDef;
	bodyDef.type = e_dynamicBody;
	
	// Position the body 10 meters high from the world origin.
	bodyDef.position.Set(0.0f, 10.0f, 0.0f);
	
	// Set the initial angular velocity to pi radians (180 degrees) per second.
	bodyDef.angularVelocity.Set(0.0f, B3_PI, 0.0f);
	
	b3Body* body = world->CreateBody(bodyDef);

	// Create a unit box positioned at the world origin and 
	// aligned with the world frame.
	b3BoxHull bodyBox;
	bodyBox.SetIdentity();

	// Create the box collision shape.
	b3HullShape bodyShape;
	bodyShape.m_hull = &bodyBox;

	// Add the box to the body.
	b3FixtureDef bodyBoxDef;
	bodyBoxDef.shape = &bodyShape;
	bodyBoxDef.density = 1.0f;
	body->CreateFixture(bodyBoxDef);

	// Run a small game loop of 60 frames length.
	for (uint32 i = 0; i < 60; ++i)
	{
		// Perform a time step of the world in this frame.
		world->Step(timeStep, velocityIterations, positionIterations);
		
		// Read the body position and orientation in this frame.
		b3Vec3 position = body->GetPosition();
		b3Quat orientation = body->GetOrientation();
		
		// Decode the axis and angle of rotation about it from the quaternion.
		b3Vec3 axis;
		scalar angle;
		orientation.GetAxisAngle(&axis, &angle);

		// Visualize the body state in this frame.
		printf("position = %.2f %.2f %.2f\n", position.x, position.y, position.z);
		printf("axis = %.2f %.2f %.2f, angle = %.2f\n\n", axis.x, axis.y, axis.z, angle);
	}
	
	// Now destroy the bodies since the world manages their lifetime.
	delete world;

	return 0;
}
```

Now we are going to explain step-by-step each part of the code.

### Including Bounce headers

We first start by including the main project header which is bounce.h. We also include stdio.h in order to use printf later.

```cpp
#include <bounce/bounce.h>
#include <stdio.h>
```

### Creating the world

In Bounce a world is an object managing all the physics objects and also exposing interfaces for the user to perform different types of physics queries. Every program that uses Bounce requires the user to create (and destroy) a world.

```cpp	
b3World* world = new b3World();
```

You can set the world acceleration of gravity. The function used to set the world acceleration of gravity is

```cpp
b3World::SetGravity(const b3Vec3& gravity);
```

For example, near the surface of the Earth each object accelerates towards the Earth center of mass by approximately 9.8 m/s^2. Units are meters per second squared. Here we set gravity to the vector (0, -9.8, 0) assuming that the world surface is the xz-plane.

```cpp
const b3Vec3 gravity(0.0f, -9.8f, 0.0f);
world->SetGravity(gravity);
```

Next we need to define the simulation parameters. First we must set a time-step for the simulation. A time-step is how many seconds the simulation should be advanced at each step. For performing a single time-step the following function needs to be called.

```cpp
void b3World::Step(float32 timeStep, uint32 velocityIterations, uint32 positionIterations);
```

It is recommended to use a small and constant time step during the simulation for several reasons. Mainly because of simulation realism, determinism, and stability. Remember that thus time has units of seconds.

```cpp
const float32 timeStep = 1.0f / 60.0f;
```

In order to satisfy constraints (due to joints and contacts) while reaching good performance, Bounce uses iterative constraint solvers. The iterative constraint solvers require defining the number of solver iterations. The velocity solver corrects violated velocities of rigid bodies while the position solver fixes violated positions of rigid bodies. Basically the number of iterations depends on the simulation configuration. For example, a tall stack of boxes will probably require a large number of velocity iterations in order to remain stable. A small number of iterations can make the stack fall. This is not realistic.

```cpp
const uint32 velocityIterations = 8;
const uint32 positionIterations = 2;
```

Now that we have set up the world let us add some rigid bodies to it.

### Creating the ground body

Bodies are created and destroyed by the following functions.

```cpp
b3Body* b3World:: CreateBody(const b3BodyDef& def);
void b3World::DestroyBody(b3Body* body);
```

Body creation requires you to define the initial configuration of the body using a body definition. b3BodyDef is the body definition. By default, the body definition is configured to set the body to a static body and located at the origin (0, 0, 0).

```cpp
b3BodyDef groundDef;
b3Body* ground = world->CreateBody(groundDef);
```

### Creating the ground shape

In the example code we create a simple box shape for the ground body using the object b3BoxHull. 
b3BoxHull is a collision shape that extends a base b3Hull object (a convex hull). One way for the user to manipulate the box dimensions is by setting the box from box extensions in the b3BoxHull constructor.

```cpp
b3BoxHull groundBox(10.0f, 1.0f, 10.0f);
```

After the box is created, we will need to create a hull collision shape and make its hull pointer point to the box above. This shape is used for collision detection and doesn't contain physics properties.


```cpp
b3HullShape groundShape;
groundShape.m_hull = &groundBox;
```

Physics properties are contained in a fixture. Fixtures are created and destroyed using the following functions.

```cpp
b3Fixture* b3Body::CreateFixture(const b3FixtureDef& def); 
void b3Body::DestroyFixture(b3Fixture* fixture);
```
             
The first function needs a fixture definition (b3FixtureDef) to be given as parameter. But before we create the fixture, we need to attach a collision shape to the fixture definition.
    
```cpp
b3FixtureDef groundBoxDef;
groundBoxDef.shape = &groundShape;
ground->CreateFixture(groundBoxDef);
```

As you can see the function b3Body::CreateFixture, it returns a pointer to a fixture. In this simple program, however, the created fixture is not used externally so we don’t need to keep track of the fixture pointer. 

**Note**: Shapes are positioned relative to the rigid body. You can add multiple shapes per body. You can also reuse collision shapes. For example, you can create another fixture for the rigid body using the same collision shape that we’ve defined previously.

### Creating the dynamic body 

Similarly to the previous example, we can create a dynamic rigid body and attach a shape to it. However, before creating the body we must set b3BodyDef::type to e_dynamicBody. 
We also must configure the fixture definition to have a positive density. Simply set b3FixtureDef::density to a positive value. Remember that density has units of kilograms per cubic meter (kg/m^3). 
    
```cpp
b3BodyDef bodyDef;
bodyDef.type = e_dynamicBody;
	
bodyDef.position.Set(0.0f, 10.0f, 0.0f);
bodyDef.angularVelocity.Set(0.0f, B3_PI, 0.0f);
	
b3Body* body = world->CreateBody(bodyDef);

b3BoxHull bodyBox;
bodyBox.SetIdentity();

b3HullShape bodyShape;
bodyShape.m_hull = &bodyBox;

b3FixtureDef bodyBoxDef;
bodyBoxDef.shape = &bodyShape;
bodyBoxDef.density = 1.0f;
body->CreateFixture(bodyBoxDef);
```

The function b3BoxHull::SetIdentity above creates a unit box geometry.

### Stepping the world

Now that world is configured you can perform a simulation step by calling the function below. Calling this function advances the simulation by the amount of time-step given.

```cpp
world->Step(timeStep, velocityIterations, positionIterations);	
```

###	Visualizing the simulation results

Having executed the function b3World::Step the dynamic rigid body we created in the last section has been moved downwards due to acceleration of gravity. Now it’s a good time to see the simulation results in the screen.

```cpp
b3Vec3 position = body->GetPosition();
b3Quat orientation = body->GetOrientation();
		
b3Vec3 axis;
float32 angle;
orientation.GetAxisAngle(&axis, &angle);
                             
printf("position = %.2f %.2f %.2f\n", position.x, position.y, position.z);
printf("axis = %.2f %.2f %.2f, angle = %.2f\n\n", axis.x, axis.y, axis.z, angle);
```

This part of code retrieves the position and orientation of the rigid body at the current frame. printf outputs the results in the console. 

**Note**: Bounce uses quaternions for representing rotations. While it does not requires the user to have deep knowledge of quaternions, basic notion of it might help when programming with the library. Bounce mathematics library has convenience functions for converting between a quaternion and a rotation matrix, so no need to worry in practice if you’re familiar only with rotation matrices.

### Destroying the world

Finally, destroying the world will destroy all the rigid bodies that were created previously. Likewise, when the bodies are destroyed their associated shapes are destroyed as well. 


```cpp
delete world;
```

## End

Congratulations! You’ve reached the end of Bounce Getting Started guide. For further information about the project please visit https://github.com/irlanrobson/bounce. 

