## Bounce

Bounce is a 3D physics engine for games.

## Features

### Common

* Efficient data structures with no use of STL
* Fast memory allocators
* Built-in math library
* Tunable settings used across the entire library
* Integrated profiler
* Included debug draw utility

### Collision

* Continuous collision detection
* Dynamic tree broadphase
* Static tree for triangle meshes
* SAT
* GJK
* Spheres, capsules, triangles, convex hulls, triangle meshes 
* Optimized pair management

### Dynamics

* Rigid bodies
* Contact, friction, restitution
* Sphere, cone, revolute, and more joint types
* Joint motors, limits
* Constraint graphs
* Simulation islands and sleep management
* Linear time solver
* Stable shape stacking
* One-shot contact manifolds
* Contact clustering, reduction, and persistence
* Contact callbacks: begin, pre-solve, post-solve
* Ray-casting, shape-casting, and volume queries

### Testbed
	
* OpenGL with GLFW and GLAD
* UI by imgui
* Mouse picking
* Orbit camera
* CMake build system

### Documentation

* Doxygen API documentation</li>
* [Getting Started](https://github.com/irlanrobson/bounce/blob/master/doc/getting_started.md)

**Note**: Use the Getting Started guide and the Testbed for learning how to use Bounce. Testbed is a collection of visual tests and examples that can support the development of the library. As you would imagine, this application is not part of the library.

## License

Bounce is released under the [zlib license](https://en.wikipedia.org/wiki/Zlib_License). Please recognize this software in the product documentation if possible.

## Dependencies

### Testbed

These are the external dependencies for the Testbed example project. If you don't care about the Testbed you don't need these dependencies. 

* [GLFW](https://www.glfw.org/)
* [GLAD](https://glad.dav1d.de/)
* [imgui](https://github.com/ocornut/imgui)

## Building
* Install [CMake](https://cmake.org/)
* Ensure CMake is in the user 'PATH'

### Visual Studio

* You can run 'build.bat' from the command prompt
* Building results are in the build sub-folder
* Open bounce.sln

### Any system

* Run 'build.sh' from a bash shell
* Building results are in the build sub-folder

## Contributing

You can ask anything relative to this project using the Discussions section. Please do not use the issue tracker for asking questions. The issue tracker is not a place for this.

Please do not open pull requests with bugfixes or new features that require core library changes. Open an issue first for discussion. 
