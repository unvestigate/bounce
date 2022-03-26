/*
* Copyright (c) 2016-2019 Irlan Robson 
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BOUNCE_H
#define BOUNCE_H

// These files constitute the main Bounce API.

#include <bounce/common/settings.h>
#include <bounce/common/draw.h>

#include <bounce/collision/geometry/geometry.h>
#include <bounce/collision/geometry/sphere.h>
#include <bounce/collision/geometry/capsule.h>
#include <bounce/collision/geometry/hull.h>
#include <bounce/collision/geometry/box_hull.h>
#include <bounce/collision/geometry/cylinder_hull.h>
#include <bounce/collision/geometry/cone_hull.h>
#include <bounce/collision/geometry/mesh.h>
#include <bounce/collision/geometry/grid_mesh.h>

#include <bounce/collision/shapes/sphere_shape.h>
#include <bounce/collision/shapes/capsule_shape.h>
#include <bounce/collision/shapes/triangle_shape.h>
#include <bounce/collision/shapes/hull_shape.h>
#include <bounce/collision/shapes/mesh_shape.h>

#include <bounce/collision/collision.h>
#include <bounce/collision/broad_phase.h>

#include <bounce/dynamics/contacts/contact.h>

#include <bounce/dynamics/joints/mouse_joint.h>
#include <bounce/dynamics/joints/spring_joint.h>
#include <bounce/dynamics/joints/weld_joint.h>
#include <bounce/dynamics/joints/sphere_joint.h>
#include <bounce/dynamics/joints/revolute_joint.h>
#include <bounce/dynamics/joints/cone_joint.h>
#include <bounce/dynamics/joints/friction_joint.h>
#include <bounce/dynamics/joints/motor_joint.h>
#include <bounce/dynamics/joints/prismatic_joint.h>
#include <bounce/dynamics/joints/wheel_joint.h>

#include <bounce/dynamics/body.h>
#include <bounce/dynamics/fixture.h>
#include <bounce/dynamics/time_step.h>
#include <bounce/dynamics/world.h>
#include <bounce/dynamics/world_callbacks.h>

#endif
