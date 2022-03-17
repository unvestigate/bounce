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

#include <bounce/collision/collision.h>
#include <bounce/collision/geometry/sphere.h>
#include <bounce/collision/geometry/capsule.h>
#include <bounce/collision/geometry/box_hull.h>
#include <bounce/collision/geometry/cylinder_hull.h>
#include <bounce/collision/geometry/cone_hull.h>

const b3Sphere b3Sphere_identity(b3Vec3(scalar(0), scalar(0), scalar(0)), scalar(1)); 

const b3Capsule b3Capsule_identity(b3Vec3(scalar(0), scalar(-0.5), scalar(0)), b3Vec3(scalar(0), scalar(0.5), scalar(0)), scalar(1));

const b3BoxHull b3BoxHull_identity(scalar(1), scalar(1), scalar(1));

const b3CylinderHull b3CylinderHull_identity(scalar(1), scalar(1));

const b3ConeHull b3ConeHull_identity(scalar(1), scalar(1));