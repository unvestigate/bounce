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

#ifndef B3_SHAPE_H
#define B3_SHAPE_H

#include <bounce/common/math/transform.h>
#include <bounce/common/graphics/color.h>
#include <bounce/collision/collision.h>
#include <bounce/collision/geometry/aabb.h>
#include <bounce/collision/geometry/sphere.h>

class b3BlockAllocator;
class b3Draw;

// This structure stores the mass-related data of a shape.
struct b3MassData 
{
	// The position of the shape's centroid relative to the shape's origin. 
	b3Vec3 center;

	// The mass of the shape, typically in kg.
	scalar mass;

	// The rotational inertia of the shape about the local origin.
	b3Mat33 I;
};

// Return the Steiner's matrix given the displacement vector from the old 
// center of rotation to the new center of rotation.
// The result equals to transpose( skew(v) ) * skew(v) or diagonal(v^2) - outer(v, v)
inline b3Mat33 b3Steiner(const b3Vec3& v)
{
	scalar xx = v.x * v.x;
	scalar yy = v.y * v.y;
	scalar zz = v.z * v.z;

	b3Mat33 S;

	S.x.x = yy + zz;
	S.x.y = -v.x * v.y;
	S.x.z = -v.x * v.z;

	S.y.x = S.x.y;
	S.y.y = xx + zz;
	S.y.z = -v.y * v.z;

	S.z.x = S.x.z;
	S.z.y = S.y.z;
	S.z.z = xx + yy;

	return S;
}

// Compute the inertia matrix of a body measured in 
// inertial frame (variable over time) given the 
// inertia matrix in body-fixed frame (constant) 
// and a rotation matrix representing the orientation 
// of the body frame relative to the inertial frame.
inline b3Mat33 b3RotateToFrame(const b3Mat33& inertia, const b3Mat33& rotation)
{
	return rotation * inertia * b3Transpose(rotation);
}

// This class represents a collision geometry used for collision detection.
class b3Shape
{
public:
	enum Type
	{
		e_sphere = 0,
		e_capsule = 1,
		e_triangle = 2,
		e_hull = 3,
		e_mesh = 4,
		e_typeCount = 5
	};

	// Default destructor does nothing.
	virtual ~b3Shape() { }

	// Get the shape type.
	Type GetType() const;

	// Clone this shape using the given allocator.
	virtual b3Shape* Clone(b3BlockAllocator* allocator) const = 0;

	// Calculate the mass data for this shape given the shape density.
	virtual void ComputeMass(b3MassData* data, scalar density) const = 0;

	// Compute the shape world AABB.
	virtual void ComputeAABB(b3AABB* aabb, const b3Transform& xf) const = 0;

	// Test if a sphere is contained inside this shape.
	virtual bool TestSphere(const b3Sphere& sphere, const b3Transform& xf) const = 0;

	// Compute the ray intersection point, normal of surface, and fraction.
	virtual bool RayCast(b3RayCastOutput* output, const b3RayCastInput& input, const b3Transform& xf) const = 0;

	// Debug draw this shape in frame mode.
	void Draw(b3Draw* draw, const b3Transform& xf, const b3Color& color) const;

	// Debug draw this shape in solid mode.
	void DrawSolid(b3Draw* draw, const b3Transform& xf, const b3Color& color) const;

	// The shape types. 
	// Types currently supported are spheres, capsules, 
	// triangles, convex hulls, and triangle meshes.
	Type m_type;

	// Radius of the shape. For convex hulls this must be B3_HULL_RADIUS. There is no support for 
	// rounded polyhedrons.
	scalar m_radius;
};

inline b3Shape::Type b3Shape::GetType() const 
{ 
	return m_type; 
}

#endif
