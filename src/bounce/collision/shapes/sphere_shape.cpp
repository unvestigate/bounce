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

#include <bounce/collision/shapes/sphere_shape.h>
#include <bounce/common/memory/block_allocator.h>

b3SphereShape::b3SphereShape() 
{
	m_type = e_sphere;
	m_radius = scalar(0);
	m_center.SetZero();
}

b3Shape* b3SphereShape::Clone(b3BlockAllocator* allocator) const
{
	void* mem = allocator->Allocate(sizeof(b3SphereShape));
	b3SphereShape* clone = new (mem)b3SphereShape;
	*clone = *this;
	return clone;
}

void b3SphereShape::ComputeMass(b3MassData* massData, scalar density) const 
{
	scalar volume = (scalar(4) / scalar(3)) * B3_PI * m_radius * m_radius * m_radius;
	scalar mass = density * volume;
	
	// Inertia about the local shape center of mass 
	// Then shift to the local body origin
	scalar I = mass * (scalar(0.4) * m_radius * m_radius + b3Dot(m_center, m_center));
	
	massData->center = m_center;
	massData->mass = mass;
	massData->I = b3Mat33Diagonal(I);
}

void b3SphereShape::ComputeAABB(b3AABB* aabb, const b3Transform& xf) const 
{
	b3Vec3 center = b3Mul(xf, m_center);
	b3Vec3 r(m_radius, m_radius, m_radius);
	aabb->lowerBound = center - r;
	aabb->upperBound = center + r;
}

bool b3SphereShape::TestSphere(const b3Sphere& sphere, const b3Transform& xf) const
{
	b3Vec3 center = b3Mul(xf, m_center);
	scalar radius = m_radius + sphere.radius;
	scalar rr = radius * radius;
	b3Vec3 d = sphere.center - center;
	scalar dd = b3Dot(d, d);
	return dd <= rr;
}

bool b3SphereShape::RayCast(b3RayCastOutput* output, const b3RayCastInput& input, const b3Transform& xf) const
{
	// dot(x - c, x - c) - r^2 = 0
	// S = p1 + t * (p2 - p1)
	// Solve for t:
	// dot(p1 + t * d - c, P + t * d - c) - r^2 =
	// dot(d, d) * t^2 + 2 * dot(m, d) * t + dot(m, m) - r^2 = 0
	// m = p1 - c
	// d = p2 - p1
	b3Vec3 d = input.p2 - input.p1;
	scalar a = b3Dot(d, d);
	
	// Check for short segment.
	if (a < B3_EPSILON * B3_EPSILON)
	{
		return false;
	}
		
	// Solve quadratic equation
	// a * t^2 + 2 * b * t + c = 0
	// a = dot(d, d)
	// b = dot(m, d)
	// c = dot(m, m) - r^2
	// t = -b +/- sqrt(b * b - a * c) / a
	b3Vec3 m = input.p1 - b3Mul(xf, m_center);
	scalar b = b3Dot(m, d);
	scalar c = b3Dot(m, m) - m_radius * m_radius;
	
	scalar disc = b * b - a * c;

	// Check for negative discriminant.
	// Does the ray misses the sphere completely?
	if (disc < scalar(0))
	{
		return false;
	}

	// Find the minimum time of impact of the line with the sphere.
	// t_min = -b - sqrt(disc)
	// t_max = -b + sqrt(disc) 
	scalar t = -b - b3Sqrt(disc);
	
	// Is the intersection point on the segment?
	if (t > scalar(0) && t <= input.maxFraction * a) 
	{
		// Finish solution.
		t /= a;
		output->fraction = t;
		output->normal = b3Normalize(m + t * d);
		return true;
	}
	
	return false;
}