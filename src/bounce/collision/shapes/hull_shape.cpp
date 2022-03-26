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

#include <bounce/collision/shapes/hull_shape.h>
#include <bounce/collision/geometry/hull.h>
#include <bounce/collision/gjk/gjk.h>
#include <bounce/collision/gjk/gjk_proxy.h>
#include <bounce/common/memory/block_allocator.h>

b3HullShape::b3HullShape()
{
	m_type = e_hull;
	m_radius = B3_HULL_RADIUS;
	m_hull = nullptr;
}

b3Shape* b3HullShape::Clone(b3BlockAllocator* allocator) const
{
	void* mem = allocator->Allocate(sizeof(b3HullShape));
	b3HullShape* clone = new (mem)b3HullShape;
	*clone = *this;
	return clone;
}

void b3HullShape::ComputeMass(b3MassData* massData, scalar density) const
{
	// M. Kallay - "Computing the Moment of Inertia of a Solid Defined by a Triangle Mesh"
	// https://github.com/erich666/jgt-code/blob/master/Volume_11/Number_2/Kallay2006/Moment_of_Inertia.cpp
	
	// Polyhedron mass, center of mass, and inertia.
	// Let rho be the polyhedron density per unit volume

	// mass = rho * int(1 * dV) 

	// centroid.x = (1 / mass) * rho * int(x * dV)
	// centroid.y = (1 / mass) * rho * int(y * dV)
	// centroid.z = (1 / mass) * rho * int(z * dV)

	// Ixx = rho * int((y^2 + z^2) * dV) 
	// Iyy = rho * int((x^2 + z^2) * dV)
	// Izz = rho * int((x^2 + y^2) * dV)
	
	// Ixy = -rho * int((x * y) * dV)
	// Ixz = -rho * int((x * z) * dV)
	// Iyz = -rho * int((y * z) * dV)

	// Iyx = Ixy
	// Izx = Ixz
	// Izy = Iyz
	B3_ASSERT(m_hull->vertexCount >= 4);

	// Put the hull relative to a point that is inside the hull 
	// to help reducing round-off errors.
	b3Vec3 s; s.SetZero();
	for (uint32 i = 0; i < m_hull->vertexCount; ++i)
	{
		s += m_hull->vertices[i];
	}
	s /= scalar(m_hull->vertexCount);

	scalar volume = scalar(0);

	b3Vec3 center; center.SetZero();
	
	scalar xx = scalar(0);
	scalar xy = scalar(0);
	scalar yy = scalar(0);
	scalar xz = scalar(0);
	scalar zz = scalar(0);
	scalar yz = scalar(0);

	for (uint32 i = 0; i < m_hull->faceCount; ++i)
	{
		const b3Face* face = m_hull->GetFace(i);
		const b3HalfEdge* begin = m_hull->GetEdge(face->edge);

		const b3HalfEdge* edge = m_hull->GetEdge(begin->next);
		do
		{
			const b3HalfEdge* next = m_hull->GetEdge(edge->next);
			
			uint32 i1 = begin->origin;
			uint32 i2 = edge->origin;
			uint32 i3 = next->origin;

			b3Vec3 v1 = m_hull->GetVertex(i1) - s;
			b3Vec3 v2 = m_hull->GetVertex(i2) - s;
			b3Vec3 v3 = m_hull->GetVertex(i3) - s;

			// Signed tetrahedron volume
			scalar D = b3Det(v1, v2, v3);

			// Contribution to the mass
			volume += D;

			// Contribution to the centroid
			b3Vec3 v4 = v1 + v2 + v3;
			
			center += D * v4;

			// Contribution to moment of inertia monomials
			xx += D * (v1.x * v1.x + v2.x * v2.x + v3.x * v3.x + v4.x * v4.x);
			yy += D * (v1.y * v1.y + v2.y * v2.y + v3.y * v3.y + v4.y * v4.y);
			zz += D * (v1.z * v1.z + v2.z * v2.z + v3.z * v3.z + v4.z * v4.z);
			xy += D * (v1.x * v1.y + v2.x * v2.y + v3.x * v3.y + v4.x * v4.y);
			xz += D * (v1.x * v1.z + v2.x * v2.z + v3.x * v3.z + v4.x * v4.z);
			yz += D * (v1.y * v1.z + v2.y * v2.z + v3.y * v3.z + v4.y * v4.z);

			edge = next;
		} while (m_hull->GetEdge(edge->next) != begin);
	}

	b3Mat33 I;

	I.x.x = yy + zz;
	I.x.y = -xy;
	I.x.z = -xz;

	I.y.x = -xy;
	I.y.y = xx + zz;
	I.y.z = -yz;

	I.z.x = -xz;
	I.z.y = -yz;
	I.z.z = xx + yy;
	
	// Total mass
	massData->mass = density * volume / 6.0f;
	
	// Center of mass
	B3_ASSERT(volume > B3_EPSILON);
	center /= scalar(4) * volume;
	massData->center = center + s;

	// Inertia relative to the local origin (s).
	massData->I = (density / scalar(120)) * I;
	
	// Shift the inertia to center of mass then to the body origin.
	// Ib = Ic - m * c^2 + m * m.c^2
	// Simplification:
	// Ib = Ic + m * (m.c^2 - c^2)
	massData->I += massData->mass * (b3Steiner(massData->center) - b3Steiner(center));
}

void b3HullShape::ComputeAABB(b3AABB* aabb, const b3Transform& xf) const
{
	aabb->Set(m_hull->vertices, m_hull->vertexCount, xf);
	aabb->Extend(m_radius);
}

bool b3HullShape::TestSphere(const b3Sphere& sphere, const b3Transform& xf) const
{
	b3GJKProxy proxy1;
	proxy1.m_count = m_hull->vertexCount;
	proxy1.m_vertices = m_hull->vertices;
	proxy1.m_radius = m_radius;

	b3GJKProxy proxy2;
	proxy2.m_buffer[0] = b3MulT(xf, sphere.center);
	proxy2.m_count = 1;
	proxy2.m_vertices = proxy2.m_buffer;
	proxy2.m_radius = sphere.radius;

	b3GJKOutput query = b3GJK(b3Transform_identity, proxy1, b3Transform_identity, proxy2, false);

	if (query.distance <= m_radius + sphere.radius)
	{
		return true;
	}

	return false;
}

bool b3HullShape::RayCast(b3RayCastOutput* output, const b3RayCastInput& input, const b3Transform& xf) const
{
	uint32 planeCount = m_hull->faceCount;
	const b3Plane* planes = m_hull->planes;

	// Put the segment into the poly's frame of reference.
	b3Vec3 p1 = b3MulC(xf.rotation, input.p1 - xf.translation);
	b3Vec3 p2 = b3MulC(xf.rotation, input.p2 - xf.translation);
	b3Vec3 d = p2 - p1;

	scalar lower = scalar(0);
	scalar upper = input.maxFraction;

	uint32 index = B3_MAX_U32;

	// s(lower) = p1 + lower * d, 0 <= lower <= kupper
	// The segment intersects the plane if a 'lower' exists
	// for which s(lower) is inside all half-spaces.

	// Solve line segment to plane:
	// dot(n, s(lower)) = offset
	// dot(n, p1 + lower * d) = offset
	// dot(n, p1) + dot(n, lower * d) = offset
	// dot(n, p1) + lower * dot(n, d) = offset
	// lower * dot(n, d) = offset - dot(n, p1)
	// lower = (offset - dot(n, p1)) / dot(n, d)

	for (uint32 i = 0; i < planeCount; ++i)
	{
		scalar numerator = planes[i].offset - b3Dot(planes[i].normal, p1);
		scalar denominator = b3Dot(planes[i].normal, d);

		if (denominator == scalar(0))
		{
			// s is parallel to this half-space.
			if (numerator < scalar(0))
			{
				// s is outside of this half-space.
				// dot(n, p1) and dot(n, p2) < 0.
				return false;
			}
		}
		else
		{
			// Original predicates:
			// lower < numerator / denominator, for denominator < 0
			// upper < numerator / denominator, for denominator < 0
			// Optimized predicates:
			// lower * denominator > numerator
			// upper * denominator > numerator
			if (denominator < scalar(0))
			{
				// s enters this half-space.
				if (numerator < lower * denominator)
				{
					// Increase lower.
					lower = numerator / denominator;
					index = i;
				}
			}
			else
			{
				// s exits the half-space.	
				if (numerator < upper * denominator)
				{
					// Decrease upper.
					upper = numerator / denominator;
				}
			}

			// Exit if intersection becomes empty.
			if (upper < lower)
			{
				return false;
			}
		}
	}

	B3_ASSERT(lower >= scalar(0) && lower <= input.maxFraction);

	if (index != B3_MAX_U32)
	{
		output->fraction = lower;
		output->normal = b3Mul(xf.rotation, planes[index].normal);
		return true;
	}

	return false;
}
