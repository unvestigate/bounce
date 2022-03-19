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

#include <bounce/collision/shapes/triangle_shape.h>
#include <bounce/collision/gjk/gjk_proxy.h>
#include <bounce/collision/gjk/gjk.h>
#include <bounce/common/memory/block_allocator.h>

b3TriangleShape::b3TriangleShape()
{
	m_type = e_triangle;
	m_radius = B3_HULL_RADIUS;
	m_hasE1Vertex = false;
	m_hasE2Vertex = false;
	m_hasE3Vertex = false;
}

b3Shape* b3TriangleShape::Clone(b3BlockAllocator* allocator) const
{
	void* mem = allocator->Allocate(sizeof(b3TriangleShape));
	b3TriangleShape* clone = new (mem)b3TriangleShape;
	*clone = *this;
	return clone;
}

void b3TriangleShape::Set(const b3Vec3& v1, const b3Vec3& v2, const b3Vec3& v3)
{
	m_vertex1 = v1;
	m_vertex2 = v2;
	m_vertex3 = v3;
	m_hasE1Vertex = false;
	m_hasE2Vertex = false;
	m_hasE3Vertex = false;
}

void b3TriangleShape::ComputeMass(b3MassData* massData, scalar density) const
{
	B3_NOT_USED(density);
	massData->center = (m_vertex1 + m_vertex2 + m_vertex3) / scalar(3);
	massData->mass = 0.0f;
	massData->I.SetZero();
}

void b3TriangleShape::ComputeAABB(b3AABB* aabb, const b3Transform& xf) const
{
	b3Vec3 lower = b3Min(m_vertex1, b3Min(m_vertex2, m_vertex3));
	b3Vec3 upper = b3Max(m_vertex1, b3Max(m_vertex2, m_vertex3));

	b3Vec3 r(m_radius, m_radius, m_radius);
	aabb->lowerBound = lower - r;
	aabb->upperBound = upper + r;
}

bool b3TriangleShape::TestSphere(const b3Sphere& sphere, const b3Transform& xf) const
{
	b3GJKProxy proxy1;
	proxy1.vertexBuffer[0] = m_vertex1;
	proxy1.vertexBuffer[1] = m_vertex2;
	proxy1.vertexBuffer[2] = m_vertex3;
	proxy1.vertexCount = 3;
	proxy1.vertices = proxy1.vertexBuffer;

	b3GJKProxy proxy2;
	proxy2.vertexBuffer[0] = b3MulT(xf, sphere.vertex);
	proxy2.vertexCount = 1;
	proxy2.vertices = proxy2.vertexBuffer;

	b3GJKOutput query = b3GJK(b3Transform_identity, proxy1, b3Transform_identity, proxy2, false);

	if (query.distance <= m_radius + sphere.radius)
	{
		return true;
	}

	return false;
}

bool b3TriangleShape::RayCast(b3RayCastOutput* output, const b3RayCastInput& input, const b3Transform& xf) const
{
	// Put the ray into the triangle's frame of reference.
	b3Vec3 p1 = b3MulT(xf, input.p1);
	b3Vec3 p2 = b3MulT(xf, input.p2);
	scalar maxFraction = input.maxFraction;

	b3Vec3 d = p2 - p1;

	if (b3LengthSquared(d) < B3_EPSILON * B3_EPSILON)
	{
		return false;
	}

	b3Vec3 v1 = m_vertex1, v2 = m_vertex2, v3 = m_vertex3;
	b3Vec3 n = b3Cross(v2 - v1, v3 - v1);
	scalar len = b3Length(n);

	if (len == scalar(0))
	{
		return false;
	}

	n /= len;

	scalar num = b3Dot(n, v1 - p1);
	scalar den = b3Dot(n, d);

	if (den == scalar(0))
	{
		return false;
	}

	scalar fraction = num / den;

	// Is the intersection not on the segment?
	if (fraction < scalar(0) || maxFraction < fraction)
	{
		return false;
	}

	b3Vec3 Q = p1 + fraction * d;

	b3Vec3 A = v1;
	b3Vec3 B = v2;
	b3Vec3 C = v3;

	b3Vec3 AB = B - A;
	b3Vec3 AC = C - A;

	b3Vec3 QA = A - Q;
	b3Vec3 QB = B - Q;
	b3Vec3 QC = C - Q;

	b3Vec3 QB_x_QC = b3Cross(QB, QC);
	b3Vec3 QC_x_QA = b3Cross(QC, QA);
	b3Vec3 QA_x_QB = b3Cross(QA, QB);

	b3Vec3 AB_x_AC = b3Cross(AB, AC);

	// Barycentric coordinates for Q
	scalar u = b3Dot(QB_x_QC, AB_x_AC);
	scalar v = b3Dot(QC_x_QA, AB_x_AC);
	scalar w = b3Dot(QA_x_QB, AB_x_AC);

	// Is the intersection on the triangle?
	if (u >= scalar(0) && v >= scalar(0) && w >= scalar(0))
	{
		output->fraction = fraction;

		// Does the ray start from below or above the triangle?
		if (num > scalar(0))
		{
			output->normal = b3Mul(xf.rotation, -n);
		}
		else
		{
			output->normal = b3Mul(xf.rotation, n);
		}

		return true;
	}

	return false;
}