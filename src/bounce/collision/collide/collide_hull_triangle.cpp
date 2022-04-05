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
#include <bounce/collision/shapes/hull_shape.h>
#include <bounce/collision/shapes/triangle_shape.h>
#include <bounce/collision/geometry/triangle_hull.h>

void b3CollideHullAndTriangle(b3Manifold& manifold,
	const b3Transform& xf1, const b3HullShape* hull1,
	const b3Transform& xf2, const b3TriangleShape* triangle2)
{
	const b3Hull* h1 = hull1->m_hull;
	b3TriangleHull h2(triangle2->m_vertex1, triangle2->m_vertex2, triangle2->m_vertex3);

	b3HullShape hull2;
	hull2.m_hull = &h2;
	hull2.m_radius = triangle2->m_radius;

	b3CollideHulls(manifold, xf1, hull1, xf2, &hull2);

	// Adjust normals
	b3Vec3 vertices[3] = { triangle2->m_vertex1, triangle2->m_vertex2, triangle2->m_vertex3 };
	bool hasWing[3] = { triangle2->m_hasE1Vertex, triangle2->m_hasE2Vertex, triangle2->m_hasE3Vertex };
	b3Vec3 edgeWings[3] = { triangle2->m_e1Vertex, triangle2->m_e2Vertex, triangle2->m_e3Vertex };
	
	// These map hull half edges to triangle edges
	uint32 halfEdgeEdges[6] = { 0, 0, 1, 1, 2, 2 };

	b3Plane plane2 = xf2 * h2.planes[0];
	b3Vec3 centroid1 = xf1 * h1->centroid;
	b3Vec3 centroid2 = xf2 * h2.centroid;

	for (uint32 i = 0; i < manifold.pointCount; ++i)
	{
		b3ManifoldPoint* mp = manifold.points + i;
		b3FeaturePair pair = mp->id.key.pair;

		uint32 e2;
		if (mp->edgeContact)
		{
			e2 = pair.inEdge2;
		}
		else
		{
			if (pair.inEdge2 == B3_NULL_EDGE)
			{
				e2 = pair.outEdge2;
			}
			else
			{
				e2 = pair.inEdge2;
			}

			if (e2 == B3_NULL_EDGE)
			{
				continue;
			}
		}

		uint32 edgeIndex = halfEdgeEdges[e2];

		if (hasWing[edgeIndex] == false)
		{
			continue;
		}

		uint32 ev1 = edgeIndex;
		uint32 ev2 = edgeIndex + 1 < 3 ? edgeIndex + 1 : 0;

		// Adjacent triangle
		b3Vec3 A = edgeWings[edgeIndex];
		b3Vec3 B = vertices[ev2];
		b3Vec3 C = vertices[ev1];

		b3Vec3 center = (A + B + C) / scalar(3);
		b3Plane plane = h2.trianglePlanes[0];
		scalar distance = b3Distance(center, plane);

		// Is the edge coplanar?
		const scalar kCoplanarTol = scalar(0.005);
		if (distance > -kCoplanarTol && distance < kCoplanarTol)
		{
			b3Vec3 localN1 = mp->localNormal1;
			b3Vec3 localC1 = mp->localPoint1;
			b3Vec3 localC2 = mp->localPoint2;

			b3Vec3 n1 = b3Mul(xf1.rotation, localN1);
			b3Vec3 c1 = xf1 * localC1;
			b3Vec3 c2 = xf2 * localC2;
			scalar s = b3Dot(c2 - c1, n1);

			b3Vec3 n = plane2.normal;
			
			// Ensure normal orientation to triangle B.
			if (b3Dot(n, centroid2 - centroid1) < scalar(0))
			{
				n = -n;
			}

			// c2 is constant
			// c2 = c1 + s * n1
			// c1 = c2 - s * n1
			b3Vec3 nc1 = c2 - s * n;

			mp->localNormal1 = b3MulC(xf1.rotation, n);
			mp->localPoint1 = b3MulT(xf1, nc1);
		}
	}
}