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

#include <bounce/collision/collide/collide.h>
#include <bounce/collision/collide/manifold.h>
#include <bounce/collision/shapes/triangle_shape.h>
#include <bounce/collision/shapes/hull_shape.h>
#include <bounce/collision/geometry/triangle_hull.h>

// Half-edge to edge map
struct b3EdgeMap
{
	b3EdgeMap()
	{
		m_halfEdgeEdges[0] = 0;
		m_halfEdgeEdges[2] = 1;
		m_halfEdgeEdges[4] = 2;

		m_halfEdgeEdges[1] = 0;
		m_halfEdgeEdges[3] = 1;
		m_halfEdgeEdges[5] = 2;
	}

	bool IsEdgeCoplanar(uint32 index) const;
	
	const b3TriangleHull* m_triangleHull;
	bool m_hasWing[3];
	b3Vec3 m_edgeWings[3];
	uint32 m_halfEdgeEdges[6];
};

bool b3EdgeMap::IsEdgeCoplanar(uint32 halfEdgeIndex) const
{
	uint32 edgeIndex = m_halfEdgeEdges[halfEdgeIndex];

	if (m_hasWing[edgeIndex] == false)
	{
		return false;
	}
	
	uint32 ev1 = edgeIndex;
	uint32 ev2 = edgeIndex + 1 < 3 ? edgeIndex + 1 : 0;

	// Adjacent triangle
	b3Vec3 A = m_edgeWings[edgeIndex];
	b3Vec3 B = m_triangleHull->triangleVertices[ev2];
	b3Vec3 C = m_triangleHull->triangleVertices[ev1];
	
	b3Vec3 center = (A + B + C) / scalar(3);

	b3Plane frontPlane = m_triangleHull->trianglePlanes[0];

	scalar distance = b3Distance(center, frontPlane);

	const scalar kCoplanarTol = 0.005f;
	if (distance > -kCoplanarTol && distance < kCoplanarTol)
	{
		return true;
	}

	return false;
}

void b3CollideTriangleAndHull(b3Manifold& manifold,
	const b3Transform& xf1, const b3TriangleShape* triangle1,
	const b3Transform& xf2, const b3HullShape* hull2,
	b3ConvexCache* cache, 
	const b3Transform& xf01, const b3Transform& xf02)
{
	b3TriangleHull h1(triangle1->m_vertex1, triangle1->m_vertex2, triangle1->m_vertex3);
	const b3Hull* h2 = hull2->m_hull;

	b3HullShape hull1;
	hull1.m_hull = &h1;
	hull1.m_radius = triangle1->m_radius;

	b3CollideHulls(manifold, xf1, &hull1, xf2, hull2, cache, xf01, xf02);

	// Adjust normals
	b3EdgeMap edgeMap;
	edgeMap.m_triangleHull = &h1;
	
	edgeMap.m_hasWing[0] = triangle1->m_hasE1Vertex;
	edgeMap.m_hasWing[1] = triangle1->m_hasE2Vertex;
	edgeMap.m_hasWing[2] = triangle1->m_hasE3Vertex;

	edgeMap.m_edgeWings[0] = triangle1->m_e1Vertex;
	edgeMap.m_edgeWings[1] = triangle1->m_e2Vertex;
	edgeMap.m_edgeWings[2] = triangle1->m_e3Vertex;

	b3Plane plane1 = xf1 * h1.planes[0];
	b3Vec3 centroid1 = xf1 * h1.centroid;
	b3Vec3 centroid2 = xf2 * h2->centroid;

	for (uint32 i = 0; i < manifold.pointCount; ++i)
	{
		b3ManifoldPoint* mp = manifold.points + i;

		b3FeaturePair pair = mp->featurePair;

		uint32 e1;
		if (mp->edgeContact)
		{
			e1 = pair.inEdge1;
		}
		else
		{
			if (pair.inEdge1 == B3_NULL_EDGE)
			{
				e1 = pair.outEdge1;
			}
			else
			{
				e1 = pair.inEdge1;
			}

			if (e1 == B3_NULL_EDGE)
			{
				continue;
			}
		}

		b3Vec3 localN1 = mp->localNormal1;
		b3Vec3 localC1 = mp->localPoint1;
		b3Vec3 localC2 = mp->localPoint2;

		b3Vec3 n1 = b3Mul(xf1.rotation, localN1);
		b3Vec3 c1 = xf1 * localC1;
		b3Vec3 c2 = xf2 * localC2;
		scalar s = b3Dot(c2 - c1, n1);

		if (edgeMap.IsEdgeCoplanar(e1))
		{
			b3Vec3 n = plane1.normal;
			
			if (b3Dot(n, centroid2 - centroid1) < scalar(0))
			{
				n = -n;
			}

			// c1 is constant
			// c2 = c1 + s * n1
			// c1 = c2 - s * n1
			b3Vec3 nc2 = c1 + s * n;

			mp->localNormal1 = b3MulC(xf1.rotation, n);
			mp->localPoint2 = b3MulT(xf2, nc2);
		}
	}
}