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
#include <bounce/collision/clip.h>
#include <bounce/collision/shapes/triangle_shape.h>
#include <bounce/collision/shapes/capsule_shape.h>
#include <bounce/collision/geometry/capsule.h>
#include <bounce/collision/geometry/triangle_hull.h>
#include <bounce/collision/geometry/geometry.h>
#include <bounce/collision/gjk/gjk_proxy.h>
#include <bounce/collision/gjk/gjk.h>
#include <bounce/collision/sat/sat_hull_edge.h>

static void b3BuildEdgeContact(b3Manifold& manifold,
	const b3Transform& xf1, const b3TriangleHull* hull1, uint32 index1, 
	const b3Transform& xf2, const b3Capsule* hull2)
{
	const b3HalfEdge* edge1 = hull1->GetEdge(index1);
	const b3HalfEdge* twin1 = hull1->GetEdge(index1 + 1);

	b3Vec3 C1 = xf1 * hull1->centroid;
	b3Vec3 P1 = xf1 * hull1->GetVertex(edge1->origin);
	b3Vec3 Q1 = xf1 * hull1->GetVertex(twin1->origin);
	b3Vec3 E1 = Q1 - P1;

	b3Vec3 P2 = xf2 * hull2->vertex1;
	b3Vec3 Q2 = xf2 * hull2->vertex2;
	b3Vec3 E2 = Q2 - P2;

	b3Vec3 N = b3Cross(E1, E2);
	N.Normalize();
	
	// Ensure normal orientation to capsule.
	if (b3Dot(N, P1 - C1) < scalar(0))
	{
		N = -N;
	}

	// Compute the closest points on the two segments.
	b3Vec3 point1, point2;
	b3ClosestPointsOnSegments(point1, point2, P1, Q1, P2, Q2);

	b3FeaturePair pair = b3MakePair(index1, index1 + 1, 0, 1);

	manifold.pointCount = 1;
	manifold.points[0].localNormal1 = b3MulC(xf1.rotation, N);
	manifold.points[0].localPoint1 = b3MulT(xf1, point1);
	manifold.points[0].localPoint2 = b3MulT(xf2, point2);
	manifold.points[0].id = b3MakeID(pair);
}

static void b3BuildFaceContact(b3Manifold& manifold,
	const b3Transform& xf1, const b3TriangleHull* hull1, uint32 index1, scalar r1,
	const b3Transform& xf2, const b3Capsule* hull2, scalar r2)
{
	b3Capsule worldHull2(xf2 * hull2->vertex1, xf2 * hull2->vertex2, r2);
	b3ClipVertex segment2[2];
	b3BuildSegment(segment2, &worldHull2);

	// Clip segment 2 against the side planes of the reference face.
	b3ClipVertex clipSegment2[2];
	uint32 clipCount = b3ClipSegmentToFaceSidePlanes(clipSegment2, segment2, xf1, r1, index1, hull1);

	// Project clipped edge 2 onto the reference face.
	b3Plane localPlane1 = hull1->GetPlane(index1);
	b3Plane plane1 = xf1 * localPlane1;

	scalar totalRadius = r1 + r2;

	uint32 pointCount = 0;
	for (uint32 i = 0; i < clipCount; ++i)
	{
		b3Vec3 c2 = clipSegment2[i].position;
		scalar s = b3Distance(c2, plane1);
		if (s <= totalRadius)
		{
			b3Vec3 c1 = b3ClosestPointOnPlane(c2, plane1);

			b3ManifoldPoint* mp = manifold.points + pointCount;
			mp->localNormal1 = localPlane1.normal;
			mp->localPoint1 = b3MulT(xf1, c1);
			mp->localPoint2 = b3MulT(xf2, c2);
			mp->id = b3MakeID(clipSegment2[i].pair);

			++pointCount;
		}
	}

	manifold.pointCount = pointCount;
}

void b3CollideTriangleAndCapsule(b3Manifold& manifold,
	const b3Transform& xf1, const b3TriangleShape* triangle1,
	const b3Transform& xf2, const b3CapsuleShape* capsule2)
{
	scalar r1 = triangle1->m_radius;
	scalar r2 = capsule2->m_radius;

	scalar totalRadius = r1 + r2;

	b3GJKProxy proxy1(triangle1, 0);
	b3GJKProxy proxy2(capsule2, 0);

	b3SimplexCache simplex;
	simplex.count = 0;
	
	b3GJKOutput query = b3GJK(xf1, proxy1, xf2, proxy2, false, &simplex);

	if (query.distance > totalRadius)
	{
		return;
	}

	b3TriangleHull hull1(triangle1->m_vertex1, triangle1->m_vertex2, triangle1->m_vertex3);
	b3Capsule hull2(capsule2->m_vertex1, capsule2->m_vertex2, r2);

	if (query.distance > scalar(0))
	{
		b3Vec3 c1 = query.point1;
		b3Vec3 c2 = query.point2;
		scalar d = query.distance;

		// Define reference normal.
		b3Vec3 N1 = (c2 - c1) / d;

		// Search reference face.
		b3Vec3 localN1 = b3MulC(xf1.rotation, N1);
		uint32 index1 = hull1.GetSupportFace(localN1);
		b3Vec3 localFaceN1 = hull1.GetPlane(index1).normal;

		// Paralell vectors |v1xv2| = sin(theta)
		const scalar kTol = scalar(0.005);
		b3Vec3 N = b3Cross(localN1, localFaceN1);
		scalar L = b3Dot(N, N);
		if (L < kTol * kTol)
		{
			// Reference face found.
			// Try to build a face contact.
			b3BuildFaceContact(manifold, xf1, &hull1, index1, r1, xf2, &hull2, r2);
			if (manifold.pointCount == 2)
			{
				return;
			}
		}

		if (triangle1->m_hasE1Vertex || triangle1->m_hasE2Vertex || triangle1->m_hasE3Vertex)
		{
			b3GJKFeaturePair featurePair = b3GetFeaturePair(simplex);

			if (featurePair.count1 == 2)
			{
				uint32 v1 = featurePair.index1[0];
				uint32 v2 = featurePair.index1[1];

				b3Vec3 vertices[3] = { triangle1->m_vertex1, triangle1->m_vertex2, triangle1->m_vertex3 };
				bool hasWing[3] = { triangle1->m_hasE1Vertex, triangle1->m_hasE2Vertex, triangle1->m_hasE3Vertex };
				b3Vec3 edgeWings[3] = { triangle1->m_e1Vertex, triangle1->m_e2Vertex, triangle1->m_e3Vertex };

				bool edgeFound = false;
				uint32 edgeIndex;
				for (uint32 i = 0; i < 3; ++i)
				{
					uint32 j = i + 1 < 3 ? i + 1 : 0;

					if (v1 == i && v2 == j)
					{
						edgeFound = true;
						edgeIndex = i;
						break;
					}

					if (v2 == i && v1 == j)
					{
						edgeFound = true;
						edgeIndex = i;
						break;
					}
				}

				B3_ASSERT(edgeFound == true);

				if (hasWing[edgeIndex] == true)
				{
					uint32 ev1 = edgeIndex;
					uint32 ev2 = edgeIndex + 1 < 3 ? edgeIndex + 1 : 0;

					// Put the closest point on the capsule to the frame of the triangle.
					b3Vec3 Q = b3MulT(xf1, c2);

					// Adjacent face triangle
					b3Vec3 A = edgeWings[edgeIndex];
					b3Vec3 B = vertices[ev2];
					b3Vec3 C = vertices[ev1];

					scalar wABC[4];
					b3BarycentricCoordinates(wABC, A, B, C, Q);

					// Is the closest point on the capsule in the Region ABC of the adjacent face?
					if (wABC[0] > scalar(0) && wABC[1] > scalar(0) && wABC[2] > scalar(0))
					{
						return;
					}
					
					b3Vec3 center = (A + B + C) / scalar(3);
					b3Plane plane = hull1.trianglePlanes[0];
					scalar distance = b3Distance(center, plane);

					// Is the edge coplanar?
					const scalar kCoplanarTol = scalar(0.005);
					if (distance > -kCoplanarTol && distance < kCoplanarTol)
					{
						b3Vec3 n = plane.normal;
						
						if (b3Dot(n, localN1) < scalar(0))
						{
							n = -n;
						}

						// c1 is constant
						// c2 = c1 + s * n1
						// c1 = c2 - s * n1
						b3Vec3 nc2 = c1 + d * b3Mul(xf1.rotation, n);

						localN1 = n;
						c2 = nc2;
					}
				}
			}
		}

		manifold.pointCount = 1;
		manifold.points[0].localNormal1 = localN1;
		manifold.points[0].localPoint1 = b3MulT(xf1, c1);
		manifold.points[0].localPoint2 = b3MulT(xf2, c2);
		manifold.points[0].id = b3MakeID(0, 0);

		return;
	}

	b3FaceQuery faceQuery = b3QueryFaceSeparation(xf1, &hull1, xf2, &hull2);
	if (faceQuery.separation > totalRadius)
	{
		return;
	}

	b3EdgeQuery edgeQuery = b3QueryEdgeSeparation(xf1, &hull1, xf2, &hull2);
	if (edgeQuery.separation > totalRadius)
	{
		return;
	}

	const scalar kRelEdgeTol = scalar(0.90);
	const scalar kAbsTol = scalar(0.5) * B3_LINEAR_SLOP;
	
	if (edgeQuery.separation > kRelEdgeTol * faceQuery.separation + kAbsTol)
	{
		b3BuildEdgeContact(manifold, xf1, &hull1, edgeQuery.index1, xf2, &hull2);
	}
	else
	{
		b3BuildFaceContact(manifold, xf1, &hull1, faceQuery.index, r1, xf2, &hull2, r2);
	}
}