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
#include <bounce/collision/collide/clip.h>
#include <bounce/collision/collide/manifold.h>
#include <bounce/collision/shapes/hull_shape.h>
#include <bounce/collision/shapes/capsule_shape.h>
#include <bounce/collision/geometry/hull.h>
#include <bounce/collision/geometry/capsule.h>

static void b3BuildEdgeContact(b3Manifold& manifold,
	const b3Transform& xf1, const b3Hull* hull1, uint32 index1,
	const b3Transform& xf2, const b3Capsule* hull2)
{
	const b3HalfEdge* edge1 = hull1->GetEdge(index1);
	const b3HalfEdge* twin1 = hull1->GetEdge(index1 + 1);

	b3Vec3 C1 = xf1 * hull1->centroid;
	b3Vec3 P1 = xf1 * hull1->GetVertex(edge1->origin);
	b3Vec3 Q1 = xf1 * hull1->GetVertex(twin1->origin);
	b3Vec3 E1 = Q1 - P1;
	b3Vec3 N1 = E1;
	scalar L1 = N1.Normalize();
	B3_ASSERT(L1 > scalar(0));

	b3Vec3 P2 = xf2 * hull2->vertex1;
	b3Vec3 Q2 = xf2 * hull2->vertex2;
	b3Vec3 E2 = Q2 - P2;
	b3Vec3 N2 = E2;
	scalar L2 = N2.Normalize();
	B3_ASSERT(L2 > scalar(0));
	
	// Compute the closest points on the two lines.
	scalar b = b3Dot(N1, N2);
	scalar den = scalar(1) - b * b;
	if (den == scalar(0))
	{
		return;
	}

	scalar inv_den = scalar(1) / den;

	b3Vec3 E3 = P1 - P2;

	scalar d = b3Dot(N1, E3);
	scalar e = b3Dot(N2, E3);

	scalar s = inv_den * (b * e - d);
	scalar t = inv_den * (e - b * d);

	b3Vec3 c1 = P1 + s * N1;
	b3Vec3 c2 = P2 + t * N2;

	// Ensure normal orientation to capsule.
	b3Vec3 N = b3Cross(E1, E2);
	scalar LN = N.Normalize();
	B3_ASSERT(LN > scalar(0));
	if (b3Dot(N, P1 - C1) < scalar(0))
	{
		N = -N;
	}

	b3FeaturePair pair = b3MakePair(index1, index1 + 1, 0, 1);

	manifold.pointCount = 1;
	manifold.points[0].localNormal1 = b3MulC(xf1.rotation, N);
	manifold.points[0].localPoint1 = b3MulT(xf1, c1);
	manifold.points[0].localPoint2 = b3MulT(xf2, c2);
	manifold.points[0].key = b3MakeKey(pair);
}

static void b3BuildFaceContact(b3Manifold& manifold,
	const b3Transform& xf1, const b3Hull* hull1, uint32 index1, scalar r1,
	const b3Transform& xf2, const b3Capsule* hull2, scalar r2)
{
	b3Capsule worldHull2(xf2 * hull2->vertex1, xf2 * hull2->vertex2, r2);
	
	b3ClipVertex segment2[2];
	b3BuildSegment(segment2, &worldHull2);

	// Clip the capsule segment against the side planes of the reference face.
	b3ClipVertex clipSegment2[2];
	uint32 clipCount = b3ClipSegmentToFaceSidePlanes(clipSegment2, segment2, xf1, r1, index1, hull1);

	// Project clipped segment on the reference face.
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
			mp->key = b3MakeKey(clipSegment2[i].pair);

			++pointCount;
		}
	}

	manifold.pointCount = pointCount;
}

void b3CollideHullAndCapsule(b3Manifold& manifold,
	const b3Transform& xf1, const b3HullShape* hull1,
	const b3Transform& xf2, const b3CapsuleShape* capsule2)
{
	scalar r1 = hull1->m_radius, r2 = capsule2->m_radius;

	scalar totalRadius = r1 + r2;

	b3ShapeGJKProxy proxy1(hull1, 0);
	b3ShapeGJKProxy proxy2(capsule2, 0);

	b3GJKOutput gjk = b3GJK(xf1, proxy1, xf2, proxy2, false);

	if (gjk.distance > totalRadius)
	{
		return;
	}

	const b3Hull* h1 = hull1->m_hull;
	const b3Capsule h2(capsule2->m_vertex1, capsule2->m_vertex2, r2);

	if (gjk.distance > scalar(0))
	{
		b3Vec3 c1 = gjk.point1;
		b3Vec3 c2 = gjk.point2;
		scalar d = gjk.distance;

		// Define reference normal.
		b3Vec3 N1 = (c2 - c1) / d;

		// Search reference face.
		b3Vec3 localN1 = b3MulC(xf1.rotation, N1);
		uint32 index1 = h1->GetSupportFace(localN1);
		b3Vec3 localFaceN1 = h1->GetPlane(index1).normal;

		// Paralell vectors |v1xv2| = sin(theta)
		const scalar kTol = scalar(0.005);
		b3Vec3 N = b3Cross(localN1, localFaceN1);
		scalar L = b3Dot(N, N);
		if (L < kTol * kTol)
		{
			// Reference face found.
			// Try to build a face contact.
			b3BuildFaceContact(manifold, xf1, h1, index1, r1, xf2, &h2, r2);
			if (manifold.pointCount == 2)
			{
				return;
			}
		}

		manifold.pointCount = 1;
		manifold.points[0].localNormal1 = localN1;
		manifold.points[0].localPoint1 = b3MulT(xf1, c1);
		manifold.points[0].localPoint2 = b3MulT(xf2, c2);
		manifold.points[0].key.triangleKey = B3_NULL_TRIANGLE;
		manifold.points[0].key.key1 = 0;
		manifold.points[0].key.key2 = 0;

		return;
	}

	b3FaceQuery faceQuery1 = b3QueryFaceSeparation(xf1, h1, xf2, &h2);
	if (faceQuery1.separation > totalRadius)
	{
		return;
	}

	b3EdgeQuery edgeQuery = b3QueryEdgeSeparation(xf1, h1, xf2, &h2);
	if (edgeQuery.separation > totalRadius)
	{
		return;
	}

	const scalar kTol = scalar(0.1) * B3_LINEAR_SLOP;
	if (edgeQuery.separation > faceQuery1.separation + kTol)
	{
		b3BuildEdgeContact(manifold, xf1, h1, edgeQuery.index1, xf2, &h2);
	}
	else
	{
		b3BuildFaceContact(manifold, xf1, h1, faceQuery1.index, r1, xf2, &h2, r2);
	}
}