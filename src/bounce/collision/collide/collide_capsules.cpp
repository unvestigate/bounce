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
#include <bounce/collision/shapes/capsule_shape.h>
#include <bounce/collision/geometry/capsule.h>

// Check if two segments are paralell.
static bool b3AreParalell(const b3Capsule& segment1, const b3Capsule& segment2)
{
	b3Vec3 E1 = segment1.vertex2 - segment1.vertex1;
	scalar L1 = b3Length(E1);
	if (L1 < B3_LINEAR_SLOP)
	{
		return false;
	}

	b3Vec3 E2 = segment2.vertex2 - segment2.vertex1;
	scalar L2 = b3Length(E2);
	if (L2 < B3_LINEAR_SLOP)
	{
		return false;
	}

	// |e1 x e2| = sin(theta) * |e1| * |e2|
	const scalar kTol = scalar(0.005f);
	b3Vec3 N = b3Cross(E1, E2);
	return b3Length(N) < kTol * L1 * L2;
}

void b3CollideCapsules(b3Manifold& manifold, 
	const b3Transform& xf1, const b3CapsuleShape* capsule1,
	const b3Transform& xf2, const b3CapsuleShape* capsule2)
{
	b3Capsule segment1;
	segment1.vertex1 = xf1 * capsule1->m_vertex1;
	segment1.vertex2 = xf1 * capsule1->m_vertex2;
	
	b3Capsule segment2;
	segment2.vertex1 = xf2 * capsule2->m_vertex1;
	segment2.vertex2 = xf2 * capsule2->m_vertex2;
	
	b3Vec3 point1, point2;
	b3ClosestPointsOnSegments(point1, point2, segment1.vertex1, segment1.vertex2, segment2.vertex1, segment2.vertex2);
	
	scalar distance = b3Distance(point1, point2);

	scalar r1 = capsule1->m_radius;
	scalar r2 = capsule2->m_radius;
	scalar totalRadius = r1 + r2;
	if (distance > totalRadius)
	{
		return;
	}

	if (distance > scalar(0) && b3AreParalell(segment1, segment2))
	{
		// Clip segment 1 against the side planes of segment 2.
		b3ClipVertex inSegment1[2];
		b3BuildSegment(inSegment1, &segment1);

		b3ClipVertex clipSegment1[2];
		uint32 clipCount = b3ClipSegmentToSegmentSidePlanes(clipSegment1, inSegment1, &segment2);

		if (clipCount == 2)
		{
			b3Vec3 cp1 = b3ClosestPointOnSegment(clipSegment1[0].position, segment2.vertex1, segment2.vertex2);
			b3Vec3 cp2 = b3ClosestPointOnSegment(clipSegment1[1].position, segment2.vertex1, segment2.vertex2);

			scalar d1 = b3Distance(clipSegment1[0].position, cp1);
			scalar d2 = b3Distance(clipSegment1[1].position, cp2);

			if (d1 > B3_EPSILON && d1 <= totalRadius && d2 > B3_EPSILON && d2 <= totalRadius)
			{
				b3Vec3 n1 = (cp1 - clipSegment1[0].position) / d1;
				b3Vec3 n2 = (cp2 - clipSegment1[1].position) / d2;

				manifold.pointCount = 2;

				manifold.points[0].localNormal1 = b3MulC(xf1.rotation, n1);
				manifold.points[0].localPoint1 = b3MulT(xf1, clipSegment1[0].position);
				manifold.points[0].localPoint2 = b3MulT(xf2, cp1);
				manifold.points[0].id = b3MakeID(clipSegment1[0].pair);

				manifold.points[1].localNormal1 = b3MulC(xf1.rotation, n2);
				manifold.points[1].localPoint1 = b3MulT(xf1, clipSegment1[1].position);
				manifold.points[1].localPoint2 = b3MulT(xf2, cp2);
				manifold.points[1].id = b3MakeID(clipSegment1[1].pair);

				return;
			}
		}
	}

	if (distance > scalar(0))
	{
		b3Vec3 normal = (point2 - point1) / distance;
		
		manifold.pointCount = 1;
		manifold.points[0].localNormal1 = b3MulC(xf1.rotation, normal);
		manifold.points[0].localPoint1 = b3MulT(xf1, point1);
		manifold.points[0].localPoint2 = b3MulT(xf2, point2);
		manifold.points[0].id = b3MakeID(0, 0);

		return;
	}
}