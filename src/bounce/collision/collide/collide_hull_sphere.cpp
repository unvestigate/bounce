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
#include <bounce/collision/shapes/hull_shape.h>
#include <bounce/collision/shapes/sphere_shape.h>
#include <bounce/collision/geometry/hull.h>

void b3CollideHullAndSphere(b3Manifold& manifold,
	const b3Transform& xf1, const b3HullShape* hull1,
	const b3Transform& xf2, const b3SphereShape* sphere2)
{
	scalar radius = hull1->m_radius + sphere2->m_radius;
	const b3Hull* h1 = hull1->m_hull;

	// Sphere center in the frame of the hull.
	b3Vec3 cLocal = b3MulT(xf1, b3Mul(xf2, sphere2->m_center));

	// Find the minimum separation face.	
	uint32 faceIndex = 0;
	scalar separation = -B3_MAX_SCALAR;

	for (uint32 i = 0; i < h1->faceCount; ++i)
	{
		b3Plane plane = h1->GetPlane(i);
		scalar s = b3Distance(cLocal, plane);

		if (s > radius)
		{
			// Early out.
			return;
		}

		if (s > separation)
		{
			faceIndex = i;
			separation = s;
		}
	}

	if (separation < scalar(0))
	{
		// The center is inside the hull.
		b3Plane localPlane1 = h1->planes[faceIndex];

		b3Vec3 c1 = b3ClosestPointOnPlane(cLocal, localPlane1);

		manifold.pointCount = 1;
		manifold.points[0].localNormal1 = localPlane1.normal;
		manifold.points[0].localPoint1 = c1;
		manifold.points[0].localPoint2 = sphere2->m_center;
		manifold.points[0].id = b3MakeID(0, 0);
		return;
	}

	b3ShapeGJKProxy proxy1(hull1, 0);
	b3ShapeGJKProxy proxy2(sphere2, 0);
	
	b3GJKOutput query = b3GJK(xf1, proxy1, xf2, proxy2, false);

	if (query.distance > radius)
	{
		return;
	}

	if (query.distance > scalar(0))
	{
		b3Vec3 c1 = query.point1;
		b3Vec3 c2 = query.point2;
		scalar d = query.distance;

		b3Vec3 normal = (c2 - c1) / d;

		manifold.pointCount = 1;
		manifold.points[0].localNormal1 = b3MulC(xf1.rotation, normal);
		manifold.points[0].localPoint1 = b3MulT(xf1, c1);
		manifold.points[0].localPoint2 = sphere2->m_center;
		manifold.points[0].id = b3MakeID(0, 0);
	}
}