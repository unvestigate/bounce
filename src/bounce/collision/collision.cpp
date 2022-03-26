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
#include <bounce/collision/gjk/gjk_proxy.h>
#include <bounce/collision/gjk/gjk.h>

void b3Manifold::Initialize()
{
	pointCount = 0;
	tangentImpulse.SetZero();
	motorImpulse = scalar(0);
	motorSpeed = scalar(0);
	tangentSpeed1 = scalar(0);
	tangentSpeed2 = scalar(0);
	for (uint32 i = 0; i < B3_MAX_MANIFOLD_POINTS; ++i)
	{
		b3ManifoldPoint* p = points + i;
		p->normalImpulse = scalar(0);
		p->persisting = false;
		p->edgeContact = false;
	}
}

void b3Manifold::Initialize(const b3Manifold& oldManifold)
{
	tangentImpulse = oldManifold.tangentImpulse;
	motorImpulse = oldManifold.motorImpulse;

	for (uint32 i = 0; i < pointCount; ++i)
	{
		b3ManifoldPoint* p1 = points + i;

		for (uint32 j = 0; j < oldManifold.pointCount; ++j)
		{
			const b3ManifoldPoint* p2 = oldManifold.points + j;

			if (p2->id == p1->id)
			{
				p1->normalImpulse = p2->normalImpulse;
				p1->persisting = true;
				break;
			}
		}
	}
}

void b3WorldManifoldPoint::Initialize(const b3ManifoldPoint* p, scalar rA, const b3Transform& xfA, scalar rB, const b3Transform& xfB)
{
	b3Vec3 nA = b3Mul(xfA.rotation, p->localNormal1);
	b3Vec3 cA = xfA * p->localPoint1;
	b3Vec3 cB = xfB * p->localPoint2;

	b3Vec3 pA = cA + rA * nA;
	b3Vec3 pB = cB - rB * nA;

	point = scalar(0.5) * (pA + pB);
	normal = nA;
	separation = b3Dot(cB - cA, nA) - rA - rB;
}

void b3WorldManifold::Initialize(const b3Manifold* m, scalar rA, const b3Transform& xfA, scalar rB, const b3Transform& xfB)
{
	center.SetZero();
	normal.SetZero();
	pointCount = m->pointCount;
	for (uint32 i = 0; i < pointCount; ++i)
	{
		const b3ManifoldPoint* p = m->points + i;
		b3WorldManifoldPoint* wp = points + i;

		wp->Initialize(p, rA, xfA, rB, xfB);

		center += wp->point;
		normal += wp->normal;
	}

	if (pointCount > 0)
	{
		center /= scalar(pointCount);
		normal.Normalize();

		tangent1 = b3Perp(normal);
		tangent2 = b3Cross(tangent1, normal);
	}
}

bool b3TestOverlap(const b3Transform& xfA, uint32 indexA, const b3Shape* shapeA,
	const b3Transform& xfB, uint32 indexB, const b3Shape* shapeB)
{
	b3GJKProxy proxyA(shapeA, indexA);
	b3GJKProxy proxyB(shapeB, indexB);

	b3GJKOutput distance = b3GJK(xfA, proxyA, xfB, proxyB, true);

	const scalar kTol = scalar(10) * B3_EPSILON;
	return distance.distance <= kTol;
}