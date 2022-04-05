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

	b3GJKOutput query = b3GJK(xfA, proxyA, xfB, proxyB, true);

	const scalar kTol = scalar(10) * B3_EPSILON;
	return query.distance <= kTol;
}

// Compute the closest point on a segment to a point. 
b3Vec3 b3ClosestPointOnSegment(const b3Vec3& Q, const b3Vec3& A, const b3Vec3& B)
{
	b3Vec3 AB = B - A;

	// Barycentric coordinates for Q
	scalar u = b3Dot(B - Q, AB);
	scalar v = b3Dot(Q - A, AB);

	if (v <= scalar(0))
	{
		return A;
	}

	if (u <= scalar(0))
	{
		return B;
	}

	scalar den = b3Dot(AB, AB);
	if (den < B3_LINEAR_SLOP * B3_LINEAR_SLOP)
	{
		return A;
	}

	b3Vec3 P = (u * A + v * B) / den;
	return P;
}

void b3ClosestPointsOnSegments(b3Vec3& C1, b3Vec3& C2,
	const b3Vec3& P1, const b3Vec3& Q1,
	const b3Vec3& P2, const b3Vec3& Q2)
{
	b3Vec3 E1 = Q1 - P1;
	scalar L1 = b3Length(E1);

	b3Vec3 E2 = Q2 - P2;
	scalar L2 = b3Length(E2);

	if (L1 < B3_LINEAR_SLOP && L2 < B3_LINEAR_SLOP)
	{
		C1 = P1;
		C2 = P2;
		return;
	}

	if (L1 < B3_LINEAR_SLOP)
	{
		C1 = P1;
		C2 = b3ClosestPointOnSegment(P1, P2, Q2);
		return;
	}

	if (L2 < B3_LINEAR_SLOP)
	{
		C1 = b3ClosestPointOnSegment(P2, P1, Q1);
		C2 = P2;
		return;
	}

	B3_ASSERT(L1 > scalar(0));
	b3Vec3 N1 = E1 / L1;

	B3_ASSERT(L2 > scalar(0));
	b3Vec3 N2 = E2 / L2;

	// Solve Ax = b
	// [1 -dot(n1, n2)][x1] = [-dot(n1, p1 - p2)] 
	// [dot(n2, n1) -1][x2] = [-dot(n2, p1 - p2)]
	scalar b = b3Dot(N1, N2);
	scalar den = scalar(1) - b * b;

	if (den != scalar(0))
	{
		scalar inv_den = scalar(1) / den;

		b3Vec3 E3 = P1 - P2;

		scalar d = b3Dot(N1, E3);
		scalar e = b3Dot(N2, E3);

		scalar s = inv_den * (b * e - d);
		scalar t = inv_den * (e - b * d);

		C1 = P1 + s * N1;
		C2 = P2 + t * N2;
	}
	else
	{
		C1 = P1;
		C2 = P2;
	}

	// Clamp C1 to segment 1.
	C1 = b3ClosestPointOnSegment(C1, P1, Q1);

	// Recompute closest points on segments.
	C2 = b3ClosestPointOnSegment(C1, P2, Q2);
	C1 = b3ClosestPointOnSegment(C2, P1, Q1);
}
