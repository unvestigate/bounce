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

#include <bounce/collision/sat/sat.h>
#include <bounce/collision/geometry/hull.h>

// Implementation of the SAT (Separating Axis Test) for 
// convex hulls. Thanks to Dirk Gregorius for his presentation 
// at GDC 2013.

static B3_FORCE_INLINE scalar b3Project(const b3Hull* hull, const b3Plane& plane)
{
	b3Vec3 support = hull->GetVertex(hull->GetSupportVertex(-plane.normal));
	return b3Distance(support, plane);
}

// Query minimum separation distance and axis of the first hull planes.
b3FaceQuery b3QueryFaceSeparation(const b3Transform& xf1, const b3Hull* hull1,
	const b3Transform& xf2, const b3Hull* hull2)
{
	// Perform computations in the local space of the second hull.
	b3Transform xf = b3MulT(xf2, xf1);

	// Here greater means less than since is a signed distance.
	uint32 maxIndex = 0;
	scalar maxSeparation = -B3_MAX_SCALAR;

	for (uint32 i = 0; i < hull1->faceCount; ++i)
	{
		b3Plane plane = xf * hull1->GetPlane(i);
		scalar separation = b3Project(hull2, plane);
		if (separation > maxSeparation)
		{
			maxIndex = i;
			maxSeparation = separation;
		}
	}

	b3FaceQuery out;
	out.index = maxIndex;
	out.separation = maxSeparation;
	return out;
}

static B3_FORCE_INLINE bool b3IsMinkowskiFace(const b3Vec3& A, const b3Vec3& B, const b3Vec3& B_x_A, const b3Vec3& C, const b3Vec3& D, const b3Vec3& D_x_C)
{
	scalar ADC = b3Dot(A, D_x_C);
	scalar BDC = b3Dot(B, D_x_C);

	scalar CBA = b3Dot(C, B_x_A);
	scalar DBA = b3Dot(D, B_x_A);

	return CBA * DBA < scalar(0) && // Test arc CD against AB plane.
		ADC * BDC < scalar(0) && // Test arc AB against DC plane.
		CBA * BDC > scalar(0); // Test if arcs AB and CD are on the same hemisphere.
}

static scalar b3Project(const b3Vec3& P1, const b3Vec3& Q1, const b3Vec3& E1, const b3Vec3& P2, const b3Vec3& E2, const b3Vec3& C1)
{
	scalar L1 = b3Length(E1);
	B3_ASSERT(L1 > B3_LINEAR_SLOP);
	if (L1 < B3_LINEAR_SLOP)
	{
		return -B3_MAX_SCALAR;
	}
	
	scalar L2 = b3Length(E2);
	B3_ASSERT(L2 > B3_LINEAR_SLOP);
	if (L2 < B3_LINEAR_SLOP)
	{
		return -B3_MAX_SCALAR;
	}
	
	// Skip over almost parallel edges.
	const scalar kTol = scalar(0.005);
	
	b3Vec3 E1_x_E2 = b3Cross(E1, E2);
	scalar L = b3Length(E1_x_E2);
	if (L < kTol * L1 * L2)
	{
		return -B3_MAX_SCALAR;
	}

	// Ensure consistent normal orientation to hull B.
	b3Vec3 N = (scalar(1) / L) * E1_x_E2;
	if (b3Dot(N, P1 - C1) < scalar(0))
	{
		N = -N;
	}

	// d = dot(N, P2) - offset = dot(N, P2) - dot(N, P1) = dot(N, P2 - P1) 
	scalar separation = b3Dot(N, P2 - P1);
	return separation;
}

b3EdgeQuery b3QueryEdgeSeparation(const b3Transform& xf1, const b3Hull* hull1,
	const b3Transform& xf2, const b3Hull* hull2)
{
	// Query minimum separation distance and axis of the first hull planes.
	// Perform computations in the local space of the second hull.
	b3Transform xf = b3MulT(xf2, xf1);
	b3Vec3 C1 = xf * hull1->centroid;

	uint32 maxIndex1 = 0;
	uint32 maxIndex2 = 0;
	scalar maxSeparation = -B3_MAX_SCALAR;

	// Loop through the first hull's unique edges.
	for (uint32 i = 0; i < hull1->edgeCount; i += 2)
	{
		const b3HalfEdge* edge1 = hull1->GetEdge(i);
		const b3HalfEdge* twin1 = hull1->GetEdge(i + 1);

		B3_ASSERT(edge1->twin == i + 1 && twin1->twin == i);

		b3Vec3 P1 = xf * hull1->GetVertex(edge1->origin);
		b3Vec3 Q1 = xf * hull1->GetVertex(twin1->origin);
		b3Vec3 E1 = Q1 - P1;

		// The Gauss Map of edge 1.
		b3Vec3 U1 = b3Mul(xf.rotation, hull1->GetPlane(edge1->face).normal);
		b3Vec3 V1 = b3Mul(xf.rotation, hull1->GetPlane(twin1->face).normal);

		// Loop through the second hull's unique edges.
		for (uint32 j = 0; j < hull2->edgeCount; j += 2)
		{
			const b3HalfEdge* edge2 = hull2->GetEdge(j);
			const b3HalfEdge* twin2 = hull2->GetEdge(j + 1);

			B3_ASSERT(edge2->twin == j + 1 && twin2->twin == j);

			b3Vec3 P2 = hull2->GetVertex(edge2->origin);
			b3Vec3 Q2 = hull2->GetVertex(twin2->origin);
			b3Vec3 E2 = Q2 - P2;

			// The Gauss Map of edge 2.
			b3Vec3 U2 = hull2->GetPlane(edge2->face).normal;
			b3Vec3 V2 = hull2->GetPlane(twin2->face).normal;

			// Negate the Gauss Map 2 for account for the MD.
			if (b3IsMinkowskiFace(U1, V1, -E1, -U2, -V2, -E2))
			{
				scalar separation = b3Project(P1, Q1, E1, P2, E2, C1);
				if (separation > maxSeparation)
				{
					maxSeparation = separation;
					maxIndex1 = i;
					maxIndex2 = j;
				}
			}
		}
	}

	b3EdgeQuery out;
	out.index1 = maxIndex1;
	out.index2 = maxIndex2;
	out.separation = maxSeparation;
	return out;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

b3CacheType b3FeatureCache::ReadState(
	const b3Transform& xf1, const b3Hull* hull1,
	const b3Transform& xf2, const b3Hull* hull2, scalar totalRadius)
{
	// If the cache was empty or flushed choose an arbitrary feature pair.
	if (cacheType == b3CacheType::e_empty)
	{
		cacheType = b3CacheType::e_separation;
		featureType = b3FeatureType::e_face1;
		index1 = 0;
		index2 = 0;
	}

	switch (featureType)
	{
	case b3FeatureType::e_edges:
	{
		return ReadEdges(xf1, hull1, xf2, hull2, totalRadius);
	}
	case b3FeatureType::e_face1:
	{
		return ReadFace(xf1, hull1, xf2, hull2, totalRadius);
	}
	case b3FeatureType::e_face2:
	{
		return ReadFace(xf2, hull2, xf1, hull1, totalRadius);
	}
	default:
	{
		return b3CacheType::e_empty;
	}
	}
}

b3CacheType b3FeatureCache::ReadFace(
	const b3Transform& xf1, const b3Hull* hull1,
	const b3Transform& xf2, const b3Hull* hull2, scalar totalRadius)
{
	// Perform computations in the local space of the second hull.
	b3Transform xf = b3MulT(xf2, xf1);
	b3Plane plane = xf * hull1->GetPlane(index1);
	scalar separation = b3Project(hull2, plane);
	if (separation > totalRadius)
	{
		return b3CacheType::e_separation;
	}
	return b3CacheType::e_overlap;
}

b3CacheType b3FeatureCache::ReadEdges(
	const b3Transform& xf1, const b3Hull* hull1,
	const b3Transform& xf2, const b3Hull* hull2, scalar totalRadius)
{
	uint32 i = index1;
	uint32 j = index2;

	// Query minimum separation distance and axis of the first hull planes.
	// Perform computations in the local space of the second hull.
	b3Transform xf = b3MulT(xf2, xf1);
	b3Vec3 C1 = xf * hull1->centroid;

	const b3HalfEdge* edge1 = hull1->GetEdge(i);
	const b3HalfEdge* twin1 = hull1->GetEdge(i + 1);

	B3_ASSERT(edge1->twin == i + 1 && twin1->twin == i);

	b3Vec3 P1 = xf * hull1->GetVertex(edge1->origin);
	b3Vec3 Q1 = xf * hull1->GetVertex(twin1->origin);
	b3Vec3 E1 = Q1 - P1;

	// The Gauss Map of edge 1.
	b3Vec3 U1 = b3Mul(xf.rotation, hull1->GetPlane(edge1->face).normal);
	b3Vec3 V1 = b3Mul(xf.rotation, hull1->GetPlane(twin1->face).normal);

	const b3HalfEdge* edge2 = hull2->GetEdge(j);
	const b3HalfEdge* twin2 = hull2->GetEdge(j + 1);

	B3_ASSERT(edge2->twin == j + 1 && twin2->twin == j);

	b3Vec3 P2 = hull2->GetVertex(edge2->origin);
	b3Vec3 Q2 = hull2->GetVertex(twin2->origin);
	b3Vec3 E2 = Q2 - P2;

	// The Gauss Map of edge 2.
	b3Vec3 U2 = hull2->GetPlane(edge2->face).normal;
	b3Vec3 V2 = hull2->GetPlane(twin2->face).normal;

	// Negate the Gauss Map 2 for account for the MD.
	if (b3IsMinkowskiFace(U1, V1, -E1, -U2, -V2, -E2))
	{
		scalar separation = b3Project(P1, Q1, E1, P2, E2, C1);
		if (separation > totalRadius)
		{
			return b3CacheType::e_separation;
		}
		else
		{
			return b3CacheType::e_overlap;
		}
	}

	// We can't determine the cache type 
	// therefore must run SAT.
	return b3CacheType::e_empty;
}