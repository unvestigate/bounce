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
#include <bounce/collision/cluster.h>
#include <bounce/collision/shapes/hull_shape.h>
#include <bounce/collision/geometry/hull.h>
#include <bounce/collision/geometry/geometry.h>
#include <bounce/collision/sat/sat.h>

bool b3_convexCache = true;
uint32 b3_convexCalls = 0, b3_convexCacheHits = 0;

static void b3BuildEdgeContact(b3Manifold& manifold,
	const b3Transform& xf1, uint32 index1, const b3HullShape* hull1,
	const b3Transform& xf2, uint32 index2, const b3HullShape* hull2)
{
	const b3Hull* h1 = hull1->m_hull;
	const b3HalfEdge* edge1 = h1->GetEdge(index1);
	const b3HalfEdge* twin1 = h1->GetEdge(index1 + 1);

	b3Vec3 C1 = xf1 * h1->centroid;
	b3Vec3 P1 = xf1 * h1->GetVertex(edge1->origin);
	b3Vec3 Q1 = xf1 * h1->GetVertex(twin1->origin);
	b3Vec3 E1 = Q1 - P1;
	b3Vec3 N1 = E1;
	scalar L1 = N1.Normalize();
	B3_ASSERT(L1 > B3_LINEAR_SLOP);

	const b3Hull* h2 = hull2->m_hull;
	const b3HalfEdge* edge2 = h2->GetEdge(index2);
	const b3HalfEdge* twin2 = h2->GetEdge(index2 + 1);

	b3Vec3 C2 = xf2 * h2->centroid;
	b3Vec3 P2 = xf2 * h2->GetVertex(edge2->origin);
	b3Vec3 Q2 = xf2 * h2->GetVertex(twin2->origin);
	b3Vec3 E2 = Q2 - P2;
	b3Vec3 N2 = E2;
	scalar L2 = N2.Normalize();
	B3_ASSERT(L2 > B3_LINEAR_SLOP);

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

	b3Vec3 point1 = P1 + s * N1;
	b3Vec3 point2 = P2 + t * N2;

	// Ensure normal orientation to hull 2.
	b3Vec3 N = b3Cross(E1, E2);
	scalar LN = N.Normalize();
	B3_ASSERT(LN > scalar(0));
	if (b3Dot(N, P1 - C1) < scalar(0))
	{
		N = -N;
	}

	b3FeaturePair pair = b3MakePair(index1, index1 + 1, index2, index2 + 1);

	manifold.pointCount = 1;
	manifold.points[0].localNormal1 = b3MulC(xf1.rotation, N);
	manifold.points[0].localPoint1 = b3MulT(xf1, point1);
	manifold.points[0].localPoint2 = b3MulT(xf2, point2);
	manifold.points[0].id = b3MakeID(pair);
	manifold.points[0].edgeContact = true;
}

static void b3BuildFaceContact(b3Manifold& manifold,
	const b3Transform& xf1, uint32 index1, const b3HullShape* hull1,
	const b3Transform& xf2, const b3HullShape* hull2,
	bool flipNormal)
{
	const b3Hull* h1 = hull1->m_hull;
	scalar r1 = hull1->m_radius;

	const b3Hull* h2 = hull2->m_hull;
	scalar r2 = hull2->m_radius;

	scalar totalRadius = r1 + r2;

	// 1. Define the reference face plane (1).
	const b3Face* face1 = h1->GetFace(index1);
	const b3HalfEdge* edge1 = h1->GetEdge(face1->edge);
	b3Plane localPlane1 = h1->GetPlane(index1);
	b3Vec3 localNormal1 = localPlane1.normal;
	b3Vec3 localPoint1 = h1->GetVertex(edge1->origin);
	b3Plane plane1 = b3Mul(xf1, localPlane1);

	// 2. Find the incident face polygon (2).	

	// Put the reference plane normal in the frame of the incident hull (2).
	b3Vec3 normal1 = b3MulC(xf2.rotation, plane1.normal);

	// Find the support face polygon in the *negated* direction.
	b3StackArray<b3ClipVertex, 32> polygon2;
	uint32 index2 = h2->GetSupportFace(-normal1);
	b3BuildPolygon(polygon2, xf2, index2, h2);

	// 3. Clip incident face polygon (2) against the reference face (1) side planes.
	b3StackArray<b3ClipVertex, 32> clipPolygon2;
	b3ClipPolygonToFaceSidePlanes(clipPolygon2, polygon2, xf1, totalRadius, index1, h1);
	if (clipPolygon2.IsEmpty())
	{
		return;
	}

	// 4. Project the clipped polygon on the reference plane for reduction.
	// Ensure the deepest point is contained in the reduced polygon.
	b3StackArray<b3ClusterVertex, 32> polygon1;

	uint32 minIndex = 0;
	scalar minSeparation = B3_MAX_SCALAR;

	for (uint32 i = 0; i < clipPolygon2.Count(); ++i)
	{
		b3ClipVertex v2 = clipPolygon2[i];
		scalar separation = b3Distance(v2.position, plane1);

		if (separation <= totalRadius)
		{
			if (separation < minSeparation)
			{
				minIndex = polygon1.Count();
				minSeparation = separation;
			}

			b3ClusterVertex v1;
			v1.position = b3ClosestPointOnPlane(v2.position, plane1);
			v1.clipIndex = i;
			polygon1.PushBack(v1);
		}
	}

	if (polygon1.IsEmpty())
	{
		return;
	}

	// 5. Reduce.
	b3Vec3 normal = plane1.normal;

	// Ensure normal orientation to hull 2.
	b3Vec3 orientedNormal = flipNormal ? -normal : normal;

	b3StackArray<b3ClusterVertex, 32> reducedPolygon1;
	b3ReducePolygon(reducedPolygon1, polygon1, orientedNormal, minIndex);
	
	// 6. Build face contact.
	uint32 pointCount = reducedPolygon1.Count();
	for (uint32 i = 0; i < pointCount; ++i)
	{
		uint32 clipIndex = reducedPolygon1[i].clipIndex;
		b3ClipVertex v2 = clipPolygon2[clipIndex];
		b3Vec3 v1 = b3ClosestPointOnPlane(v2.position, plane1);

		b3ManifoldPoint* mp = manifold.points + i;

		if (flipNormal)
		{
			// Swap the feature pairs.
			b3FeaturePair pair = b3MakePair(v2.pair.inEdge2, v2.pair.outEdge2, v2.pair.inEdge1, v2.pair.outEdge1);

			mp->localNormal1 = b3MulC(xf2.rotation, orientedNormal);
			mp->localPoint1 = b3MulT(xf2, v2.position);
			mp->localPoint2 = b3MulT(xf1, v1);
			mp->id = b3MakeID(pair);
			mp->edgeContact = false;
		}
		else
		{
			mp->localNormal1 = b3MulC(xf1.rotation, orientedNormal);
			mp->localPoint1 = b3MulT(xf1, v1);
			mp->localPoint2 = b3MulT(xf2, v2.position);
			mp->id = b3MakeID(v2.pair);
			mp->edgeContact = false;
		}
	}

	manifold.pointCount = pointCount;
}

static void b3CollideHulls(b3Manifold& manifold,
	const b3Transform& xf1, const b3HullShape* hull1,
	const b3Transform& xf2, const b3HullShape* hull2)
{
	B3_ASSERT(manifold.pointCount == 0);

	const b3Hull* h1 = hull1->m_hull;
	scalar r1 = hull1->m_radius;

	const b3Hull* h2 = hull2->m_hull;
	scalar r2 = hull2->m_radius;

	scalar totalRadius = r1 + r2;

	b3FaceQuery faceQuery1 = b3QueryFaceSeparation(xf1, h1, xf2, h2);
	if (faceQuery1.separation > totalRadius)
	{
		return;
	}

	b3FaceQuery faceQuery2 = b3QueryFaceSeparation(xf2, h2, xf1, h1);
	if (faceQuery2.separation > totalRadius)
	{
		return;
	}

	b3EdgeQuery edgeQuery = b3QueryEdgeSeparation(xf1, h1, xf2, h2);
	if (edgeQuery.separation > totalRadius)
	{
		return;
	}

	// Use hysteresis for jitter reduction.
	const scalar kRelEdgeTol = scalar(0.90);
	const scalar kRelFaceTol = scalar(0.98);
	const scalar kAbsTol = scalar(0.5) * B3_LINEAR_SLOP;
	
	if (edgeQuery.separation > kRelEdgeTol * b3Max(faceQuery1.separation, faceQuery2.separation) + kAbsTol)
	{
		b3BuildEdgeContact(manifold, xf1, edgeQuery.index1, hull1, xf2, edgeQuery.index2, hull2);
	}
	else
	{
		if (faceQuery2.separation > kRelFaceTol * faceQuery1.separation + kAbsTol)
		{
			b3BuildFaceContact(manifold, xf2, faceQuery2.index, hull2, xf1, hull1, true);
		}
		else
		{
			b3BuildFaceContact(manifold, xf1, faceQuery1.index, hull1, xf2, hull2, false);
		}
	}

	// Heuristic succeded. 
	if (manifold.pointCount > 0)
	{
		return;
	}

	// Heuristic failed. Fallback.
	if (edgeQuery.separation > b3Max(faceQuery1.separation, faceQuery2.separation))
	{
		b3BuildEdgeContact(manifold, xf1, edgeQuery.index1, hull1, xf2, edgeQuery.index2, hull2);
	}
	else
	{
		if (faceQuery2.separation > faceQuery1.separation)
		{
			b3BuildFaceContact(manifold, xf2, faceQuery2.index, hull2, xf1, hull1, true);
		}
		else
		{
			b3BuildFaceContact(manifold, xf1, faceQuery1.index, hull1, xf2, hull2, false);
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

static void b3RebuildEdgeContact(b3Manifold& manifold,
	const b3Transform& xf1, uint32 index1, const b3HullShape* hull1,
	const b3Transform& xf2, uint32 index2, const b3HullShape* hull2)
{
	const b3Hull* h1 = hull1->m_hull;
	const b3HalfEdge* edge1 = h1->GetEdge(index1);
	const b3HalfEdge* twin1 = h1->GetEdge(index1 + 1);

	b3Vec3 C1 = xf1 * h1->centroid;
	b3Vec3 P1 = xf1 * h1->GetVertex(edge1->origin);
	b3Vec3 Q1 = xf1 * h1->GetVertex(twin1->origin);
	b3Vec3 E1 = Q1 - P1;
	b3Vec3 N1 = E1;
	scalar L1 = N1.Normalize();
	B3_ASSERT(L1 > scalar(0));

	const b3Hull* h2 = hull2->m_hull;
	const b3HalfEdge* edge2 = h2->GetEdge(index2);
	const b3HalfEdge* twin2 = h2->GetEdge(index2 + 1);

	b3Vec3 C2 = xf2 * h2->centroid;
	b3Vec3 P2 = xf2 * h2->GetVertex(edge2->origin);
	b3Vec3 Q2 = xf2 * h2->GetVertex(twin2->origin);
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

	b3Vec3 point1 = P1 + s * N1;
	b3Vec3 point2 = P2 + t * N2;

	// Check if the closest points are still lying on the opposite segments 
	// using Barycentric coordinates.
	scalar w1[3];
	b3BarycentricCoordinates(w1, P2, Q2, point1);
	
	scalar w2[3];
	b3BarycentricCoordinates(w2, P1, Q1, point2);

	if (w1[1] > scalar(0) && w1[1] <= w1[2] && 
		w2[1] > scalar(0) && w2[1] <= w2[2])
	{
		b3Vec3 N = b3Cross(E1, E2);
		scalar LN = N.Normalize();
		B3_ASSERT(LN > scalar(0));
		if (b3Dot(N, P1 - C1) < scalar(0))
		{
			N = -N;
		}

		b3FeaturePair pair = b3MakePair(index1, index1 + 1, index2, index2 + 1);

		manifold.pointCount = 1;
		manifold.points[0].localNormal1 = b3MulC(xf1.rotation, N);
		manifold.points[0].localPoint1 = b3MulT(xf1, point1);
		manifold.points[0].localPoint2 = b3MulT(xf2, point2);
		manifold.points[0].id = b3MakeID(pair);
		manifold.points[0].edgeContact = true;
	}
}

static void b3RebuildFaceContact(b3Manifold& manifold,
	const b3Transform& xf1, uint32 index1, const b3HullShape* hull1,
	const b3Transform& xf2, const b3HullShape* hull2, bool flipNormal, 
	const b3Transform& xf01, const b3Transform& xf02)
{
	b3Quat q01 = xf01.rotation, q1 = xf1.rotation;
	b3Quat q02 = xf02.rotation, q2 = xf2.rotation;

	// Check if the relative orientation has changed.
	// Here the second orientation seen by the first orientation.
	
	// The old relative rotation.
	// "q02 - q01"
	b3Quat dq0 = b3Conjugate(q01) * q02;

	// The new relative rotation.
	// "q2 - q1"
	b3Quat dq = b3Conjugate(q1) * q2;

	// Relative rotation between the new relative rotation and the old relative rotation.
	// "dq - dq0"
	b3Quat q = b3Conjugate(dq0) * dq;

	// Check the relative absolute cosine because 
	// we want to check orientation similarity.
	const scalar kCosTol = scalar(0.995);
	if (b3Abs(q.s) > kCosTol)
	{
		b3BuildFaceContact(manifold, xf1, index1, hull1, xf2, hull2, flipNormal);
	}
}

static void b3CollideHullsCache(b3Manifold& manifold,
	const b3Transform& xf1, const b3HullShape* hull1,
	const b3Transform& xf2, const b3HullShape* hull2,
	b3FeatureCache& cache)
{
	B3_ASSERT(cache.cacheType == b3CacheType::e_empty);

	const b3Hull* h1 = hull1->m_hull;
	scalar r1 = hull1->m_radius;

	const b3Hull* h2 = hull2->m_hull;
	scalar r2 = hull2->m_radius;

	scalar totalRadius = r1 + r2;

	b3FaceQuery faceQuery1 = b3QueryFaceSeparation(xf1, h1, xf2, h2);
	if (faceQuery1.separation > totalRadius)
	{
		// Write a separation cache.
		cache = b3MakeCache(b3CacheType::e_separation, b3FeatureType::e_face1, faceQuery1.index, faceQuery1.index);
		return;
	}

	b3FaceQuery faceQuery2 = b3QueryFaceSeparation(xf2, h2, xf1, h1);
	if (faceQuery2.separation > totalRadius)
	{
		// Write a separation cache.
		cache = b3MakeCache(b3CacheType::e_separation, b3FeatureType::e_face2, faceQuery2.index, faceQuery2.index);
		return;
	}

	b3EdgeQuery edgeQuery = b3QueryEdgeSeparation(xf1, h1, xf2, h2);
	if (edgeQuery.separation > totalRadius)
	{
		// Write a separation cache.
		cache = b3MakeCache(b3CacheType::e_separation, b3FeatureType::e_edges, edgeQuery.index1, edgeQuery.index2);
		return;
	}
	
	// Use hysteresis for jitter reduction.
	const scalar kRelEdgeTol = scalar(0.90);
	const scalar kRelFaceTol = scalar(0.98);
	const scalar kAbsTol = scalar(0.5) * B3_LINEAR_SLOP;

	if (edgeQuery.separation > kRelEdgeTol * b3Max(faceQuery1.separation, faceQuery2.separation) + kAbsTol)
	{
		b3BuildEdgeContact(manifold, xf1, edgeQuery.index1, hull1, xf2, edgeQuery.index2, hull2);

		// Write an overlap cache.		
		cache = b3MakeCache(b3CacheType::e_overlap, b3FeatureType::e_edges, edgeQuery.index1, edgeQuery.index2);
		return;
	}
	else
	{
		if (faceQuery2.separation > kRelFaceTol * faceQuery1.separation + kAbsTol)
		{
			b3BuildFaceContact(manifold, xf2, faceQuery2.index, hull2, xf1, hull1, true);
			if (manifold.pointCount > 0)
			{
				// Write an overlap cache.
				cache = b3MakeCache(b3CacheType::e_overlap, b3FeatureType::e_face2, faceQuery2.index, faceQuery2.index);
				return;
			}
		}
		else
		{
			b3BuildFaceContact(manifold, xf1, faceQuery1.index, hull1, xf2, hull2, false);
			if (manifold.pointCount > 0)
			{
				// Write an overlap cache.
				cache = b3MakeCache(b3CacheType::e_overlap, b3FeatureType::e_face1, faceQuery1.index, faceQuery1.index);
				return;
			}
		}
	}

	// Heuristic failed. Fallback.
	if (edgeQuery.separation > b3Max(faceQuery1.separation, faceQuery2.separation))
	{
		b3BuildEdgeContact(manifold, xf1, edgeQuery.index1, hull1, xf2, edgeQuery.index2, hull2);
		
		// Write an overlap cache.		
		cache = b3MakeCache(b3CacheType::e_overlap, b3FeatureType::e_edges, edgeQuery.index1, edgeQuery.index2);
		return;
	}
	else
	{
		if (faceQuery2.separation > faceQuery1.separation)
		{
			b3BuildFaceContact(manifold, xf2, faceQuery2.index, hull2, xf1, hull1, true);
			if (manifold.pointCount > 0)
			{
				// Write an overlap cache.
				cache = b3MakeCache(b3CacheType::e_overlap, b3FeatureType::e_face2, faceQuery2.index, faceQuery2.index);
				return;
			}
		}
		else
		{
			b3BuildFaceContact(manifold, xf1, faceQuery1.index, hull1, xf2, hull2, false);
			if (manifold.pointCount > 0)
			{
				// Write an overlap cache.
				cache = b3MakeCache(b3CacheType::e_overlap, b3FeatureType::e_face1, faceQuery1.index, faceQuery1.index);
				return;
			}
		}
	}
}

static void b3CollideHullsCache(b3Manifold& manifold,
	const b3Transform& xf1, const b3HullShape* hull1,
	const b3Transform& xf2, const b3HullShape* hull2,
	b3FeatureCache& cache, 
	const b3Transform& xf01, const b3Transform& xf02)
{
	const b3Hull* h1 = hull1->m_hull;
	scalar r1 = hull1->m_radius;

	const b3Hull* h2 = hull2->m_hull;
	scalar r2 = hull2->m_radius;

	scalar totalRadius = r1 + r2;

	// Read cache
	b3CacheType state0 = cache.cacheType;
	b3CacheType state1 = cache.ReadState(xf1, h1, xf2, h2, totalRadius);

	if (state0 == b3CacheType::e_separation && 
		state1 == b3CacheType::e_separation)
	{
		// Separation cache hit.
		++b3_convexCacheHits;
		return;
	}

	if (state0 == b3CacheType::e_overlap &&
		state1 == b3CacheType::e_overlap)
	{
		// Try to rebuild or reclip the features.
		switch (cache.featureType)
		{
		case b3FeatureType::e_edges:
		{
			b3RebuildEdgeContact(manifold, xf1, cache.index1, hull1, xf2, cache.index2, hull2);
			break;
		}
		case b3FeatureType::e_face1:
		{
			b3RebuildFaceContact(manifold, xf1, cache.index1, hull1, xf2, hull2, false, xf01, xf02);
			break;
		}
		case b3FeatureType::e_face2:
		{
			b3RebuildFaceContact(manifold, xf2, cache.index1, hull2, xf1, hull1, true, xf02, xf01);
			break;
		}
		default:
		{
			break;
		}
		}

		if (manifold.pointCount > 0)
		{
			// Overlap cache hit.
			++b3_convexCacheHits;
			return;
		}
	}

	// Separation cache miss.
	// Overlap cache miss.
	// Flush the cache.
	cache.cacheType = b3CacheType::e_empty;
	b3CollideHullsCache(manifold, xf1, hull1, xf2, hull2, cache);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void b3CollideHulls(b3Manifold& manifold,
	const b3Transform& xf1, const b3HullShape* hull1,
	const b3Transform& xf2, const b3HullShape* hull2,
	b3FeatureCache& cache, 
	const b3Transform& xf01, const b3Transform& xf02)
{
	++b3_convexCalls;
	if (b3_convexCache)
	{
		b3CollideHullsCache(manifold, xf1, hull1, xf2, hull2, cache, xf01, xf02);
	}
	else
	{
		b3CollideHulls(manifold, xf1, hull1, xf2, hull2);
	}
}