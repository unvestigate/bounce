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

#include <bounce/collision/cluster.h>
#include <bounce/collision/collision.h>
#include <bounce/collision/geometry/geometry.h>

static B3_FORCE_INLINE bool b3IsCCW(const b3Vec3& A, const b3Vec3& B, const b3Vec3& C, const b3Vec3& N)
{
	b3Vec3 n = b3Cross(B - A, C - A);
	return b3Dot(n, N) > scalar(0);
}

static B3_FORCE_INLINE bool b3IsCCW(const b3Vec3& A, const b3Vec3& B, const b3Vec3& C, const b3Vec3& D, const b3Vec3& N)
{
	return b3IsCCW(A, B, C, N) && b3IsCCW(C, D, A, N);
}

void b3SortPolygon(b3ClusterPolygon& pOut,
	const b3ClusterPolygon& pIn, const b3Vec3& pNormal)
{
	B3_ASSERT(pOut.IsEmpty());
	B3_ASSERT(pIn.Count() > 0);
	
	pOut = pIn;

	b3Vec3 A = pOut[0].position;
	for (uint32 i = 1; i < pOut.Count(); ++i)
	{
		b3ClusterVertex& vi = pOut[i];
		b3Vec3 B = vi.position;
		for (uint32 j = i + 1; j < pOut.Count(); ++j)
		{
			b3ClusterVertex& vj = pOut[j];
			b3Vec3 C = vj.position;
			if (b3IsCCW(A, B, C, pNormal) == false)
			{
				b3Swap(vi, vj);
			}
		}
	}
}

void b3ReducePolygon(b3ClusterPolygon& pOut, 
	const b3ClusterPolygon& pIn, const b3Vec3& pNormal, uint32 initialPoint)
{
	B3_ASSERT(pIn.Count() > 0);
	B3_ASSERT(pOut.Count() == 0);
	B3_ASSERT(initialPoint < pIn.Count());

	if (pIn.Count() <= B3_MAX_MANIFOLD_POINTS)
	{
		b3SortPolygon(pOut, pIn, pNormal);
		return;
	}

	B3_ASSERT(pIn.Count() > B3_MAX_MANIFOLD_POINTS);

	b3StackArray<bool, 256> chosens;
	chosens.Resize(pIn.Count());
	for (uint32 i = 0; i < chosens.Count(); ++i)
	{
		chosens[i] = false;
	}

	{
		uint32 index = initialPoint;
		pOut.PushBack(pIn[index]);
		chosens[index] = true;
	}

	{
		b3Vec3 A = pOut[0].position;

		uint32 index = 0;
		scalar max = -B3_MAX_SCALAR;
		for (uint32 i = 0; i < pIn.Count(); ++i)
		{
			if (chosens[i]) { continue; }

			b3Vec3 B = pIn[i].position;
			b3Vec3 d = B - A;
			scalar dd = b3Dot(d, d);
			if (dd > max)
			{
				max = dd;
				index = i;
			}
		}

		// Coincidence check.
		if (max < B3_EPSILON * B3_EPSILON)
		{
			return;
		}

		pOut.PushBack(pIn[index]);
		chosens[index] = true;
	}

	{
		b3Vec3 A = pOut[0].position;
		b3Vec3 B = pOut[1].position;

		uint32 index = 0;
		scalar max = -B3_MAX_SCALAR;
		for (uint32 i = 0; i < pIn.Count(); ++i)
		{
			if (chosens[i]) { continue; }

			b3Vec3 C = pIn[i].position;
			b3Vec3 N = b3Cross(B - A, C - A);
			scalar sa2 = b3Dot(N, pNormal);
			scalar a2 = b3Abs(sa2);
			if (a2 > max)
			{
				max = a2;
				index = i;
			}
		}

		// Colinearity check.
		// Use wanky tolerance for reasonable performance.
		const scalar kAreaTol = scalar(0.01);
		if (max < scalar(2) * kAreaTol)
		{
			// Return the largest segment AB.
			return;
		}

		// Ensure CCW order of triangle ABC.
		b3Vec3 C = pIn[index].position;
		if (b3IsCCW(A, B, C, pNormal) == false)
		{
			b3Swap(pOut[0], pOut[1]);
		}

		pOut.PushBack(pIn[index]);
		chosens[index] = true;
	}

	{
		b3Vec3 A = pOut[0].position;
		b3Vec3 B = pOut[1].position;
		b3Vec3 C = pOut[2].position;

		B3_ASSERT(b3IsCCW(A, B, C, pNormal));

		uint32 index = 0;
		scalar min = B3_MAX_SCALAR;
		for (uint32 i = 0; i < pIn.Count(); ++i)
		{
			if (chosens[i]) { continue; }

			b3Vec3 D = pIn[i].position;
			b3Vec3 N = b3Cross(B - A, D - A);
			scalar sa2 = b3Dot(N, pNormal);
			if (sa2 < min)
			{
				min = sa2;
				index = i;
			}
		}

		// Colinearity check.
		const scalar kAreaTol = scalar(0.01);
		if (b3Abs(min) < scalar(2) * kAreaTol)
		{
			// Return the face ABC.
			return;
		}

		pOut.PushBack(pIn[index]);
		chosens[index] = true;
	}

	// Sort output polygon
	B3_ASSERT(pOut.Count() <= B3_MAX_MANIFOLD_POINTS);
	
	b3StackArray<b3ClusterVertex, B3_MAX_MANIFOLD_POINTS> quad;
	b3SortPolygon(quad, pOut, pNormal);
	
	// Output polygon
	pOut = quad;
}

b3ClusterSolver::b3ClusterSolver()
{
	m_iterations = 0;
}

void b3ClusterSolver::InitializeClusters()
{
	B3_ASSERT(m_clusters.IsEmpty());
	
	const uint32 kMaxClusters = 3;

	if (m_observations.Count() <= kMaxClusters)
	{
		for (uint32 i = 0; i < m_observations.Count(); ++i)
		{
			AddCluster(m_observations[i].point);
		}
		return;
	}

	B3_ASSERT(m_observations.Count() > 3);

	// This is used to skip observations that were 
	// used to initialize a cluster centroid.
	b3StackArray<bool, 256> chosens;
	chosens.Resize(m_observations.Count());
	for (uint32 i = 0; i < m_observations.Count(); ++i)
	{
		chosens[i] = false;
	}
	
	// Choose the most opposing faces.
	{
		uint32 index = 0;
		const b3Observation& o = m_observations[index];
		b3Vec3 A = o.point;
		AddCluster(A);
		chosens[index] = true;
	}

	{
		b3Vec3 A = m_clusters[0].centroid;

		uint32 index = 0;
		scalar max = -B3_MAX_SCALAR;
		for (uint32 i = 0; i < m_observations.Count(); ++i)
		{
			if (chosens[i])	{ continue;	}

			const b3Observation& o = m_observations[i];

			b3Vec3 B = o.point;
			scalar dd = b3DistanceSquared(A, B);
			if (dd > max)
			{
				max = dd;
				index = i;
			}
		}

		const b3Observation& o = m_observations[index];
		b3Vec3 B = o.point;
		AddCluster(B);
		chosens[index] = true;
	}

	B3_ASSERT(m_clusters.Count() == 1 || m_clusters.Count() == 2);

	if (m_clusters.Count() == 1)
	{
		b3Vec3 A = m_clusters[0].centroid;

		uint32 index = 0;
		scalar max = -B3_MAX_SCALAR;
		for (uint32 i = 0; i < m_observations.Count(); ++i)
		{
			if (chosens[i])	{ continue; }

			const b3Observation& o = m_observations[i];
			
			b3Vec3 B = o.point;

			scalar dd = b3DistanceSquared(A, B);
			if (dd > max)
			{
				max = dd;
				index = i;
			}
		}

		const b3Observation& o = m_observations[index];
		b3Vec3 B = o.point;
		AddCluster(B);
		chosens[index] = true;
		return;
	}

	B3_ASSERT(m_clusters.Count() == 2);

	{
		b3Vec3 A = m_clusters[0].centroid;
		b3Vec3 B = m_clusters[1].centroid;

		uint32 index = 0;
		scalar max = -B3_MAX_SCALAR;
		for (uint32 i = 0; i < m_observations.Count(); ++i)
		{
			if (chosens[i]) { continue; }

			const b3Observation& o = m_observations[i];
			
			b3Vec3 C = o.point;
			b3Vec3 Q = b3ClosestPointOnSegment(C, A, B);

			scalar dd = b3DistanceSquared(C, Q);
			if (dd > max)
			{
				max = dd;
				index = i;
			}
		}

		const b3Observation& o = m_observations[index];
		b3Vec3 C = o.point;
		AddCluster(C);
		chosens[index] = true;
	}
}

void b3ClusterSolver::AddCluster(const b3Vec3& centroid)
{
	if (m_clusters.IsEmpty())
	{
		// Add a new cluster
		b3Cluster c;
		c.centroid = centroid;
		m_clusters.PushBack(c);
		return;
	}

	uint32 bestIndex = FindCluster(centroid);
	b3Cluster& bestCluster = m_clusters[bestIndex];

	// Should we merge the cluster?
	const scalar kTol = scalar(0.05);
	
	if (b3DistanceSquared(centroid, bestCluster.centroid) <= kTol * kTol)
	{
		// Merge the clusters
		bestCluster.centroid += centroid;
		bestCluster.centroid.Normalize();
		return;
	}

	// Add a new cluster
	b3Cluster c;
	c.centroid = centroid;
	m_clusters.PushBack(c);
}

uint32 b3ClusterSolver::FindCluster(const b3Vec3& point) const
{
	uint32 bestIndex = 0;
	scalar bestValue = B3_MAX_SCALAR;
	for (uint32 i = 0; i < m_clusters.Count(); ++i)
	{
		b3Vec3 centroid = m_clusters[i].centroid;
		scalar metric = b3DistanceSquared(point, centroid);
		if (metric < bestValue)
		{
			bestValue = metric;
			bestIndex = i;
		}
	}
	return bestIndex;
}

static void b3MoveObservationsToCluster(b3Array<b3Observation>& observations, uint32 fromCluster, uint32 toCluster)
{
	for (uint32 i = 0; i < observations.Count(); ++i)
	{
		b3Observation& obs = observations[i];
		if (obs.cluster == fromCluster)
		{
			obs.cluster = toCluster;
		}
	}
}

void b3ClusterSolver::Solve()
{
	// Initialize clusters
	InitializeClusters();

	// Run k-means

	// Termination criteria 
	const uint32 kMaxIters = 10;

	uint32 iter = 0;
	while (iter < kMaxIters)
	{
		// Assign each observation to the closest cluster centroid.
		for (uint32 i = 0; i < m_observations.Count(); ++i)
		{
			b3Observation& obs = m_observations[i];
			obs.cluster = FindCluster(obs.point);
		}

		// Compute the new cluster centroids.
		for (uint32 i = 0; i < m_clusters.Count(); ++i)
		{
			b3Cluster& cluster = m_clusters[i];

			// Center
			b3Vec3 centroid;
			centroid.SetZero();

			uint32 pointCount = 0;
			for (uint32 j = 0; j < m_observations.Count(); ++j)
			{
				const b3Observation& obs = m_observations[j];
				if (obs.cluster == i)
				{
					centroid += obs.point;
					++pointCount;
				}
			}

			if (pointCount > 0)
			{
				centroid /= scalar(pointCount);
				cluster.centroid = centroid;
			}
		}

		++iter;
	}

	m_iterations = iter;
	
	// Remove empty clusters
	b3StackArray<b3Cluster, 256> usedClusters;

	for (uint32 i = 0; i < m_clusters.Count(); ++i)
	{
		// Count the observations in the ith cluster
		uint32 obsCount = 0;
		for (uint32 j = 0; j < m_observations.Count(); ++j)
		{
			const b3Observation& obs = m_observations[j];
			if (obs.cluster == i)
			{
				++obsCount;
			}
		}

		if (obsCount > 0)
		{
			// Copy the clusters observations into the new cluster.
			b3MoveObservationsToCluster(m_observations, i, usedClusters.Count());
			usedClusters.PushBack(m_clusters[i]);
		}
	}

	m_clusters = usedClusters;
}

void b3ClusterSolver::Run(b3Manifold outManifolds[3], uint32& numOut, 
	const b3Manifold* inManifolds, uint32 numIn,
	const b3Transform& xfA, scalar radiusA, const b3Transform& xfB, scalar radiusB)
{
	// Initialize observations
	for (uint32 i = 0; i < numIn; ++i)
	{
		b3WorldManifold wm;
		wm.Initialize(inManifolds + i, radiusA, xfA, radiusB, xfB);

		for (uint32 j = 0; j < wm.pointCount; ++j)
		{
			b3Observation o;
			o.point = wm.points[j].normal;
			o.cluster = B3_NULL_CLUSTER;
			o.manifold = i;
			o.manifoldPoint = j;
			
			m_observations.PushBack(o);
		}
	}

	// Solve
	Solve();

	// Reduce, weld, and output contact manifold

	B3_ASSERT(m_clusters.Count() <= B3_MAX_MANIFOLDS);

	for (uint32 i = 0; i < m_clusters.Count(); ++i)
	{		
		// Gather manifold points.
		b3Vec3 center;
		center.SetZero();
		
		b3Vec3 normal;
		normal.SetZero();

		b3StackArray<b3ClusterVertex, 64> polygonB;
		for (uint32 j = 0; j < m_observations.Count(); ++j)
		{
			b3Observation& o = m_observations[j];
			if (o.cluster != i)
			{
				continue;
			}

			const b3Manifold* m = inManifolds + o.manifold;
			const b3ManifoldPoint* mp = m->points + o.manifoldPoint;

			b3WorldManifoldPoint wmp;
			wmp.Initialize(mp, radiusA, xfA, radiusB, xfB);

			b3ClusterVertex cv;
			cv.position = wmp.point;
			cv.clipIndex = j;
			polygonB.PushBack(cv);

			center += wmp.point;
			normal += o.point;
		}

		if (polygonB.IsEmpty())
		{
			continue;
		}

		center /= scalar(polygonB.Count());
		normal.Normalize();

		B3_ASSERT(numOut < B3_MAX_MANIFOLDS);
		b3Manifold* manifold = outManifolds + numOut;
		++numOut;

		// Reduce.
		// Ensure deepest point is contained.
		uint32 minIndex = 0;
		scalar minSeparation = B3_MAX_SCALAR;
		for (uint32 j = 0; j < polygonB.Count(); ++j)
		{
			const b3Observation* o = m_observations.Get(polygonB[j].clipIndex);
			const b3Manifold* inManifold = inManifolds + o->manifold;
			const b3ManifoldPoint* inPoint = inManifold->points + o->manifoldPoint;

			b3WorldManifoldPoint wmp;
			wmp.Initialize(inPoint, radiusA, xfA, radiusB, xfB);

			scalar separation = wmp.separation;
			if (separation < minSeparation)
			{
				minIndex = j;
				minSeparation = separation;
			}

			polygonB[j].position = polygonB[j].position + separation * normal;
		}
		
		b3StackArray<b3ClusterVertex, B3_MAX_MANIFOLD_POINTS> quadB;
		b3ReducePolygon(quadB, polygonB, normal, minIndex);
		for (uint32 j = 0; j < quadB.Count(); ++j)
		{
			b3ClusterVertex v = quadB[j];
			uint32 inIndex = v.clipIndex;

			const b3Observation* o = m_observations.Get(inIndex);
			const b3Manifold* inManifold = inManifolds + o->manifold;
			const b3ManifoldPoint* inPoint = inManifold->points + o->manifoldPoint;

			manifold->points[j] = *inPoint;
		}

		manifold->pointCount = quadB.Count();
	}

	B3_ASSERT(numOut <= B3_MAX_MANIFOLDS);
}