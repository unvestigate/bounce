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

#include <bounce/collision/gjk/gjk.h>
#include <bounce/collision/gjk/gjk_proxy.h>
#include <bounce/collision/shapes/sphere_shape.h>
#include <bounce/collision/shapes/capsule_shape.h>
#include <bounce/collision/shapes/triangle_shape.h>
#include <bounce/collision/shapes/hull_shape.h>
#include <bounce/collision/shapes/mesh_shape.h>
#include <bounce/collision/geometry/hull.h>
#include <bounce/collision/geometry/mesh.h>

///////////////////////////////////////////////////////////////////////////////////////////////////

// Implementation of the GJK (Gilbert-Johnson-Keerthi) algorithm 
// using Voronoi regions and Barycentric coordinates.

uint32 b3_gjkCalls = 0, b3_gjkIters = 0, b3_gjkMaxIters = 0;
uint32 b3_gjkCacheHits = 0;

void b3GJKProxy::Set(const b3Shape* shape, uint32 index)
{
	switch (shape->GetType())
	{
	case b3Shape::e_sphere:
	{
		const b3SphereShape* sphere = (b3SphereShape*)shape;
		m_count = 1;
		m_vertices = &sphere->m_center;
		m_radius = sphere->m_radius;
		break;
	}
	case b3Shape::e_capsule:
	{
		const b3CapsuleShape* capsule = (b3CapsuleShape*)shape;
		m_count = 2;
		m_vertices = &capsule->m_vertex1;
		m_radius = capsule->m_radius;
		break;
	}
	case b3Shape::e_triangle:
	{
		const b3TriangleShape* triangle = (b3TriangleShape*)shape;
		m_count = 3;
		m_vertices = &triangle->m_vertex1;
		m_radius = triangle->m_radius;
		break;
	}
	case b3Shape::e_hull:
	{
		const b3HullShape* hull = (b3HullShape*)shape;
		m_count = hull->m_hull->vertexCount;
		m_vertices = hull->m_hull->vertices;
		m_radius = hull->m_radius;
		break;
	}
	case b3Shape::e_mesh:
	{
		const b3MeshShape* mesh = (b3MeshShape*)shape;
		B3_ASSERT(index < mesh->m_mesh->triangleCount);
		
		const b3MeshTriangle* triangle = mesh->m_mesh->GetTriangle(index);

		m_buffer[0] = b3Mul(mesh->m_scale, mesh->m_mesh->vertices[triangle->v1]);
		m_buffer[1] = b3Mul(mesh->m_scale, mesh->m_mesh->vertices[triangle->v2]);
		m_buffer[2] = b3Mul(mesh->m_scale, mesh->m_mesh->vertices[triangle->v3]);

		m_count = 3;
		m_vertices = m_buffer;
		m_radius = mesh->m_radius;
		break;
	}
	default:
	{
		B3_ASSERT(false);
		break;
	}
	}
}

void b3GJKProxy::Set(const b3Vec3* vertices, uint32 count, scalar radius)
{
	m_vertices = vertices;
	m_count = count;
	m_radius = radius;
}

// Convert a point Q from Cartesian coordinates to Barycentric coordinates (u, v) 
// with respect to a segment AB.
// The last output value is the divisor.
static B3_FORCE_INLINE void b3Barycentric(scalar out[3],
	const b3Vec3& A, const b3Vec3& B,
	const b3Vec3& Q)
{
	b3Vec3 AB = B - A;
	b3Vec3 QA = A - Q;
	b3Vec3 QB = B - Q;

	//scalar divisor = b3Dot(AB, AB);

	out[0] = b3Dot(QB, AB);
	out[1] = -b3Dot(QA, AB);
	out[2] = out[0] + out[1];
}

// Convert a point Q from Cartesian coordinates to Barycentric coordinates (u, v, w) 
// with respect to a triangle ABC.
// The last output value is the divisor.
static B3_FORCE_INLINE void b3Barycentric(scalar out[4],
	const b3Vec3& A, const b3Vec3& B, const b3Vec3& C,
	const b3Vec3& Q)
{
	b3Vec3 AB = B - A;
	b3Vec3 AC = C - A;

	b3Vec3 QA = A - Q;
	b3Vec3 QB = B - Q;
	b3Vec3 QC = C - Q;

	b3Vec3 QB_x_QC = b3Cross(QB, QC);
	b3Vec3 QC_x_QA = b3Cross(QC, QA);
	b3Vec3 QA_x_QB = b3Cross(QA, QB);

	b3Vec3 AB_x_AC = b3Cross(AB, AC);

	//scalar divisor = b3Dot(AB_x_AC, AB_x_AC);

	out[0] = b3Dot(QB_x_QC, AB_x_AC);
	out[1] = b3Dot(QC_x_QA, AB_x_AC);
	out[2] = b3Dot(QA_x_QB, AB_x_AC);
	out[3] = out[0] + out[1] + out[2];
}

// Convert a point Q from Cartesian coordinates to Barycentric coordinates (u, v, w, x) 
// with respect to a tetrahedron ABCD.
// The last output value is the (positive) divisor.
static B3_FORCE_INLINE void b3Barycentric(scalar out[5],
	const b3Vec3& A, const b3Vec3& B, const b3Vec3& C, const b3Vec3& D,
	const b3Vec3& Q)
{
	b3Vec3 AB = B - A;
	b3Vec3 AC = C - A;
	b3Vec3 AD = D - A;

	b3Vec3 QA = A - Q;
	b3Vec3 QB = B - Q;
	b3Vec3 QC = C - Q;
	b3Vec3 QD = D - Q;

	scalar divisor = b3Det(AB, AC, AD);
	scalar sign = b3Sign(divisor);

	out[0] = sign * b3Det(QB, QC, QD);
	out[1] = sign * b3Det(QA, QD, QC);
	out[2] = sign * b3Det(QA, QB, QD);
	out[3] = sign * b3Det(QA, QC, QB);
	out[4] = sign * divisor;
}

b3Vec3 b3Simplex::GetSearchDirection(const b3Vec3& Q) const
{
	switch (m_count)
	{
	case 1:
	{
		return Q - m_vertices[0].point;
	}
	case 2:
	{
		b3Vec3 A = m_vertices[0].point;
		b3Vec3 B = m_vertices[1].point;

		b3Vec3 AB = B - A;
		b3Vec3 AQ = Q - A;
		b3Vec3 AB_x_AQ = b3Cross(AB, AQ);
		b3Vec3 PQ = b3Cross(AB_x_AQ, AB);
		return PQ;
	}
	case 3:
	{
		b3Vec3 A = m_vertices[0].point;
		b3Vec3 B = m_vertices[1].point;
		b3Vec3 C = m_vertices[2].point;

		b3Vec3 AB = B - A;
		b3Vec3 AC = C - A;
		b3Vec3 AQ = Q - A;
		b3Vec3 AB_x_AC = b3Cross(AB, AC);
		scalar sign = b3Dot(AB_x_AC, AQ);
		if (sign > scalar(0))
		{
			return AB_x_AC;
		}
		else
		{
			return -AB_x_AC;
		}
	}
	default:
	{
		B3_ASSERT(false);
		return b3Vec3(scalar(0), scalar(0), scalar(0));
	}
	}
}

b3Vec3 b3Simplex::GetClosestPoint() const
{
	switch (m_count)
	{
	case 0:
		B3_ASSERT(false);
		return b3Vec3(scalar(0), scalar(0), scalar(0));
	case 1:
		return m_vertices[0].point;
	case 2:
		return m_vertices[0].weight * m_vertices[0].point + m_vertices[1].weight * m_vertices[1].point;
	case 3:
		return m_vertices[0].weight * m_vertices[0].point + m_vertices[1].weight * m_vertices[1].point + m_vertices[2].weight * m_vertices[2].point;
	case 4:
		return b3Vec3(scalar(0), scalar(0), scalar(0));
	default:
		B3_ASSERT(false);
		return b3Vec3(scalar(0), scalar(0), scalar(0));
	}
}

void b3Simplex::GetClosestPoints(b3Vec3* p1, b3Vec3* p2) const
{
	switch (m_count)
	{
	case 0:
		B3_ASSERT(false);
		break;
	case 1:
		*p1 = m_vertices[0].point1;
		*p2 = m_vertices[0].point2;
		break;

	case 2:
		*p1 = m_vertices[0].weight * m_vertices[0].point1 + m_vertices[1].weight * m_vertices[1].point1;
		*p2 = m_vertices[0].weight * m_vertices[0].point2 + m_vertices[1].weight * m_vertices[1].point2;
		break;

	case 3:
		*p1 = m_vertices[0].weight * m_vertices[0].point1 + m_vertices[1].weight * m_vertices[1].point1 + m_vertices[2].weight * m_vertices[2].point1;
		*p2 = m_vertices[0].weight * m_vertices[0].point2 + m_vertices[1].weight * m_vertices[1].point2 + m_vertices[2].weight * m_vertices[2].point2;
		break;
	case 4:
		*p1 = m_vertices[0].weight * m_vertices[0].point1 + m_vertices[1].weight * m_vertices[1].point1 + m_vertices[2].weight * m_vertices[2].point1 + m_vertices[3].weight * m_vertices[3].point1;
		*p2 = *p1;
		break;
	default:
		B3_ASSERT(false);
		break;
	}
}

// Closest point on edge AB to Q.
void b3Simplex::Solve2(const b3Vec3& Q)
{
	b3SimplexVertex A = m_vertices[0];
	b3SimplexVertex B = m_vertices[1];

	// Test vertex regions
	scalar wAB[3];
	b3Barycentric(wAB, A.point, B.point, Q);

	// R A
	if (wAB[1] <= scalar(0))
	{
		m_count = 1;
		m_vertices[0] = A;
		m_vertices[0].weight = scalar(1);
		return;
	}

	// R B
	if (wAB[0] <= scalar(0))
	{
		m_count = 1;
		m_vertices[0] = B;
		m_vertices[0].weight = scalar(1);
		return;
	}

	// R AB
	scalar divisor = wAB[2];
	scalar s = scalar(1) / divisor;
	m_count = 2;
	m_vertices[0] = A;
	m_vertices[0].weight = s * wAB[0];
	m_vertices[1] = B;
	m_vertices[1].weight = s * wAB[1];
}

// Closest point on face ABC to Q.
// Voronoi regions: A, B, C, AB, AC, BC, ABC, ACB.
void b3Simplex::Solve3(const b3Vec3& Q)
{
	b3SimplexVertex A = m_vertices[0];
	b3SimplexVertex B = m_vertices[1];
	b3SimplexVertex C = m_vertices[2];

	// Test vertex regions
	scalar wAB[3], wBC[3], wCA[3];
	b3Barycentric(wAB, A.point, B.point, Q);
	b3Barycentric(wBC, B.point, C.point, Q);
	b3Barycentric(wCA, C.point, A.point, Q);

	// R A
	if (wAB[1] <= scalar(0) && wCA[0] <= scalar(0))
	{
		m_count = 1;
		m_vertices[0] = A;
		m_vertices[0].weight = scalar(1);
		return;
	}

	// R B
	if (wAB[0] <= scalar(0) && wBC[1] <= scalar(0))
	{
		m_count = 1;
		m_vertices[0] = B;
		m_vertices[0].weight = scalar(1);
		return;
	}

	// R C
	if (wBC[0] <= scalar(0) && wCA[1] <= scalar(0))
	{
		m_count = 1;
		m_vertices[0] = C;
		m_vertices[0].weight = scalar(1);
		return;
	}

	// Test edge regions		
	scalar wABC[4];
	b3Barycentric(wABC, A.point, B.point, C.point, Q);

	// This is used to help testing if the face degenerates 
	// into an edge.
	scalar area = wABC[3];

	// R AB
	if (wAB[0] > scalar(0) && wAB[1] > scalar(0) && area * wABC[2] <= scalar(0))
	{
		scalar divisor = wAB[2];
		B3_ASSERT(divisor > scalar(0));
		scalar s = scalar(1) / divisor;
		m_count = 2;
		m_vertices[0] = A;
		m_vertices[0].weight = s * wAB[0];
		m_vertices[1] = B;
		m_vertices[1].weight = s * wAB[1];
		return;
	}

	// R BC
	if (wBC[0] > scalar(0) && wBC[1] > scalar(0) && area * wABC[0] <= scalar(0))
	{
		scalar divisor = wBC[2];
		B3_ASSERT(divisor > scalar(0));
		scalar s = scalar(1) / divisor;
		m_count = 2;
		m_vertices[0] = B;
		m_vertices[0].weight = s * wBC[0];
		m_vertices[1] = C;
		m_vertices[1].weight = s * wBC[1];
		return;
	}

	// R CA
	if (wCA[0] > scalar(0) && wCA[1] > scalar(0) && area * wABC[1] <= scalar(0))
	{
		scalar divisor = wCA[2];
		B3_ASSERT(divisor > scalar(0));
		scalar s = scalar(1) / divisor;
		m_count = 2;
		m_vertices[0] = C;
		m_vertices[0].weight = s * wCA[0];
		m_vertices[1] = A;
		m_vertices[1].weight = s * wCA[1];
		return;
	}

	// R ABC/ACB
	scalar divisor = wABC[3];
	if (divisor <= scalar(0))
	{
		// Give up.
		return;
	}

	B3_ASSERT(divisor > scalar(0));
	scalar s = scalar(1) / divisor;
	m_count = 3;
	m_vertices[0] = A;
	m_vertices[0].weight = s * wABC[0];
	m_vertices[1] = B;
	m_vertices[1].weight = s * wABC[1];
	m_vertices[2] = C;
	m_vertices[2].weight = s * wABC[2];
}

// Closest point on tetrahedron A, B, C, D to Q.
void b3Simplex::Solve4(const b3Vec3& Q)
{
	b3SimplexVertex A = m_vertices[0];
	b3SimplexVertex B = m_vertices[1];
	b3SimplexVertex C = m_vertices[2];
	b3SimplexVertex D = m_vertices[3];

	// Test vertex regions
	scalar wAB[3], wAC[3], wAD[3], wBC[3], wCD[3], wDB[3];
	b3Barycentric(wAB, A.point, B.point, Q);
	b3Barycentric(wBC, B.point, C.point, Q);
	b3Barycentric(wAC, A.point, C.point, Q);
	b3Barycentric(wAD, A.point, D.point, Q);
	b3Barycentric(wCD, C.point, D.point, Q);
	b3Barycentric(wDB, D.point, B.point, Q);

	// R A
	if (wAB[1] <= scalar(0) && wAC[1] <= scalar(0) && wAD[1] <= scalar(0))
	{
		m_count = 1;
		m_vertices[0] = A;
		m_vertices[0].weight = scalar(1);
		return;
	}

	// R B
	if (wAB[0] <= scalar(0) && wDB[0] <= scalar(0) && wBC[1] <= scalar(0))
	{
		m_count = 1;
		m_vertices[0] = B;
		m_vertices[0].weight = scalar(1);
		return;
	}

	// R C
	if (wAC[0] <= scalar(0) && wBC[0] <= scalar(0) && wCD[1] <= scalar(0))
	{
		m_count = 1;
		m_vertices[0] = C;
		m_vertices[0].weight = scalar(1);
		return;
	}

	// R D
	if (wAD[0] <= scalar(0) && wCD[0] <= scalar(0) && wDB[1] <= scalar(0))
	{
		m_count = 1;
		m_vertices[0] = D;
		m_vertices[0].weight = scalar(1);
		return;
	}

	// Test edge regions
	scalar wACB[4], wABD[4], wADC[4], wBCD[4];
	b3Barycentric(wACB, A.point, C.point, B.point, Q);
	b3Barycentric(wABD, A.point, B.point, D.point, Q);
	b3Barycentric(wADC, A.point, D.point, C.point, Q);
	b3Barycentric(wBCD, B.point, C.point, D.point, Q);

	// R AB
	if (wABD[2] <= scalar(0) && wACB[1] <= scalar(0) && wAB[0] > scalar(0) && wAB[1] > scalar(0))
	{
		scalar divisor = wAB[2];
		B3_ASSERT(divisor > scalar(0));
		scalar s = scalar(1) / divisor;
		m_count = 2;
		m_vertices[0] = A;
		m_vertices[0].weight = s * wAB[0];
		m_vertices[1] = B;
		m_vertices[1].weight = s * wAB[1];
		return;
	}

	// R AC
	if (wACB[2] <= scalar(0) && wADC[1] <= scalar(0) && wAC[0] > scalar(0) && wAC[1] > scalar(0))
	{
		scalar divisor = wAC[2];
		B3_ASSERT(divisor > scalar(0));
		scalar s = scalar(1) / divisor;
		m_count = 2;
		m_vertices[0] = A;
		m_vertices[0].weight = s * wAC[0];
		m_vertices[1] = C;
		m_vertices[1].weight = s * wAC[1];
		return;
	}

	// R AD
	if (wADC[2] <= scalar(0) && wABD[1] <= scalar(0) && wAD[0] > scalar(0) && wAD[1] > scalar(0))
	{
		scalar divisor = wAD[2];
		B3_ASSERT(divisor > scalar(0));
		scalar s = scalar(1) / divisor;
		m_count = 2;
		m_vertices[0] = A;
		m_vertices[0].weight = s * wAD[0];
		m_vertices[1] = D;
		m_vertices[1].weight = s * wAD[1];
		return;
	}

	// R BC
	if (wACB[0] <= scalar(0) && wBCD[2] <= scalar(0) && wBC[0] > scalar(0) && wBC[1] > scalar(0))
	{
		scalar divisor = wBC[2];
		B3_ASSERT(divisor > scalar(0));
		scalar s = scalar(1) / divisor;
		m_count = 2;
		m_vertices[0] = B;
		m_vertices[0].weight = s * wBC[0];
		m_vertices[1] = C;
		m_vertices[1].weight = s * wBC[1];
		return;
	}

	// R CD
	if (wADC[0] <= scalar(0) && wBCD[0] <= scalar(0) && wCD[0] > scalar(0) && wCD[1] > scalar(0))
	{
		scalar divisor = wCD[2];
		B3_ASSERT(divisor > scalar(0));
		scalar s = scalar(1) / divisor;
		m_count = 2;
		m_vertices[0] = C;
		m_vertices[0].weight = s * wCD[0];
		m_vertices[1] = D;
		m_vertices[1].weight = s * wCD[1];
		return;
	}

	// R DB
	if (wABD[0] <= scalar(0) && wBCD[1] <= scalar(0) &&  wDB[0] > scalar(0) && wDB[1] > scalar(0))
	{
		scalar divisor = wDB[2];
		B3_ASSERT(divisor > scalar(0));
		scalar s = scalar(1) / divisor;
		m_count = 2;
		m_vertices[0] = D;
		m_vertices[0].weight = s * wDB[0];
		m_vertices[1] = B;
		m_vertices[1].weight = s * wDB[1];
		return;
	}

	// Test face regions
	scalar wABCD[5];
	b3Barycentric(wABCD, A.point, B.point, C.point, D.point, Q);

	// R ACB 
	if (wABCD[3] <= scalar(0) && wACB[0] > scalar(0) && wACB[1] > scalar(0) && wACB[2] > scalar(0))
	{
		scalar divisor = wACB[0] + wACB[1] + wACB[2];
		B3_ASSERT(divisor > scalar(0));
		scalar s = scalar(1) / divisor;
		m_count = 3;
		m_vertices[0] = A;
		m_vertices[0].weight = s * wACB[0];
		m_vertices[1] = C;
		m_vertices[1].weight = s * wACB[1];
		m_vertices[2] = B;
		m_vertices[2].weight = s * wACB[2];
		return;
	}

	// R ABD
	if (wABCD[2] <= scalar(0) && wABD[0] > scalar(0) && wABD[1] > scalar(0) && wABD[2] > scalar(0))
	{
		scalar divisor = wABD[0] + wABD[1] + wABD[2];
		B3_ASSERT(divisor > scalar(0));
		scalar s = scalar(1) / divisor;
		m_count = 3;
		m_vertices[0] = A;
		m_vertices[0].weight = s * wABD[0];
		m_vertices[1] = B;
		m_vertices[1].weight = s * wABD[1];
		m_vertices[2] = D;
		m_vertices[2].weight = s * wABD[2];
		return;
	}

	// R ADC
	if (wABCD[1] <= scalar(0) && wADC[0] > scalar(0) && wADC[1] > scalar(0) && wADC[2] > scalar(0))
	{
		scalar divisor = wADC[0] + wADC[1] + wADC[2];
		B3_ASSERT(divisor > scalar(0));
		scalar s = scalar(1) / divisor;
		m_count = 3;
		m_vertices[0] = A;
		m_vertices[0].weight = s * wADC[0];
		m_vertices[1] = D;
		m_vertices[1].weight = s * wADC[1];
		m_vertices[2] = C;
		m_vertices[2].weight = s * wADC[2];
		return;
	}

	// R BCD
	if (wABCD[0] <= scalar(0) && wBCD[0] > scalar(0) && wBCD[1] > scalar(0) && wBCD[2] > scalar(0))
	{
		scalar divisor = wBCD[0] + wBCD[1] + wBCD[2];
		B3_ASSERT(divisor > scalar(0));
		scalar s = scalar(1) / divisor;
		m_count = 3;
		m_vertices[0] = B;
		m_vertices[0].weight = s * wBCD[0];
		m_vertices[1] = C;
		m_vertices[1].weight = s * wBCD[1];
		m_vertices[2] = D;
		m_vertices[2].weight = s * wBCD[2];
		return;
	}

	// R ABCD
	scalar divisor = wABCD[0] + wABCD[1] + wABCD[2] + wABCD[3];
	if (divisor <= scalar(0))
	{
		// Give up.
		return;
	}

	B3_ASSERT(divisor > scalar(0));
	scalar s = scalar(1) / divisor;
	m_count = 4;
	m_vertices[0].weight = s * wABCD[0];
	m_vertices[1].weight = s * wABCD[1];
	m_vertices[2].weight = s * wABCD[2];
	m_vertices[3].weight = s * wABCD[3];
}

///////////////////////////////////////////////////////////////////////////////////////////////////

b3GJKOutput b3GJK(const b3Transform& xf1, const b3GJKProxy& proxy1,
	const b3Transform& xf2, const b3GJKProxy& proxy2,
	bool applyRadius, b3SimplexCache* cache)
{
	++b3_gjkCalls;

	// Initialize the simplex.
	b3Simplex simplex;
	simplex.ReadCache(cache, xf1, proxy1, xf2, proxy2);

	// Get simplex vertices as an array.
	b3SimplexVertex* vertices = simplex.m_vertices;

	// These store the vertices of the last simplex so that we
	// can check for duplicates and prevent cycling.
	uint32 save1[4], save2[4];
	uint32 saveCount = 0;

	// Last iteration squared distance for checking if we're getting close
	// to the origin and prevent cycling.
	scalar distSq1 = B3_MAX_SCALAR;

	const b3Vec3 kOrigin(scalar(0), scalar(0), scalar(0));

	// Limit number of iterations to prevent cycling.
	const uint32 kMaxIters = 20;

	// Main iteration loop.
	uint32 iter = 0;
	while (iter < kMaxIters)
	{
		// Copy simplex so we can identify duplicates.
		saveCount = simplex.m_count;
		for (uint32 i = 0; i < saveCount; ++i)
		{
			save1[i] = vertices[i].index1;
			save2[i] = vertices[i].index2;
		}

		// Determine the closest point on the simplex and
		// remove unused vertices.
		switch (simplex.m_count)
		{
		case 1:
			break;
		case 2:
			simplex.Solve2(kOrigin);
			break;
		case 3:
			simplex.Solve3(kOrigin);
			break;
		case 4:
			simplex.Solve4(kOrigin);
			break;
		default:
			B3_ASSERT(false);
			break;
		}

		// If we have 4 points, then the origin is in the corresponding tethrahedron.
		if (simplex.m_count == 4)
		{
			break;
		}

		// Compute the closest point.
		b3Vec3 p = simplex.GetClosestPoint();
		scalar distSq2 = b3Dot(p, p);
		// Ensure we're getting close to the origin.
		if (distSq2 >= distSq1)
		{
			//break;
		}
		distSq1 = distSq2;

		// Get search direction.
		b3Vec3 d = simplex.GetSearchDirection(kOrigin);

		// Ensure the search direction is non-zero.
		if (b3Dot(d, d) < B3_EPSILON * B3_EPSILON)
		{
			break;
		}

		// Compute a tentative new simplex vertex using support points.
		b3SimplexVertex* vertex = vertices + simplex.m_count;
		vertex->index1 = proxy1.GetSupportIndex(b3MulC(xf1.rotation, -d));
		vertex->point1 = b3Mul(xf1, proxy1.GetVertex(vertex->index1));
		vertex->index2 = proxy2.GetSupportIndex(b3MulC(xf2.rotation, d));
		vertex->point2 = b3Mul(xf2, proxy2.GetVertex(vertex->index2));
		vertex->point = vertex->point2 - vertex->point1;

		// Iteration count is equated to the number of support point calls.
		++iter;
		++b3_gjkIters;

		// Check for duplicate support points. 
		// This is the main termination criteria.
		bool duplicate = false;
		for (uint32 i = 0; i < saveCount; ++i)
		{
			if (vertex->index1 == save1[i] && vertex->index2 == save2[i])
			{
				duplicate = true;
				break;
			}
		}

		// If we found a duplicate support point we must exit to avoid cycling.
		if (duplicate)
		{
			break;
		}

		// New vertex is ok and needed.
		++simplex.m_count;
	}

	b3_gjkMaxIters = b3Max(b3_gjkMaxIters, iter);

	// Prepare result.
	b3GJKOutput output;
	simplex.GetClosestPoints(&output.point1, &output.point2);
	output.distance = b3Distance(output.point1, output.point2);
	output.iterations = iter;

	// Cache the simplex.
	simplex.WriteCache(cache);

	// Apply radius if requested.
	if (applyRadius)
	{
		scalar r1 = proxy1.m_radius;
		scalar r2 = proxy2.m_radius;

		if (output.distance > r1 + r2 && output.distance > B3_EPSILON)
		{
			// Shapes are still no overlapped.
			// Move the witness points to the outer surface.
			output.distance -= r1 + r2;
			b3Vec3 d = output.point2 - output.point1;
			b3Vec3 normal = b3Normalize(d);
			output.point1 += r1 * normal;
			output.point2 -= r2 * normal;
		}
		else
		{
			// Shapes are overlapped when radii are considered.
			// Move the witness points to the middle.
			b3Vec3 p = scalar(0.5) * (output.point1 + output.point2);
			output.point1 = p;
			output.point2 = p;
			output.distance = scalar(0);
		}
	}

	// Output result.
	return output;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

// Implements b3Simplex routines for a cached simplex.
void b3Simplex::ReadCache(const b3SimplexCache* cache,
	const b3Transform& xf1, const b3GJKProxy& proxy1,
	const b3Transform& xf2, const b3GJKProxy& proxy2)
{
	B3_ASSERT(cache->count <= 4);
	m_count = cache->count;
	for (uint32 i = 0; i < m_count; ++i)
	{
		b3SimplexVertex* v = m_vertices + i;
		v->index1 = cache->index1[i];
		v->index2 = cache->index2[i];
		b3Vec3 w1Local = proxy1.GetVertex(v->index1);
		b3Vec3 w2Local = proxy2.GetVertex(v->index2);
		v->point1 = xf1 * w1Local;
		v->point2 = xf2 * w2Local;
		v->point = v->point2 - v->point1;
		v->weight = scalar(0);
	}

	// Compute the new simplex metric
	// If it is substantially different than
	// old metric then flush the simplex.
	if (m_count > 1)
	{
		scalar metric1 = cache->metric;
		scalar metric2 = GetMetric();
		if (metric2 < scalar(0.5) * metric1 || scalar(2) * metric1 < metric2 || metric2 < B3_EPSILON)
		{
			// Flush
			m_count = 0;
		}
		else
		{
			++b3_gjkCacheHits;
		}
	}

	// If cache is empty or flushed choose an arbitrary simplex.
	if (m_count == 0)
	{
		b3SimplexVertex* v = m_vertices + 0;
		b3Vec3 w1Local = proxy1.GetVertex(0);
		b3Vec3 w2Local = proxy2.GetVertex(0);
		v->point1 = b3Mul(xf1, w1Local);
		v->point2 = b3Mul(xf2, w2Local);
		v->point = v->point2 - v->point1;
		v->weight = scalar(1);
		v->index1 = 0;
		v->index2 = 0;
		m_count = 1;
	}
}

void b3Simplex::WriteCache(b3SimplexCache* cache) const
{
	cache->metric = GetMetric();
	cache->count = m_count;
	for (uint32 i = 0; i < m_count; ++i)
	{
		cache->index1[i] = m_vertices[i].index1;
		cache->index2[i] = m_vertices[i].index2;
	}
}

scalar b3Simplex::GetMetric() const
{
	switch (m_count)
	{
	case 0:
		B3_ASSERT(false);
		return scalar(0);
	case 1:
		return scalar(0);
	case 2:
		// Magnitude
		return b3Distance(m_vertices[0].point, m_vertices[1].point);
	case 3:
	{
		// Area
		b3Vec3 E1 = m_vertices[1].point - m_vertices[0].point;
		b3Vec3 E2 = m_vertices[2].point - m_vertices[0].point;
		return b3Length(b3Cross(E1, E2));
	}
	case 4:
	{
		// Volume
		b3Vec3 E1 = m_vertices[1].point - m_vertices[0].point;
		b3Vec3 E2 = m_vertices[2].point - m_vertices[0].point;
		b3Vec3 E3 = m_vertices[3].point - m_vertices[0].point;
		scalar det = b3Det(E1, E2, E3);
		scalar sign = b3Sign(det);
		scalar volume = sign * det;
		return volume;
	}
	default:
		B3_ASSERT(false);
		return scalar(0);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

b3GJKOutput b3GJK(const b3Transform& xf1, const b3GJKProxy& proxy1,
	const b3Transform& xf2, const b3GJKProxy& proxy2, bool applyRadius)
{
	b3SimplexCache cache;
	cache.count = 0;
	return b3GJK(xf1, proxy1, xf2, proxy2, applyRadius, &cache);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

// The output of the GJK-based shape cast algorithm.
struct b3ShapeCastOutput
{
	scalar t; // time of impact
	b3Vec3 point; // contact point at t
	b3Vec3 normal; // contact normal at t
	uint32 iterations; // number of iterations 
};

// Gino van der Bergen "Smooth Mesh Contacts with GJK"
// Game Physics Pearls 2010, page 99
bool b3ShapeCast(b3ShapeCastOutput* output,
	const b3Transform& xf1, const b3GJKProxy& proxy1,
	const b3Transform& xf2, const b3GJKProxy& proxy2, const b3Vec3& translation2)
{
	scalar r1 = proxy1.m_radius;
	scalar r2 = proxy2.m_radius;
	scalar radius = r1 + r2;

	b3Vec3 r = translation2;

	scalar t = scalar(0);
	b3Vec3 n = b3Vec3_zero;

	uint32 index1 = proxy1.GetSupportIndex(b3MulC(xf1.rotation, -r));
	uint32 index2 = proxy2.GetSupportIndex(b3MulC(xf2.rotation, r));
	b3Vec3 w1 = xf1 * proxy1.GetVertex(index1);
	b3Vec3 w2 = xf2 * proxy2.GetVertex(index2);
	b3Vec3 v = w1 - w2;

	b3Simplex simplex;
	simplex.m_count = 0;

	b3SimplexVertex* vertices = simplex.m_vertices;

	uint32 save1[4], save2[4];
	uint32 saveCount = 0;

	const uint32 kMaxIters = 20;
	const scalar kTolerance = scalar(10) * B3_EPSILON;

	scalar maxTolerance = scalar(1);

	uint32 iter = 0;
	while (iter < kMaxIters && b3Abs(b3LengthSquared(v) - radius * radius) > kTolerance * maxTolerance)
	{
		// Support in direction -v
		index1 = proxy1.GetSupportIndex(b3MulC(xf1.rotation, -v));
		index2 = proxy2.GetSupportIndex(b3MulC(xf2.rotation, v));
		w1 = xf1 * proxy1.GetVertex(index1);
		w2 = xf2 * proxy2.GetVertex(index2);
		b3Vec3 p = w1 - w2;

		// Support plane on boundary of CSO is (-v, p)
		// -v is normal at p
		scalar vp = b3Dot(v, p);
		scalar vr = b3Dot(v, r);

		if (vp - radius > t * vr)
		{
			if (vr > scalar(0))
			{
				t = (vp - radius) / vr;

				if (t > scalar(1))
				{
					output->iterations = iter;
					return false;
				}

				n = -v;

				// Flush the simplex
				simplex.m_count = 0;
				saveCount = 0;
			}
			else
			{
				output->iterations = iter;
				return false;
			}
		}

		// Unite p - s to simplex
		b3Vec3 s = t * r;

		b3SimplexVertex* vertex = vertices + simplex.m_count;
		vertex->index1 = index1;
		vertex->point1 = w1;
		vertex->index2 = index2;
		vertex->point2 = w2;
		vertex->point = p - s;

		// If we found a duplicate support point we must exit to avoid cycling.
		bool duplicate = false;
		for (uint32 i = 0; i < saveCount; ++i)
		{
			if (vertex->index1 == save1[i] && vertex->index2 == save2[i])
			{
				duplicate = true;
				break;
			}
		}

		if (duplicate)
		{
			break;
		}

		++simplex.m_count;

		// Compute tolerance
		maxTolerance = -B3_EPSILON;
		for (uint32 i = 0; i < simplex.m_count; ++i)
		{
			maxTolerance = b3Max(maxTolerance, b3LengthSquared(vertices[i].point));
		}

		// Copy simplex so we can identify duplicates.
		saveCount = simplex.m_count;
		for (uint32 i = 0; i < saveCount; ++i)
		{
			save1[i] = vertices[i].index1;
			save2[i] = vertices[i].index2;
		}

		// Sub-solve
		const b3Vec3 origin = b3Vec3_zero;

		switch (simplex.m_count)
		{
		case 1:
			break;
		case 2:
			simplex.Solve2(origin);
			break;
		case 3:
			simplex.Solve3(origin);
			break;
		case 4:
			simplex.Solve4(origin);
			break;
		default:
			B3_ASSERT(false);
			break;
		}

		if (simplex.m_count == 4)
		{
			break;
		}

		v = simplex.GetClosestPoint();

		++iter;
	}

	// Prepare output.
	b3Vec3 point1, point2;
	simplex.GetClosestPoints(&point1, &point2);

	if (b3LengthSquared(v) > B3_EPSILON * B3_EPSILON)
	{
		n = -v;
	}

	n.Normalize();

	output->t = t;
	output->point = point1 + r1 * n;
	output->normal = n;
	output->iterations = iter;
	return true;
}