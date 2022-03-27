/*
* Copyright (c) 2016-2019 Irlan Robson 
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applicatios, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#include <bounce/dynamics/contacts/mesh_contact.h>
#include <bounce/dynamics/fixture.h>
#include <bounce/dynamics/body.h>
#include <bounce/dynamics/world.h>
#include <bounce/collision/cluster.h>
#include <bounce/collision/shapes/mesh_shape.h>
#include <bounce/collision/geometry/mesh.h>

b3MeshContact::b3MeshContact(b3Fixture* fixtureA, b3Fixture* fixtureB) : b3Contact(fixtureA, fixtureB)
{
	m_manifoldCapacity = B3_MAX_MANIFOLDS;
	m_manifolds = m_clusterManifolds;
	m_manifoldCount = 0;
	
	b3Transform xfA = fixtureA->GetBody()->GetTransform();
	b3Transform xfB = fixtureB->GetBody()->GetTransform();

	b3Transform xf = b3MulT(xfA, xfB);

	// The aabb B relative to the mesh frame.
	b3AABB fatAABB;
	fixtureB->GetShape()->ComputeAABB(&fatAABB, xf);

	B3_ASSERT(fixtureA->GetType() == b3Shape::e_mesh);

	b3MeshShape* meshShapeA = (b3MeshShape*)fixtureA->m_shape;

	B3_ASSERT(meshShapeA->m_scale.x != scalar(0));
	B3_ASSERT(meshShapeA->m_scale.y != scalar(0));
	B3_ASSERT(meshShapeA->m_scale.z != scalar(0));

	b3Vec3 inv_scale;
	inv_scale.x = scalar(1) / meshShapeA->m_scale.x;
	inv_scale.y = scalar(1) / meshShapeA->m_scale.y;
	inv_scale.z = scalar(1) / meshShapeA->m_scale.z;

	fatAABB.Scale(inv_scale);

	fatAABB.Extend(B3_AABB_EXTENSION);

	m_aabbB = fatAABB;
	m_aabbBMoved = true;

	// Pre-allocate some indices
	m_triangleCapacity = 16;
	m_triangles = (b3TriangleCache*)b3Alloc(m_triangleCapacity * sizeof(b3TriangleCache));
	m_triangleCount = 0;
}

b3MeshContact::~b3MeshContact()
{
	b3Free(m_triangles);
}

void b3MeshContact::SynchronizeFixture()
{
	b3Shape* shapeA = m_fixtureA->GetShape();
	b3Body* bodyA = m_fixtureA->GetBody();
	b3Transform xfA = bodyA->GetTransform();

	b3Shape* shapeB = m_fixtureB->GetShape();
	b3Body* bodyB = m_fixtureB->GetBody();
	b3Transform xfB = bodyB->GetTransform();
	b3Sweep sweepB = bodyB->m_sweep;
	
	b3Transform xfB0;
	xfB0.rotation = sweepB.orientation0;
	xfB0.translation = sweepB.worldCenter0 - b3Mul(xfB0.rotation, sweepB.localCenter);

	b3Transform xf1 = b3MulT(xfA, xfB0);
	b3Transform xf2 = b3MulT(xfA, xfB);

	// Compute the AABBs B in the reference frame of the mesh.
	b3AABB aabb1, aabb2;
	shapeB->ComputeAABB(&aabb1, xf1);
	shapeB->ComputeAABB(&aabb2, xf2);

	b3MeshShape* meshShapeA = (b3MeshShape*)shapeA;

	B3_ASSERT(meshShapeA->m_scale.x != scalar(0));
	B3_ASSERT(meshShapeA->m_scale.y != scalar(0));
	B3_ASSERT(meshShapeA->m_scale.z != scalar(0));

	b3Vec3 inv_scale;
	inv_scale.x = scalar(1) / meshShapeA->m_scale.x;
	inv_scale.y = scalar(1) / meshShapeA->m_scale.y;
	inv_scale.z = scalar(1) / meshShapeA->m_scale.z;

	aabb1.Scale(inv_scale);
	aabb2.Scale(inv_scale);

	// Compute an AABB that covers the swept shape (may miss some rotation effect).
	b3AABB aabb = b3Combine(aabb1, aabb2);
	
	b3Vec3 displacement = aabb2.GetCenter() - aabb1.GetCenter();

	// Update the AABB with the new (transformed) AABB and buffer move.
	m_aabbBMoved = MoveAABB(aabb, displacement);
}

bool b3MeshContact::MoveAABB(const b3AABB& aabb, const b3Vec3& displacement)
{
	// Do nothing if the new AABB is contained in the old AABB.
	if (m_aabbB.Contains(aabb))
	{
		// Do nothing if the new AABB is contained in the old AABB.
		return false;
	}

	// Update the AABB with a fat and motion predicted AABB.

	// Extend the new (original) AABB.
	b3AABB fatAABB = aabb;
	fatAABB.Extend(B3_AABB_EXTENSION);

	if (displacement.x < scalar(0))
	{
		fatAABB.lowerBound.x += B3_AABB_MULTIPLIER * displacement.x;
	}
	else
	{
		fatAABB.upperBound.x += B3_AABB_MULTIPLIER * displacement.x;
	}

	if (displacement.y < scalar(0))
	{
		fatAABB.lowerBound.y += B3_AABB_MULTIPLIER * displacement.y;
	}
	else
	{
		fatAABB.upperBound.y += B3_AABB_MULTIPLIER * displacement.y;
	}

	if (displacement.z < scalar(0))
	{
		fatAABB.lowerBound.z += B3_AABB_MULTIPLIER * displacement.z;
	}
	else
	{
		fatAABB.upperBound.z += B3_AABB_MULTIPLIER * displacement.z;
	}

	// Update proxy with the extented AABB.
	m_aabbB = fatAABB;

	// Notify the proxy has moved.
	return true;
}

void b3MeshContact::FindPairs()
{
	// Reuse the overlapping buffer if the AABB didn't move
	// significantly.
	if (m_aabbBMoved == false)
	{
		return;
	}

	// Clear the index cache.
	m_triangleCount = 0;

	const b3MeshShape* meshShapeA = (b3MeshShape*)m_fixtureA->GetShape();
	const b3Mesh* meshA = meshShapeA->m_mesh;
	const b3StaticTree* treeA = &meshA->tree;

	// Query and update the overlapping buffer.
	treeA->QueryAABB(this, m_aabbB);
}

bool b3MeshContact::Report(uint32 proxyId)
{
	b3MeshShape* meshShapeA = (b3MeshShape*)m_fixtureA->GetShape();
	const b3Mesh* meshA = meshShapeA->m_mesh;
	const b3StaticTree* treeA = &meshA->tree;

	// Add the triangle to the overlapping buffer.
	if (m_triangleCount == m_triangleCapacity)
	{
		b3TriangleCache* oldTriangles = m_triangles;
		m_triangleCapacity *= 2;
		m_triangles = (b3TriangleCache*)b3Alloc(m_triangleCapacity * sizeof(b3TriangleCache));
		memcpy(m_triangles, oldTriangles, m_triangleCount * sizeof(b3TriangleCache));
		b3Free(oldTriangles);
	}

	B3_ASSERT(m_triangleCount < m_triangleCapacity);

	b3TriangleCache* cache = m_triangles + m_triangleCount;
	cache->index = treeA->GetIndex(proxyId);
	cache->cache.cacheType = b3CacheType::e_empty;

	++m_triangleCount;

	// Keep looking for triangles.
	return true;
}

bool b3MeshContact::TestOverlap()
{
	b3Shape* shapeA = m_fixtureA->GetShape();
	b3Body* bodyA = m_fixtureA->GetBody();
	b3Transform xfA = bodyA->GetTransform();

	b3Shape* shapeB = m_fixtureB->GetShape();
	b3Body* bodyB = m_fixtureB->GetBody();
	b3Transform xfB = bodyB->GetTransform();

	// Test if at least one triangle of the shape B overlaps the shape A.
	for (uint32 i = 0; i < m_triangleCount; ++i)
	{
		b3TriangleCache* cache = m_triangles + i;
		uint32 indexA = cache->index;
		bool overlap = b3TestOverlap(xfA, indexA, shapeA, xfB, 0, shapeB);
		if (overlap == true)
		{
			return true;
		}
	}

	return false;
}

void b3MeshContact::Collide()
{
	b3Shape* shapeA = m_fixtureA->GetShape();
	b3Body* bodyA = m_fixtureA->GetBody();
	b3Transform xfA = bodyA->GetTransform();

	b3Shape* shapeB = m_fixtureB->GetShape();
	b3Body* bodyB = m_fixtureB->GetBody();
	b3Transform xfB = bodyB->GetTransform();

	b3StackAllocator* allocator = &bodyA->m_world->m_stackAllocator;

	// Create one temporary manifold per overlapping triangle.
	b3Manifold* manifolds = (b3Manifold*)allocator->Allocate(m_triangleCount * sizeof(b3Manifold));
	uint32 manifoldCount = 0;

	for (uint32 i = 0; i < m_triangleCount; ++i)
	{
		b3Manifold* manifold = manifolds + manifoldCount;
		manifold->Initialize();

		Evaluate(*manifold, xfA, xfB, i);

		for (uint32 j = 0; j < manifold->pointCount; ++j)
		{
			manifold->points[j].id.triangleKey = m_triangles[i].index;
		}

		++manifoldCount;
	}

	B3_ASSERT(m_manifoldCount == 0);

	// Perform clustering. 
	b3ClusterSolver cluster;
	cluster.Run(m_clusterManifolds, m_manifoldCount, manifolds, manifoldCount, xfA, shapeA->m_radius, xfB, shapeB->m_radius);

	allocator->Free(manifolds);
}