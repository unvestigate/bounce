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

#include <bounce/dynamics/contacts/hull_mesh_contact.h>
#include <bounce/collision/shapes/hull_shape.h>
#include <bounce/collision/shapes/mesh_shape.h>
#include <bounce/collision/shapes/triangle_shape.h>
#include <bounce/common/memory/block_allocator.h>

b3Contact* b3HullAndMeshContact::Create(b3Fixture* fixtureA, b3Fixture* fixtureB, b3BlockAllocator* allocator)
{
	void* mem = allocator->Allocate(sizeof(b3HullAndMeshContact));
	return new (mem) b3HullAndMeshContact(fixtureA, fixtureB);
}

void b3HullAndMeshContact::Destroy(b3Contact* contact, b3BlockAllocator* allocator)
{
	((b3HullAndMeshContact*)contact)->~b3HullAndMeshContact();
	allocator->Free(contact, sizeof(b3HullAndMeshContact));
}

b3HullAndMeshContact::b3HullAndMeshContact(b3Fixture* fixtureA, b3Fixture* fixtureB) : b3MeshContact(fixtureA, fixtureB, false)
{
	B3_ASSERT(fixtureA->GetType() == b3Shape::e_hull);
	B3_ASSERT(fixtureB->GetType() == b3Shape::e_mesh);
}

void b3HullAndMeshContact::Evaluate(b3Manifold& manifold, const b3Transform& xfA, const b3Transform& xfB, uint32 cacheIndex)
{
	B3_ASSERT(cacheIndex < m_triangleCount);

	b3MeshShape* mesh = (b3MeshShape*)m_fixtureB->GetShape();
	b3TriangleShape triangle;
	mesh->GetChildTriangle(&triangle, m_triangles[cacheIndex]);
	b3CollideHullAndTriangle(manifold, xfA, (b3HullShape*)m_fixtureA->GetShape(), xfB, &triangle);
}