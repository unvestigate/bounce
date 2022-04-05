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

#include <bounce/dynamics/contacts/mesh_sphere_contact.h>
#include <bounce/collision/shapes/triangle_shape.h>
#include <bounce/collision/shapes/mesh_shape.h>
#include <bounce/collision/shapes/sphere_shape.h>
#include <bounce/common/memory/block_allocator.h>

b3Contact* b3MeshAndSphereContact::Create(b3Fixture* fixtureA, b3Fixture* fixtureB, b3BlockAllocator* allocator)
{
	void* mem = allocator->Allocate(sizeof(b3MeshAndSphereContact));
	return new (mem) b3MeshAndSphereContact(fixtureA, fixtureB);
}

void b3MeshAndSphereContact::Destroy(b3Contact* contact, b3BlockAllocator* allocator)
{
	((b3MeshAndSphereContact*)contact)->~b3MeshAndSphereContact();
	allocator->Free(contact, sizeof(b3MeshAndSphereContact));
}

b3MeshAndSphereContact::b3MeshAndSphereContact(b3Fixture* fixtureA, b3Fixture* fixtureB) : b3MeshContact(fixtureA, fixtureB)
{
	B3_ASSERT(fixtureA->GetType() == b3Shape::e_mesh);
	B3_ASSERT(fixtureB->GetType() == b3Shape::e_sphere);
}

void b3MeshAndSphereContact::Evaluate(b3Manifold& manifold, const b3Transform& xfA, const b3Transform& xfB, uint32 cacheIndex)
{
	B3_ASSERT(cacheIndex < m_triangleCount);
	
	b3MeshShape* mesh = (b3MeshShape*)m_fixtureA->GetShape();
	b3TriangleShape triangle;
	mesh->GetChildTriangle(&triangle, m_triangles[cacheIndex]);
	b3CollideTriangleAndSphere(manifold, xfA, &triangle, xfB, (b3SphereShape*)m_fixtureB->GetShape());
}