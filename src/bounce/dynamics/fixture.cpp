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

#include <bounce/dynamics/fixture.h>
#include <bounce/dynamics/body.h>
#include <bounce/dynamics/world.h>
#include <bounce/dynamics/contacts/contact.h>
#include <bounce/collision/broad_phase.h>
#include <bounce/collision/shapes/sphere_shape.h>
#include <bounce/collision/shapes/capsule_shape.h>
#include <bounce/collision/shapes/triangle_shape.h>
#include <bounce/collision/shapes/hull_shape.h>
#include <bounce/collision/shapes/mesh_shape.h>
#include <bounce/collision/geometry/hull.h>
#include <bounce/collision/geometry/mesh.h>
#include <bounce/common/memory/block_allocator.h>
#include <bounce/common/draw.h>

b3Fixture::b3Fixture()
{
	m_body = nullptr;
	m_contactList = nullptr;
	m_next = nullptr;
	m_shape = nullptr;
	m_density = scalar(0);
}

void b3Fixture::Create(b3BlockAllocator* allocator, b3Body* body, const b3FixtureDef* def)
{
	m_userData = def->userData;
	m_friction = def->friction;
	m_restitution = def->restitution;
	
	m_body = body;
	
	m_isSensor = def->isSensor;
	
	m_shape = def->shape->Clone(allocator);
	
	m_density = def->density;
}

void b3Fixture::Destroy(b3BlockAllocator* allocator)
{
	// Free the child shape.
	switch (m_shape->m_type)
	{
	case b3Shape::e_sphere:
	{
		b3SphereShape* s = (b3SphereShape*)m_shape;
		s->~b3SphereShape();
		allocator->Free(s, sizeof(b3SphereShape));
		break;
	}
	case b3Shape::e_capsule:
	{
		b3CapsuleShape* s = (b3CapsuleShape*)m_shape;
		s->~b3CapsuleShape();
		allocator->Free(s, sizeof(b3CapsuleShape));
		break;
	}
	case b3Shape::e_triangle:
	{
		b3TriangleShape* s = (b3TriangleShape*)m_shape;
		s->~b3TriangleShape();
		allocator->Free(s, sizeof(b3TriangleShape));
		break;
	}
	case b3Shape::e_hull:
	{
		b3HullShape* s = (b3HullShape*)m_shape;
		s->~b3HullShape();
		allocator->Free(s, sizeof(b3HullShape));
		break;
	}
	case b3Shape::e_mesh:
	{
		b3MeshShape* s = (b3MeshShape*)m_shape;
		s->~b3MeshShape();
		allocator->Free(s, sizeof(b3MeshShape));
		break;
	}
	default:
	{
		B3_ASSERT(false);
		break;
	}
	}
	
	m_shape = nullptr;
}

void b3Fixture::SetSensor(bool flag)
{
	if (flag != m_isSensor)
	{
		if (m_body)
		{
			m_body->SetAwake(true);
		}
		m_isSensor = flag;
	}
}

void b3Fixture::Synchronize(b3BroadPhase* broadPhase, const b3Transform& transform1, const b3Transform& transform2)
{
	// Compute an AABB that covers the swept shape (may miss some rotation effect).
	b3AABB aabb1, aabb2;
	m_shape->ComputeAABB(&aabb1, transform1);
	m_shape->ComputeAABB(&aabb2, transform2);

	b3AABB aabb = b3Combine(aabb1, aabb2);

	b3Vec3 displacement = aabb2.GetCenter() - aabb1.GetCenter();

	broadPhase->MoveProxy(m_proxyId, aabb, displacement);
}

const b3AABB& b3Fixture::GetFatAABB() const
{
	return m_body->m_world->m_contactManager.m_broadPhase.GetFatAABB(m_proxyId);
}