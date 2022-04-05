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

#include <bounce/dynamics/contacts/contact.h>
#include <bounce/dynamics/contacts/sphere_contact.h>
#include <bounce/dynamics/contacts/capsule_sphere_contact.h>
#include <bounce/dynamics/contacts/capsule_contact.h>
#include <bounce/dynamics/contacts/triangle_sphere_contact.h>
#include <bounce/dynamics/contacts/triangle_capsule_contact.h>
#include <bounce/dynamics/contacts/hull_sphere_contact.h>
#include <bounce/dynamics/contacts/hull_capsule_contact.h>
#include <bounce/dynamics/contacts/hull_triangle_contact.h>
#include <bounce/dynamics/contacts/hull_contact.h>
#include <bounce/dynamics/contacts/hull_mesh_contact.h>
#include <bounce/dynamics/contacts/mesh_sphere_contact.h>
#include <bounce/dynamics/contacts/mesh_capsule_contact.h>
#include <bounce/dynamics/fixture.h>
#include <bounce/dynamics/body.h>
#include <bounce/dynamics/world.h>
#include <bounce/dynamics/world_callbacks.h>

bool b3Contact::s_initialized = false;
b3ContactRegister b3Contact::s_registers[b3Shape::e_typeCount][b3Shape::e_typeCount];

void b3Contact::AddType(b3ContactCreateFcn* createFcn, b3ContactDestroyFcn* destoryFcn,
	b3Shape::Type type1, b3Shape::Type type2)
{
	B3_ASSERT(0 <= type1 && type1 < b3Shape::e_typeCount);
	B3_ASSERT(0 <= type2 && type2 < b3Shape::e_typeCount);

	s_registers[type1][type2].createFcn = createFcn;
	s_registers[type1][type2].destroyFcn = destoryFcn;
	s_registers[type1][type2].primary = true;
	
	if (type1 != type2)
	{
		s_registers[type2][type1].createFcn = createFcn;
		s_registers[type2][type1].destroyFcn = destoryFcn;
		s_registers[type2][type1].primary = false;
	}
}

void b3Contact::InitializeRegisters()
{
	AddType(b3SphereContact::Create, b3SphereContact::Destroy, b3Shape::e_sphere, b3Shape::e_sphere);
	AddType(b3CapsuleAndSphereContact::Create, b3CapsuleAndSphereContact::Destroy, b3Shape::e_capsule, b3Shape::e_sphere);
	AddType(b3CapsuleContact::Create, b3CapsuleContact::Destroy, b3Shape::e_capsule, b3Shape::e_capsule);
	AddType(b3TriangleAndSphereContact::Create, b3TriangleAndSphereContact::Destroy, b3Shape::e_triangle, b3Shape::e_sphere);
	AddType(b3TriangleAndCapsuleContact::Create, b3TriangleAndCapsuleContact::Destroy, b3Shape::e_triangle, b3Shape::e_capsule);
	AddType(b3HullAndSphereContact::Create, b3HullAndSphereContact::Destroy, b3Shape::e_hull, b3Shape::e_sphere);
	AddType(b3HullAndCapsuleContact::Create, b3HullAndCapsuleContact::Destroy, b3Shape::e_hull, b3Shape::e_capsule);
	AddType(b3HullAndTriangleContact::Create, b3HullAndTriangleContact::Destroy, b3Shape::e_hull, b3Shape::e_triangle);
	AddType(b3HullContact::Create, b3HullContact::Destroy, b3Shape::e_hull, b3Shape::e_hull);
	AddType(b3HullAndMeshContact::Create, b3HullAndMeshContact::Destroy, b3Shape::e_hull, b3Shape::e_mesh);
	AddType(b3MeshAndSphereContact::Create, b3MeshAndSphereContact::Destroy, b3Shape::e_mesh, b3Shape::e_sphere);
	AddType(b3MeshAndCapsuleContact::Create, b3MeshAndCapsuleContact::Destroy, b3Shape::e_mesh, b3Shape::e_capsule);
}

b3Contact* b3Contact::Create(b3Fixture* fixtureA, b3Fixture* fixtureB, b3BlockAllocator* allocator)
{
	if (s_initialized == false)
	{
		InitializeRegisters();
		s_initialized = true;
	}

	b3Shape::Type type1 = fixtureA->GetType();
	b3Shape::Type type2 = fixtureB->GetType();

	B3_ASSERT(0 <= type1 && type1 < b3Shape::e_typeCount);
	B3_ASSERT(0 <= type2 && type2 < b3Shape::e_typeCount);

	const b3ContactRegister& contactRegister = s_registers[type1][type2];

	b3ContactCreateFcn* createFcn = contactRegister.createFcn;
	if (createFcn)
	{
		if (s_registers[type1][type2].primary)
		{
			return createFcn(fixtureA, fixtureB, allocator);
		}
		else
		{
			return createFcn(fixtureB, fixtureA, allocator);
		}
	}
	else
	{
		return nullptr;
	}
}

void b3Contact::Destroy(b3Contact* contact, b3BlockAllocator* allocator)
{
	B3_ASSERT(s_initialized == true);

	b3Fixture* fixtureA = contact->m_fixtureA;
	b3Fixture* fixtureB = contact->m_fixtureB;

	for (uint32 i = 0; i < contact->m_manifoldCount; ++i)
	{
		if (contact->m_manifolds[i].pointCount > 0)
		{
			if (fixtureA->IsSensor() == false && fixtureB->IsSensor() == false)
			{
				fixtureA->GetBody()->SetAwake(true);
				fixtureB->GetBody()->SetAwake(true);
				break;
			}
		}
	}

	b3Shape::Type type1 = fixtureA->GetType();
	b3Shape::Type type2 = fixtureB->GetType();

	B3_ASSERT(0 <= type1 && type1 < b3Shape::e_typeCount);
	B3_ASSERT(0 <= type2 && type2 < b3Shape::e_typeCount);

	const b3ContactRegister& contactRegister = s_registers[type1][type2];
	
	b3ContactDestroyFcn* destroyFcn = contactRegister.destroyFcn;
	destroyFcn(contact, allocator);
}

b3Contact::b3Contact(b3Fixture* fixtureA, b3Fixture* fixtureB)
{
	m_fixtureA = fixtureA;
	m_fixtureB = fixtureB;
	
	m_flags = 0;
	
	m_prev = nullptr;
	m_next = nullptr;

	m_nodeA.contact = nullptr;
	m_nodeA.prev = nullptr;
	m_nodeA.next = nullptr;
	m_nodeA.other = nullptr;

	m_nodeB.contact = nullptr;
	m_nodeB.prev = nullptr;
	m_nodeB.next = nullptr;
	m_nodeB.other = nullptr;

	m_manifoldCapacity = 0;
	m_manifoldCount = 0;
	m_manifolds = nullptr;
}

void b3Contact::GetWorldManifold(b3WorldManifold* out, uint32 index) const
{
	B3_ASSERT(index < m_manifoldCount);
	b3Manifold* m = m_manifolds + index;

	const b3Shape* shapeA = m_fixtureA->GetShape();
	const b3Body* bodyA = m_fixtureA->GetBody();
	b3Transform xfA = bodyA->GetTransform();

	const b3Shape* shapeB = m_fixtureB->GetShape();
	const b3Body* bodyB = m_fixtureB->GetBody();
	b3Transform xfB = bodyB->GetTransform();

	out->Initialize(m, shapeA->m_radius, xfA, shapeB->m_radius, xfB);
}

// Update the contact manifolds and touching status.
// Note: do not assume the fixture AABBs are overlapping or are valid.
void b3Contact::Update(b3ContactListener* listener)
{
	bool touching = false;
	bool wasTouching = (m_flags & e_touchingFlag) == e_touchingFlag;

	bool sensorA = m_fixtureA->IsSensor();
	bool sensorB = m_fixtureB->IsSensor();
	bool sensor = sensorA || sensorB;

	b3Body* bodyA = m_fixtureA->GetBody();
	b3Body* bodyB = m_fixtureB->GetBody();

	b3World* world = bodyA->m_world;
	b3StackAllocator* stack = &world->m_stackAllocator;

	if (sensor == true)
	{
		touching = TestOverlap();
		
		// Sensors don't generate manifolds.
		m_manifoldCount = 0;
	}
	else
	{
		// Copy the old contact points.
		uint32 oldManifoldCount = m_manifoldCount;
		b3Manifold* oldManifolds = (b3Manifold*)stack->Allocate(oldManifoldCount * sizeof(b3Manifold));
		memcpy(oldManifolds, m_manifolds, oldManifoldCount * sizeof(b3Manifold));

		// Clear all contact points.
		m_manifoldCount = 0;
		for (uint32 i = 0; i < m_manifoldCapacity; ++i)
		{
			m_manifolds[i].Initialize();
		}

		// Generate new contact points for the solver.
		Collide();

		// Initialize the new built contact points for warm starting the solver.
		if (world->m_warmStarting == true)
		{
			for (uint32 i = 0; i < m_manifoldCount; ++i)
			{
				b3Manifold* m2 = m_manifolds + i;
				for (uint32 j = 0; j < oldManifoldCount; ++j)
				{
					const b3Manifold* m1 = oldManifolds + j;
					m2->Initialize(*m1);
				}
			}
		}

		stack->Free(oldManifolds);

		// The shapes are overlapping if at least one contact 
		// point was built.
		for (uint32 i = 0; i < m_manifoldCount; ++i)
		{
			if (m_manifolds[i].pointCount > 0)
			{
				touching = true;
				break;
			}
		}

		// Wake the bodies associated with the shapes if the contact has began.
		if (touching != wasTouching)
		{
			bodyA->SetAwake(true);
			bodyB->SetAwake(true);
		}
	}

	// Update the contact state.
	if (touching == true)
	{
		m_flags |= e_touchingFlag;
	}
	else
	{
		m_flags &= ~e_touchingFlag;
	}

	// Notify the contact listener the new contact state.
	if (listener != nullptr)
	{
		if (wasTouching == false && touching == true)
		{
			listener->BeginContact(this);
		}

		if (wasTouching == true && touching == false)
		{
			listener->EndContact(this);
		}

		if (sensor == false && touching == true)
		{
			listener->PreSolve(this);
		}
	}
}