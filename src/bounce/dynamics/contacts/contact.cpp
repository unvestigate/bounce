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
#include <bounce/dynamics/contacts/triangle_hull_contact.h>
#include <bounce/dynamics/contacts/hull_sphere_contact.h>
#include <bounce/dynamics/contacts/hull_capsule_contact.h>
#include <bounce/dynamics/contacts/hull_contact.h>
#include <bounce/dynamics/contacts/mesh_sphere_contact.h>
#include <bounce/dynamics/contacts/mesh_capsule_contact.h>
#include <bounce/dynamics/contacts/mesh_hull_contact.h>
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
	AddType(b3TriangleAndHullContact::Create, b3TriangleAndHullContact::Destroy, b3Shape::e_triangle, b3Shape::e_hull);
	AddType(b3HullAndSphereContact::Create, b3HullAndSphereContact::Destroy, b3Shape::e_hull, b3Shape::e_sphere);
	AddType(b3HullAndCapsuleContact::Create, b3HullAndCapsuleContact::Destroy, b3Shape::e_hull, b3Shape::e_capsule);
	AddType(b3HullContact::Create, b3HullContact::Destroy, b3Shape::e_hull, b3Shape::e_hull);
	AddType(b3MeshAndSphereContact::Create, b3MeshAndSphereContact::Destroy, b3Shape::e_mesh, b3Shape::e_sphere);
	AddType(b3MeshAndCapsuleContact::Create, b3MeshAndCapsuleContact::Destroy, b3Shape::e_mesh, b3Shape::e_capsule);
	AddType(b3MeshAndHullContact::Create, b3MeshAndHullContact::Destroy, b3Shape::e_mesh, b3Shape::e_hull);
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

	for (u32 i = 0; i < contact->m_manifoldCount; ++i)
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
}

void b3Contact::GetWorldManifold(b3WorldManifold* out, u32 index) const
{
	B3_ASSERT(index < m_manifoldCount);
	b3Manifold* m = m_manifolds + index;

	const b3Fixture* fixtureA = GetFixtureA();
	const b3Shape* shapeA = fixtureA->GetShape();
	const b3Body* bodyA = fixtureA->GetBody();
	b3Transform xfA = bodyA->GetTransform();

	const b3Fixture* fixtureB = GetFixtureB();
	const b3Shape* shapeB = fixtureB->GetShape();
	const b3Body* bodyB = fixtureB->GetBody();
	b3Transform xfB = bodyB->GetTransform();

	out->Initialize(m, shapeA->m_radius, xfA, shapeB->m_radius, xfB);
}

void b3Contact::Update(b3ContactListener* listener)
{
	b3Fixture* fixtureA = GetFixtureA();
	b3Shape* shapeA = fixtureA->GetShape();
	b3Body* bodyA = fixtureA->GetBody();
	b3Transform xfA = bodyA->GetTransform();

	b3Fixture* fixtureB = GetFixtureB();
	b3Shape* shapeB = fixtureB->GetShape();
	b3Body* bodyB = fixtureB->GetBody();
	b3Transform xfB = bodyB->GetTransform();

	b3World* world = bodyA->GetWorld();

	b3StackAllocator* stack = &world->m_stackAllocator;

	bool wasOverlapping = IsOverlapping();
	bool isOverlapping = false;
	bool isSensorContact = IsSensorContact();
	bool isDynamicContact = HasDynamicBody();

	if (isSensorContact == true)
	{
		isOverlapping = TestOverlap();
		m_manifoldCount = 0;
	}
	else
	{
		// Copy the old contact points.
		u32 oldManifoldCount = m_manifoldCount;
		b3Manifold* oldManifolds = (b3Manifold*)stack->Allocate(oldManifoldCount * sizeof(b3Manifold));
		memcpy(oldManifolds, m_manifolds, oldManifoldCount * sizeof(b3Manifold));

		// Clear all contact points.
		m_manifoldCount = 0;
		for (u32 i = 0; i < m_manifoldCapacity; ++i)
		{
			m_manifolds[i].Initialize();
		}

		// Generate new contact points for the solver.
		Collide();

		// Initialize the new built contact points for warm starting the solver.
		if (world->m_warmStarting == true)
		{
			for (u32 i = 0; i < m_manifoldCount; ++i)
			{
				b3Manifold* m2 = m_manifolds + i;
				for (u32 j = 0; j < oldManifoldCount; ++j)
				{
					const b3Manifold* m1 = oldManifolds + j;
					m2->Initialize(*m1);
				}
			}
		}

		stack->Free(oldManifolds);

		// The shapes are overlapping if at least one contact 
		// point was built.
		for (u32 i = 0; i < m_manifoldCount; ++i)
		{
			if (m_manifolds[i].pointCount > 0)
			{
				isOverlapping = true;
				break;
			}
		}
	}

	// Wake the bodies associated with the shapes if the contact has began.
	if (isOverlapping != wasOverlapping)
	{
		bodyA->SetAwake(true);
		bodyB->SetAwake(true);
	}

	// Update the contact state.
	if (isOverlapping == true)
	{
		m_flags |= e_overlapFlag;
	}
	else
	{
		m_flags &= ~e_overlapFlag;
	}

	// Notify the contact listener the new contact state.
	if (listener != nullptr)
	{
		if (wasOverlapping == false && isOverlapping == true)
		{
			listener->BeginContact(this);
		}

		if (wasOverlapping == true && isOverlapping == false)
		{
			listener->EndContact(this);
		}

		if (isOverlapping == true && isDynamicContact == true && isSensorContact == false)
		{
			listener->PreSolve(this);
		}
	}
}

bool b3Contact::IsSensorContact() const
{
	return m_fixtureA->IsSensor() || m_fixtureB->IsSensor();
}

bool b3Contact::HasDynamicBody() const
{
	return m_fixtureA->m_body->m_type == e_dynamicBody || m_fixtureB->m_body->m_type == e_dynamicBody;
}