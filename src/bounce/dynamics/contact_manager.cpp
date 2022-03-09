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

#include <bounce/dynamics/contact_manager.h>
#include <bounce/dynamics/contacts/contact.h>
#include <bounce/dynamics/body.h>
#include <bounce/dynamics/fixture.h>
#include <bounce/dynamics/world_callbacks.h>
#include <bounce/common/profiler.h>

b3ContactManager::b3ContactManager()
{
	m_contactList = nullptr;
	m_contactCount = 0;
	
	m_contactListener = nullptr;
	m_contactFilter = nullptr;
	
	m_allocator = nullptr;
	m_profiler = nullptr;
}

void b3ContactManager::AddPair(void* dataA, void* dataB)
{
	b3Fixture* fixtureA = (b3Fixture*)dataA;
	b3Fixture* fixtureB = (b3Fixture*)dataB;

	b3Body* bodyA = fixtureA->GetBody();
	b3Body* bodyB = fixtureB->GetBody();

	if (bodyA == bodyB)
	{
		// Two fixtures that belong to the same body cannot collide.
		return;
	}

	// Check if there is a contact between the two fixtures.
	// Search the list A or B. The shorter if possible.
	for (b3ContactEdge* ce = fixtureB->m_contactList; ce; ce = ce->next)
	{
		if (ce->other == fixtureA)
		{
			b3Contact* c = ce->contact;

			b3Fixture* fA = c->GetFixtureA();
			b3Fixture* fB = c->GetFixtureB();

			if (fA == fixtureA && fB == fixtureB)
			{
				// A contact already exists.
				return;
			}

			if (fA == fixtureB && fB == fixtureA)
			{
				// A contact already exists.
				return;
			}
		}
	}

	// Is at least one of the bodies dynamic? 
	// Does a joint prevent the collision?
	if (bodyA->ShouldCollide(bodyB) == false)
	{
		// The bodies must not collide with each other.
		return;
	}

	// Check if the contact filter prevents the collision.
	if (m_contactFilter)
	{
		if (m_contactFilter->ShouldCollide(fixtureA, fixtureB) == false)
		{
			return;
		}
	}

	// Call the factory.
	b3Contact* c = b3Contact::Create(fixtureA, fixtureB, m_allocator);
	if (c == nullptr)
	{
		return;
	}

	// Get the fixtures from the contact again because contact creation can swap the fixtures.
	fixtureA = c->GetFixtureA();
	fixtureB = c->GetFixtureB();

	// Insert into the world.
	c->m_prev = nullptr;
	c->m_next = m_contactList;
	if (m_contactList != nullptr)
	{
		m_contactList->m_prev = c;
	}
	m_contactList = c;

	// Connect to island graph.

	// Connect to fixture 1
	c->m_nodeA.contact = c;
	c->m_nodeA.other = fixtureB;

	c->m_nodeA.prev = nullptr;
	c->m_nodeA.next = fixtureA->m_contactList;
	if (fixtureA->m_contactList != nullptr)
	{
		fixtureA->m_contactList->prev = &c->m_nodeA;
	}
	fixtureA->m_contactList = &c->m_nodeA;

	// Connect to fixture 2
	c->m_nodeB.contact = c;
	c->m_nodeB.other = fixtureA;

	c->m_nodeB.prev = nullptr;
	c->m_nodeB.next = fixtureB->m_contactList;
	if (fixtureB->m_contactList != nullptr)
	{
		fixtureB->m_contactList->prev = &c->m_nodeB;
	}
	fixtureB->m_contactList = &c->m_nodeB;

	++m_contactCount;
}

void b3ContactManager::SynchronizeFixtures()
{
	b3Contact* c = m_contactList;
	while (c)
	{
		c->SynchronizeFixture();
		c = c->m_next;
	}
}

void b3ContactManager::FindNewContacts()
{
	m_broadPhase.FindPairs(this);

	b3Contact* c = m_contactList;
	while (c)
	{
		c->FindPairs();
		c = c->m_next;
	}
}

void b3ContactManager::UpdateContacts()
{
	B3_PROFILE(m_profiler, "Update Contacts");

	// Update the state of all contacts.
	b3Contact* c = m_contactList;
	while (c)
	{
		b3Fixture* fixtureA = c->m_fixtureA;
		b3Body* bodyA = fixtureA->m_body;

		b3Fixture* fixtureB = c->m_fixtureB;
		b3Body* bodyB = fixtureB->m_body;
		
		// Check if the bodies must not collide with each other.
		if (bodyA->ShouldCollide(bodyB) == false)
		{
			b3Contact* quack = c;
			c = c->m_next;
			Destroy(quack);
			continue;
		}

		// Check for external filtering.
		if (m_contactFilter)
		{
			if (m_contactFilter->ShouldCollide(fixtureA, fixtureB) == false)
			{
				// The user has stopped the contact.
				b3Contact* quack = c;
				c = c->m_next;
				Destroy(quack);
				continue;
			}
		}

		// At least one body must be dynamic or kinematic.
		bool activeA = bodyA->IsAwake() && bodyA->m_type != e_staticBody;
		bool activeB = bodyB->IsAwake() && bodyB->m_type != e_staticBody;
		if (activeA == false && activeB == false)
		{
			c = c->GetNext();
			continue;
		}

		u32 proxyB = fixtureB->m_proxyId;
		u32 proxyA = fixtureA->m_proxyId;
		bool overlap = m_broadPhase.TestOverlap(proxyA, proxyB);
		
		// Here we destroy contacts that cease to overlap in the broad-phase.
		if (overlap == false)
		{
			b3Contact* quack = c;
			c = c->GetNext();
			Destroy(quack);
			continue;
		}

		// The contact persists.
		c->Update(m_contactListener);
		c = c->GetNext();
	}
}

void b3ContactManager::Destroy(b3Contact* c)
{
	b3Fixture* fixtureA = c->GetFixtureA();
	b3Fixture* fixtureB = c->GetFixtureB();

	if (m_contactListener && c->IsTouching())
	{
		m_contactListener->EndContact(c);
	}

	// Remove from the world.
	if (c->m_prev)
	{
		c->m_prev->m_next = c->m_next;
	}

	if (c->m_next)
	{
		c->m_next->m_prev = c->m_prev;
	}

	if (c == m_contactList)
	{
		m_contactList = c->m_next;
	}

	// Remove from fixture 1
	if (c->m_nodeA.prev)
	{
		c->m_nodeA.prev->next = c->m_nodeA.next;
	}

	if (c->m_nodeA.next)
	{
		c->m_nodeA.next->prev = c->m_nodeA.prev;
	}

	if (&c->m_nodeA == fixtureA->m_contactList)
	{
		fixtureA->m_contactList = c->m_nodeA.next;
	}

	// Remove from fixture 2
	if (c->m_nodeB.prev)
	{
		c->m_nodeB.prev->next = c->m_nodeB.next;
	}

	if (c->m_nodeB.next)
	{
		c->m_nodeB.next->prev = c->m_nodeB.prev;
	}

	if (&c->m_nodeB == fixtureB->m_contactList)
	{
		fixtureB->m_contactList = c->m_nodeB.next;
	}

	// Call the factory.
	b3Contact::Destroy(c, m_allocator);
	--m_contactCount;
}