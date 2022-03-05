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

#ifndef B3_CONTACT_H
#define B3_CONTACT_H

#include <bounce/common/math/math.h>
#include <bounce/common/template/list.h>
#include <bounce/common/template/array.h>
#include <bounce/dynamics/fixture.h>
#include <bounce/collision/collide/manifold.h>

class b3BlockPool;

class b3Shape;
class b3Body;
class b3Contact;
class b3ContactListener;
class b3BlockAllocator;
struct b3ConvexCache;

// A contact edge for the contact graph, 
// where a fixture is a vertex and a contact 
// an edge.
struct b3ContactEdge
{
	b3Fixture* m_other;
	b3Contact* m_contact;
	b3ContactEdge* m_prev;
	b3ContactEdge* m_next;
};

typedef b3Contact* b3ContactCreateFcn(b3Fixture* shapeA, b3Fixture* shapeB, b3BlockAllocator* allocator);
typedef void b3ContactDestroyFcn(b3Contact* contact, b3BlockAllocator* allocator);

struct b3ContactRegister
{
	b3ContactCreateFcn* createFcn = nullptr;
	b3ContactDestroyFcn* destroyFcn = nullptr;
	bool primary;
};

class b3Contact
{
public:
	// Get the shape A in this contact.
	const b3Fixture* GetFixtureA() const;
	b3Fixture* GetFixtureA();

	// Get the fixture B in this contact.
	const b3Fixture* GetFixtureB() const;
	b3Fixture* GetFixtureB();

	// Get the manifold capacity from this contact.
	u32 GetManifoldCapacity() const;
	
	// Get a contact manifold from this contact.
	const b3Manifold* GetManifold(u32 index) const;
	b3Manifold* GetManifold(u32 index);

	// Get the number of manifolds from this contact.
	u32 GetManifoldCount() const;

	// Get a world contact manifold from this contact.
	void GetWorldManifold(b3WorldManifold* out, u32 index) const;
	
	// Are the shapes in this contact overlapping?
	bool IsOverlapping() const;

	// Has this contact at least one sensor shape?
	bool IsSensorContact() const;

	// Has this contact at least one dynamic body?
	bool HasDynamicBody() const;

	// Get the next contact in the world contact list.
	const b3Contact* GetNext() const;
	b3Contact* GetNext();
protected:
	friend class b3World;
	friend class b3Island;
	friend class b3Fixture;
	friend class b3ContactManager;
	friend class b3ContactSolver;
	friend class b3List<b3Contact>;

	// Flags
	enum 
	{
		e_overlapFlag = 0x0001,
		e_islandFlag = 0x0002,
	};

	b3Contact(b3Fixture* fixtureA, b3Fixture* fixtureB);
	virtual ~b3Contact() { }

	static b3ContactRegister s_registers[b3Shape::e_typeCount][b3Shape::e_typeCount];
	static bool s_initialized;
	
	static void AddType(b3ContactCreateFcn* createFcn, b3ContactDestroyFcn* destoryFcn,
		b3Shape::Type type1, b3Shape::Type type2);
	
	static void InitializeRegisters();

	// Factory create.
	static b3Contact* Create(b3Fixture* fixtureA, b3Fixture* fixtureB, b3BlockAllocator* allocator);
	
	// Factory destroy.
	static void Destroy(b3Contact* contact, b3BlockAllocator* allocator);

	// Update the contact state.
	void Update(b3ContactListener* listener);

	// Collide function.
	virtual void Collide() = 0;

	// Test if the shapes in this contact are overlapping.
	virtual bool TestOverlap() = 0;

	// Some contacts store reference AABBs for internal queries and therefore 
	// need to synchronize with body transforms.
	virtual void SynchronizeFixture() { }

	// Some contacts act like a midphase and therefore need to find 
	// new internal overlapping pairs.
	virtual void FindPairs() { }

	u32 m_flags;
	
	b3Fixture* m_fixtureA;
	b3Fixture* m_fixtureB;

	// To the fixture A edge list.
	b3ContactEdge m_edgeA;
	
	// To the fixture B edge list.
	b3ContactEdge m_edgeB;
	
	// Contact manifolds.
	u32 m_manifoldCapacity;
	b3Manifold* m_manifolds;
	u32 m_manifoldCount;

	// Links to the world contact list.
	b3Contact* m_prev;
	b3Contact* m_next;
};

inline b3Fixture* b3Contact::GetFixtureA() 
{
	return m_fixtureA;
}

inline const b3Fixture* b3Contact::GetFixtureA() const 
{
	return m_fixtureA;
}

inline b3Fixture* b3Contact::GetFixtureB()
{
	return m_fixtureB;
}

inline const b3Fixture* b3Contact::GetFixtureB() const
{
	return m_fixtureB;
}

inline u32 b3Contact::GetManifoldCapacity() const
{
	return m_manifoldCapacity;
}

inline const b3Manifold* b3Contact::GetManifold(u32 index) const
{
	B3_ASSERT(index < m_manifoldCount);
	return m_manifolds + index;
}

inline b3Manifold* b3Contact::GetManifold(u32 index)
{
	B3_ASSERT(index < m_manifoldCount);
	return m_manifolds + index;
}

inline u32 b3Contact::GetManifoldCount() const
{
	return m_manifoldCount;
}

inline bool b3Contact::IsOverlapping() const 
{
	return (m_flags & e_overlapFlag) != 0;
}

inline const b3Contact* b3Contact::GetNext() const
{
	return m_next;
}

inline b3Contact* b3Contact::GetNext()
{
	return m_next;
}

#endif
