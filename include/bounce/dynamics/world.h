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

#ifndef B3_WORLD_H
#define B3_WORLD_H

#include <bounce/common/memory/block_allocator.h>
#include <bounce/common/memory/stack_allocator.h>
#include <bounce/dynamics/joint_manager.h>
#include <bounce/dynamics/contact_manager.h>
#include <bounce/dynamics/time_step.h>

struct b3BodyDef;
class b3Body;

class b3Fixture;
class b3Shape;

class b3QueryListener;
class b3QueryFilter;

class b3RayCastListener;
class b3RayCastFilter;

class b3ShapeCastListener;
class b3ShapeCastFilter;

class b3ContactListener;
class b3ContactFilter;

class b3Draw;
class b3Profiler;

// Output of b3World::RayCastSingle
struct b3RayCastSingleOutput
{
	b3Fixture* fixture; // fixture
	b3Vec3 point; // intersection point on surface
	b3Vec3 normal; // surface normal of intersection
	scalar fraction; // time of intersection on segment
};

// Output of b3World::ShapeCastSingle
struct b3ShapeCastSingleOutput
{
	b3Fixture* fixture; // fixture
	b3Vec3 point; // intersection point on surface
	b3Vec3 normal; // surface normal of intersection
	scalar fraction; // time of intersection on displacement
};

// Use a physics world to create/destroy rigid bodies and joints,
// perform ray and shape casts and also perform volume queries.
class b3World
{
public:
	b3World();
	~b3World();

	// The filter passed can tell the world to disallow the contact creation between 
	// two shapes.
	void SetContactFilter(b3ContactFilter* filter);

	// The listener passed will be notified when two body shapes begin/stays/ends
	// touching with each other.
	void SetContactListener(b3ContactListener* listener);
	
	// Set the debug draw interface. 
	// This interface is used for drawing the world entities.
	void SetDebugDraw(b3Draw* draw);

	// Set the profiler interface.
	void SetProfiler(b3Profiler* profiler);

	// Enable/disable body sleeping. This improves performance.
	void SetSleeping(bool flag);
	bool GetSleeping() const { return m_sleeping; }

	// Enable/disable warm starting. For testing.
	void SetWarmStarting(bool flag) { m_warmStarting = flag; }
	bool GetWarmStarting() const { return m_warmStarting; }

	// Set the acceleration due to the gravity force between this world and each dynamic 
	// body in the world. 
	// The acceleration has units of m/s^2.
	void SetGravity(const b3Vec3& gravity);

	// Get the acceleration due to gravity force in m/s^2.
	const b3Vec3& GetGravity() const;

	// Set flag to control automatic clearing of forces after each time step.
	void SetAutoClearForces(bool flag);

	// Get the flag that controls automatic clearing of forces after each time step.
	bool GetAutoClearForces() const;

	// Create a new rigid body.
	b3Body* CreateBody(const b3BodyDef& def);
	
	// Destroy an existing rigid body.
	void DestroyBody(b3Body* body);
	
	// Create a new joint.
	b3Joint* CreateJoint(const b3JointDef& def);

	// Remove a joint from the world and deallocate it from the memory.
	void DestroyJoint(b3Joint* joint);
	 
	// Simulate a physics step.
	// The function parameters are the ammount of time to simulate, 
	// and the number of constraint solver iterations.
	void Step(scalar dt, uint32 velocityIterations, uint32 positionIterations);
	
	// Manually clear the force buffer on all bodies. By default, forces are cleared automatically
	// after each call to Step. The default behavior is modified by calling SetAutoClearForces.
	void ClearForces();

	// Perform a ray cast with the world.
	// The given ray cast listener will be notified when a ray intersects a shape 
	// in the world. 
	// You can control on which shapes the ray-cast is performed using
	// a ray-cast filter.
	// The ray cast output is the intercepted shape, the intersection 
	// point in world space, the face normal on the shape associated with the point, 
	// and the intersection fraction.
	void RayCast(b3RayCastListener* listener, b3RayCastFilter* filter, const b3Vec3& point1, const b3Vec3& point2) const;

	// Perform a ray cast with the world.
	// If the ray doesn't intersect with a shape in the world then return false.
	// You can control on which shapes the ray-cast is performed using
	// a ray-cast filter.
	// The ray cast output is the intercepted shape, the intersection 
	// point in world space, the face normal on the shape associated with the point, 
	// and the intersection fraction.
	bool RayCastSingle(b3RayCastSingleOutput* output, b3RayCastFilter* filter, const b3Vec3& point1, const b3Vec3& point2) const;

	// Perform a shape cast with the world. This only works for given convex shapes.
	// You must supply a listener, filter, the shape, its transform and the displacement of the shape.
	// The given convex cast listener will be notified when a convex intersects a shape 
	// in the world. 
	// You can control on which shapes the convex-cast is performed using
	// a filter.
	void ShapeCast(b3ShapeCastListener* listener, b3ShapeCastFilter* filter, const b3Shape* shape, const b3Transform& xf, const b3Vec3& displacement) const;

	// Perform a shape cast with the world. This only works for given convex shapes.
	// You must supply a filter, the shape and the displacement of the shape.
	// If the convex doesn't intersect with any shape in the world then return false.
	// You can control on which shapes the convex-cast is performed using
	// the given filter.
	bool ShapeCastSingle(b3ShapeCastSingleOutput* output, b3ShapeCastFilter* filter, const b3Shape* shape, const b3Transform& xf, const b3Vec3& displacement) const;

	// Perform a AABB query with the world.
	// The query listener will be notified when two shape AABBs are overlapping.
	// You can control which shapes are reported using a query filter.
	// If the listener returns false then the query is stopped immediately.
	// Otherwise, it continues searching for new overlapping shape AABBs.
	void QueryAABB(b3QueryListener* listener, b3QueryFilter* filter, const b3AABB& aabb) const;

	// Get the head of the list of bodies in this world.
	const b3Body* GetBodyList() const;
	b3Body* GetBodyList();

	// Get the number of bodies in this world.
	uint32 GetBodyCount() const;

	// Get the list of joints in this world.
	const b3Joint* GetJointList() const;
	b3Joint* GetJointList();

	// Get the number of joints in this world.
	uint32 GetJointCount() const;

	// Get the list of contacts in this world.
	const b3Contact* GetContactList() const;
	b3Contact* GetContactList();
	
	// Get the number of contacts in this world.
	uint32 GetContactCount() const;

	// Draw the physics entities in this world.
	void DebugDraw() const;
private:
	friend class b3Body;
	friend class b3Fixture;
	friend class b3Contact;
	friend class b3ConvexContact;
	friend class b3MeshContact;
	friend class b3Joint;

	void Solve(const b3TimeStep& step);

	b3BlockAllocator m_blockAllocator;
	b3StackAllocator m_stackAllocator;

	b3ContactManager m_contactManager;
	b3JointManager m_jointManager;

	b3Body* m_bodyList;
	uint32 m_bodyCount;

	b3Vec3 m_gravity;
	bool m_sleeping;
	
	b3Draw* m_debugDraw;

	// This is used to compute the time step ratio to
	// support a variable time step.
	scalar m_inv_dt0;
	
	bool m_newContacts;
	bool m_clearForces;
	
	// These are for debugging the solver.
	bool m_warmStarting;

	b3Profiler* m_profiler;
};

inline void b3World::SetContactListener(b3ContactListener* listener)
{
	m_contactManager.m_contactListener = listener;
}

inline void b3World::SetContactFilter(b3ContactFilter* filter)
{
	m_contactManager.m_contactFilter = filter;
}

inline void b3World::SetDebugDraw(b3Draw* draw)
{
	m_debugDraw = draw;
}

inline void b3World::SetProfiler(b3Profiler* profiler)
{
	m_profiler = profiler;
	m_contactManager.m_profiler = m_profiler;
}

inline void b3World::SetGravity(const b3Vec3& gravity)
{
	m_gravity = gravity;
}

inline const b3Vec3& b3World::GetGravity() const
{
	return m_gravity;
}

inline void b3World::SetAutoClearForces(bool flag)
{
	m_clearForces = flag;
}

inline bool b3World::GetAutoClearForces() const
{
	return m_clearForces;
}

inline const b3Body* b3World::GetBodyList() const
{
	return m_bodyList;
}

inline b3Body* b3World::GetBodyList()
{
	return m_bodyList;
}

inline uint32 b3World::GetBodyCount() const
{
	return m_bodyCount;
}

inline const b3Joint* b3World::GetJointList() const
{
	return m_jointManager.m_jointList;
}

inline b3Joint* b3World::GetJointList()
{
	return m_jointManager.m_jointList;
}

inline uint32 b3World::GetJointCount() const
{
	return m_jointManager.m_jointCount;
}

inline const b3Contact* b3World::GetContactList() const
{
	return m_contactManager.m_contactList;
}

inline b3Contact* b3World::GetContactList()
{
	return m_contactManager.m_contactList;
}

inline uint32 b3World::GetContactCount() const
{
	return m_contactManager.m_contactCount;
}

#endif
