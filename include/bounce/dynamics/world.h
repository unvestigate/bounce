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

#include <bounce/common/memory/stack_allocator.h>
#include <bounce/common/memory/block_allocator.h>
#include <bounce/common/template/list.h>
#include <bounce/dynamics/time_step.h>
#include <bounce/dynamics/joint_manager.h>
#include <bounce/dynamics/contact_manager.h>

struct b3BodyDef;

class b3Body;

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
	// Bit flags to tell the world what needs to be draw.
	enum 
	{
		e_shapesFlag = 0x0001,
		e_centerOfMassesFlag = 0x0002,
		e_jointsFlag = 0x0004,
		e_contactPointsFlag = 0x0008,
		e_contactNormalsFlag = 0x0010,
		e_contactTangentsFlag = 0x0020,
		e_contactPolygonsFlag = 0x0040,
		e_aabbsFlag = 0x0080,
	};
	
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
	void Step(scalar dt, u32 velocityIterations, u32 positionIterations);
	
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

	// Get the list of bodies in this world.
	const b3List<b3Body>& GetBodyList() const;
	b3List<b3Body>& GetBodyList();

	// Get the list of joints in this world.
	const b3List<b3Joint>& GetJointList() const;
	b3List<b3Joint>& GetJointList();

	// Get the list of contacts in this world.
	const b3List<b3Contact>& GetContactList() const;
	b3List<b3Contact>& GetContactList();

	// Set the draw flags.
	void SetDrawFlags(u32 flags);
	
	// Append flags to the current draw flag.
	void AppendDrawFlags(u32 flags);
	
	// Clear draw flags.
	void ClearDrawFlags();
		
	// Draw the entities in this world.
	void Draw() const;
	
	// Draw solid the entities in this world.
	void DrawSolid() const;
private:
	friend class b3Body;
	friend class b3Fixture;
	friend class b3Contact;
	friend class b3ConvexContact;
	friend class b3MeshContact;
	friend class b3Joint;

	void Solve(scalar dt, u32 velocityIterations, u32 positionIterations);

	bool m_sleeping;
	bool m_warmStarting;
	bool m_newContacts;
	bool m_clearForces;
	b3Vec3 m_gravity;
	
	// Stack allocator
	b3StackAllocator m_stackAllocator;

	// Block allocator
	b3BlockAllocator m_blockAllocator;

	// List of bodies
	b3List<b3Body> m_bodyList;
	
	// List of joints
	b3JointManager m_jointManager;
	
	// List of contacts
	b3ContactManager m_contactManager;

	// Debug draw flags.
	u32 m_drawFlags;

	// Debug draw. 
	b3Draw* m_debugDraw;

	// Profiler.
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

inline const b3List<b3Body>& b3World::GetBodyList() const
{
	return m_bodyList;
}

inline b3List<b3Body>& b3World::GetBodyList()
{
	return m_bodyList;
}

inline const b3List<b3Joint>& b3World::GetJointList() const
{
	return m_jointManager.m_jointList;
}

inline b3List<b3Joint>& b3World::GetJointList()
{
	return m_jointManager.m_jointList;
}

inline const b3List<b3Contact>& b3World::GetContactList() const
{
	return m_contactManager.m_contactList;
}

inline b3List<b3Contact>& b3World::GetContactList()
{
	return m_contactManager.m_contactList;
}

inline void b3World::SetDrawFlags(u32 flags)
{
	m_drawFlags = flags;
}

inline void b3World::AppendDrawFlags(u32 flags)
{
	m_drawFlags |= flags;
}

inline void b3World::ClearDrawFlags()
{
	m_drawFlags = 0;	
}
	
#endif
