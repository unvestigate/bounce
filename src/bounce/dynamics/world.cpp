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

#include <bounce/dynamics/world.h>
#include <bounce/dynamics/body.h>
#include <bounce/dynamics/fixture.h>
#include <bounce/dynamics/island.h>
#include <bounce/dynamics/world_callbacks.h>
#include <bounce/dynamics/contacts/contact.h>
#include <bounce/dynamics/joints/joint.h>
#include <bounce/dynamics/time_step.h>
#include <bounce/collision/collide/collide.h>
#include <bounce/collision/time_of_impact.h>
#include <bounce/collision/gjk/gjk.h>
#include <bounce/collision/gjk/gjk_proxy.h>
#include <bounce/collision/shapes/mesh_shape.h>
#include <bounce/collision/geometry/mesh.h>
#include <bounce/common/draw.h>
#include <bounce/common/profiler.h>

extern uint32 b3_allocCalls, b3_maxAllocCalls;
extern uint32 b3_convexCalls, b3_convexCacheHits;
extern uint32 b3_gjkCalls, b3_gjkIters, b3_gjkMaxIters;
extern bool b3_convexCache;

b3World::b3World()
{
	m_bodyList = nullptr;
	m_bodyCount = 0;

	m_sleeping = false;
	m_warmStarting = true;

	m_newContacts = false;
	m_clearForces = true;

	m_gravity.Set(scalar(0), scalar(-9.8), scalar(0));
	
	m_inv_dt0 = scalar(0);

	m_contactManager.m_allocator = &m_blockAllocator;
	m_jointManager.m_allocator = &m_blockAllocator;
	
	m_debugDraw = nullptr;
	m_profiler = nullptr;
	
	b3_allocCalls = 0;
	b3_maxAllocCalls = 0;
	b3_gjkCalls = 0;
	b3_gjkIters = 0;
	b3_convexCalls = 0;
	b3_convexCacheHits = 0;
	b3_convexCache = true;
}

b3World::~b3World()
{
	// None of the objects use b3Alloc.
	b3_allocCalls = 0;
	b3_maxAllocCalls = 0;
	b3_gjkCalls = 0;
	b3_gjkIters = 0;
	b3_convexCalls = 0;
	b3_convexCacheHits = 0;
}

void b3World::SetSleeping(bool flag)
{
	m_sleeping = flag;
	if (m_sleeping == false)
	{
		for (b3Body* b = m_bodyList; b; b = b->m_next)
		{
			b->SetAwake(true);
		}
	}
}

b3Body* b3World::CreateBody(const b3BodyDef& def)
{
	void* mem = m_blockAllocator.Allocate(sizeof(b3Body));
	b3Body* b = new(mem) b3Body(def, this);
	
	// Add to body doubly linked list.
	b->m_prev = nullptr;
	b->m_next = m_bodyList;
	if (m_bodyList)
	{
		m_bodyList->m_prev = b;
	}
	m_bodyList = b;
	++m_bodyCount;

	return b;
}

void b3World::DestroyBody(b3Body* b)
{
	// Delete the attached joints.
	b3JointEdge* je = b->m_jointList;
	while (je)
	{
		b3JointEdge* je0 = je;
		je = je->next;

		DestroyJoint(je0->joint);

		b->m_jointList = je;
	}
	b->m_jointList = nullptr;

	// Delete the attached fixtures.
	b3Fixture* f = b->m_fixtureList;
	while (f)
	{
		b3Fixture* f0 = f;
		f = f->m_next;

		// Destroy the attached contacts.
		b3ContactEdge* ce = f0->m_contactList;
		while (ce)
		{
			b3ContactEdge* ce0 = ce;
			ce = ce->next;
			m_contactManager.Destroy(ce0->contact);
		}

		// Destroy broad-phase proxy.
		m_contactManager.m_broadPhase.DestroyProxy(f0->m_proxyId);

		f0->Destroy(&m_blockAllocator);
		f0->~b3Fixture();
		m_blockAllocator.Free(f0, sizeof(b3Fixture));

		b->m_fixtureList = f;
		b->m_fixtureCount -= 1;
	}
	b->m_fixtureList = nullptr;
	b->m_fixtureCount = 0;

	// Remove from body list
	if (b->m_prev)
	{
		b->m_prev->m_next = b->m_next;
	}
	
	if (b->m_next)
	{
		b->m_next->m_prev = b->m_prev;
	}

	if (b == m_bodyList)
	{
		m_bodyList = b->m_next;
	}

	--m_bodyCount;
	b->~b3Body();
	m_blockAllocator.Free(b, sizeof(b3Body));
}

b3Joint* b3World::CreateJoint(const b3JointDef& def)
{
	return m_jointManager.Create(&def);
}

void b3World::DestroyJoint(b3Joint* j)
{
	m_jointManager.Destroy(j);
}

void b3World::Step(scalar dt, uint32 velocityIterations, uint32 positionIterations)
{
	B3_PROFILE(m_profiler, "Step");

	// Clear statistics
	b3_allocCalls = 0;

	b3_convexCalls = 0;
	b3_convexCacheHits = 0;

	b3_gjkCalls = 0;
	b3_gjkIters = 0;
	b3_gjkMaxIters = 0;

	if (m_newContacts)
	{
		m_contactManager.FindNewContacts();
		m_newContacts = false;
	}
	
	b3TimeStep step;
	step.dt = dt;
	step.velocityIterations = velocityIterations;
	step.positionIterations = positionIterations;
	if (dt > scalar(0))
	{
		step.inv_dt = scalar(1) / dt;
	}
	else
	{
		step.inv_dt = scalar(0);
	}

	step.dtRatio = m_inv_dt0 * dt;
	
	step.warmStarting = m_warmStarting;

	// Update contacts. This is where some contacts are destroyed.
	m_contactManager.UpdateContacts();

	// Integrate velocities, clear forces and torques, solve constraints, integrate positions.
	if (step.dt > scalar(0))
	{
		Solve(step);
	}

	if (step.dt > scalar(0))
	{
		m_inv_dt0 = step.inv_dt;
	}

	if (m_clearForces)
	{
		ClearForces();
	}
}

void b3World::ClearForces()
{
	for (b3Body* body = m_bodyList; body; body = body->GetNext())
	{
		body->m_force.SetZero();
		body->m_torque.SetZero();
	}
}

void b3World::Solve(const b3TimeStep& step)
{
	B3_PROFILE(m_profiler, "Solve");

	// Create a worst case island.
	b3Island island(m_bodyCount,
		m_contactManager.m_contactCount,
		m_jointManager.m_jointCount,
		&m_stackAllocator,
		m_contactManager.m_contactListener,
		m_profiler);

	// Clear all visited flags for the depth first search.
	for (b3Body* b = m_bodyList; b; b = b->m_next)
	{
		b->m_flags &= ~b3Body::e_islandFlag;
	}
	for (b3Contact* c = m_contactManager.m_contactList; c; c = c->m_next)
	{
		c->m_flags &= ~b3Contact::e_islandFlag;
	}
	for (b3Joint* j = m_jointManager.m_jointList; j; j = j->m_next)
	{
		j->m_islandFlag = false;
	}

	// Build and simulate awake islands.
	uint32 stackSize = m_bodyCount;
	b3Body** stack = (b3Body * *)m_stackAllocator.Allocate(stackSize * sizeof(b3Body*));
	for (b3Body* seed = m_bodyList; seed; seed = seed->m_next)
	{
		// The seed must not be on an island.
		if (seed->m_flags & b3Body::e_islandFlag)
		{
			continue;
		}

		// Bodies that are sleeping are not solved for performance.
		if (seed->IsAwake() == false)
		{
			continue;
		}

		// The seed must be dynamic or kinematic.
		if (seed->GetType() == e_staticBody)
		{
			continue;
		}

		// Perform a depth first search on this body constraint graph.
		island.Clear();
		uint32 stackCount = 0;
		stack[stackCount++] = seed;
		seed->m_flags |= b3Body::e_islandFlag;

		while (stackCount > 0)
		{
			// Add this body to the island.
			b3Body* b = stack[--stackCount];
			island.Add(b);

			// Don't propagate islands across static bodies to keep them small.
			if (b->GetType() == e_staticBody)
			{
				continue;
			}

			// Make sure the body is awake (without resetting sleep timer).
			b->m_flags |= b3Body::e_awakeFlag;

			// Search all contacts connected to this body.
			for (b3Fixture* f = b->m_fixtureList; f; f = f->m_next)
			{
				for (b3ContactEdge* ce = f->m_contactList; ce; ce = ce->next)
				{
					b3Contact* contact = ce->contact;

					// The contact must not be on an island.
					if (contact->m_flags & b3Contact::e_islandFlag)
					{
						continue;
					}

					// The contact must be touching.
					if (contact->IsTouching() == false)
					{
						continue;
					}

					// Skip sensors. 
					bool sensorA = contact->m_fixtureA->m_isSensor;
					bool sensorB = contact->m_fixtureB->m_isSensor;
					if (sensorA || sensorB)
					{
						continue;
					}

					// Does a contact filter prevent the contact response?
					if (m_contactManager.m_contactFilter)
					{
						if (m_contactManager.m_contactFilter->ShouldRespond(contact->m_fixtureA, contact->m_fixtureB) == false)
						{
							continue;
						}
					}

					// Add contact to the island and mark it.
					island.Add(contact);
					contact->m_flags |= b3Contact::e_islandFlag;

					b3Body* other = ce->other->GetBody();

					// Skip adjacent vertex if it was visited.
					if (other->m_flags & b3Body::e_islandFlag)
					{
						continue;
					}

					// Add the other body to the island and propagate through it.
					B3_ASSERT(stackCount < stackSize);
					stack[stackCount++] = other;
					other->m_flags |= b3Body::e_islandFlag;
				}
			}

			// Search all joints connected to this body.
			for (b3JointEdge* je = b->m_jointList; je; je = je->next)
			{
				b3Joint* joint = je->joint;

				// The joint must not be on an island.
				if (joint->m_islandFlag == true)
				{
					continue;
				}

				b3Body* other = je->other;

				// Add joint to the island and mark it.
				island.Add(joint);
				joint->m_islandFlag = true;

				// The other body must not be on an island.
				if (other->m_flags & b3Body::e_islandFlag)
				{
					continue;
				}

				// Push the other body onto the stack and mark it.
				B3_ASSERT(stackCount < stackSize);
				stack[stackCount++] = other;
				other->m_flags |= b3Body::e_islandFlag;
			}
		}

		// Integrate velocities, clear forces and torques, solve constraints, integrate positions.
		island.Solve(step, m_gravity, m_sleeping);

		// Allow static bodies to participate in other islands.
		for (uint32 i = 0; i < island.m_bodyCount; ++i)
		{
			b3Body* b = island.m_bodies[i];
			if (b->m_type == e_staticBody)
			{
				b->m_flags &= ~b3Body::e_islandFlag;
			}
		}
	}

	m_stackAllocator.Free(stack);

	{
		B3_PROFILE(m_profiler, "Find New Pairs");

		for (b3Body* b = m_bodyList; b; b = b->m_next)
		{
			// If a body didn't participate on a island then it didn't move.
			if ((b->m_flags & b3Body::e_islandFlag) == 0)
			{
				continue;
			}

			if (b->GetType() == e_staticBody)
			{
				continue;
			}

			// Update fixtures for broad-phase.
			b->SynchronizeFixtures();
		}

		// Update fixtures for mid-phase.
		m_contactManager.SynchronizeFixtures();

		// Look for new contacts.
		m_contactManager.FindNewContacts();
	}
}

struct b3WorldRayCastWrapper
{
	scalar Report(const b3RayCastInput& input, uint32 proxyId)
	{
		// Get shape associated with the proxy.
		void* userData = broadPhase->GetUserData(proxyId);
		b3Fixture* fixture = (b3Fixture*)userData;

		// Does a ray-cast filter prevents the ray-cast?
		if (filter->ShouldRayCast(fixture) == false)
		{
			// Continue search from where we stopped.
			return input.maxFraction;
		}

		b3RayCastOutput output;
		bool hit = fixture->RayCast(&output, input);
		if (hit)
		{
			scalar fraction = output.fraction;
			b3Vec3 point = (scalar(1) - fraction) * input.p1 + fraction * input.p2;
			return listener->ReportFixture(fixture, point, output.normal, fraction);
		}

		// Continue search from where we stopped.
		return input.maxFraction;
	}

	b3RayCastListener* listener;
	b3RayCastFilter* filter;
	const b3BroadPhase* broadPhase;
};

void b3World::RayCast(b3RayCastListener* listener, b3RayCastFilter* filter, const b3Vec3& p1, const b3Vec3& p2) const
{
	b3WorldRayCastWrapper wrapper;
	wrapper.listener = listener;
	wrapper.filter = filter;
	wrapper.broadPhase = &m_contactManager.m_broadPhase;
	
	b3RayCastInput input;
	input.p1 = p1;
	input.p2 = p2;
	input.maxFraction = scalar(1);

	m_contactManager.m_broadPhase.RayCast(&wrapper, input);
}

struct b3WorldRayCastSingleWrapper
{
	scalar Report(const b3RayCastInput& input, uint32 proxyId)
	{
		// Get shape associated with the proxy.
		void* userData = broadPhase->GetUserData(proxyId);
		b3Fixture* fixture = (b3Fixture*)userData;

		// Does a ray-cast filter prevents the ray-cast?
		if (filter->ShouldRayCast(fixture) == false)
		{
			// Continue search from where we stopped.
			return input.maxFraction;
		}

		b3RayCastOutput output;
		bool hit = fixture->RayCast(&output, input);
		if (hit)
		{
			// Track minimum time of impact to require less memory.
			if (output.fraction < output0.fraction)
			{
				fixture0 = fixture;
				output0 = output;
			}
		}

		// Continue the search from where we stopped.
		return input.maxFraction;
	}

	b3Fixture* fixture0;
	b3RayCastOutput output0;
	const b3BroadPhase* broadPhase;
	b3RayCastFilter* filter;
};

bool b3World::RayCastSingle(b3RayCastSingleOutput* output, b3RayCastFilter* filter, const b3Vec3& p1, const b3Vec3& p2) const
{
	b3WorldRayCastSingleWrapper wrapper;
	wrapper.fixture0 = nullptr;
	wrapper.output0.fraction = B3_MAX_SCALAR;
	wrapper.broadPhase = &m_contactManager.m_broadPhase;
	wrapper.filter = filter;
	
	b3RayCastInput input;
	input.p1 = p1;
	input.p2 = p2;
	input.maxFraction = scalar(1);

	m_contactManager.m_broadPhase.RayCast(&wrapper, input);

	if (wrapper.fixture0)
	{
		scalar fraction = wrapper.output0.fraction;
		b3Vec3 point = (scalar(1) - fraction) * input.p1 + fraction * input.p2;

		output->fixture = wrapper.fixture0;
		output->point = point;
		output->normal = wrapper.output0.normal;
		output->fraction = fraction;

		return true;
	}

	return false;
}

struct b3WorldShapeCastQueryWrapper
{
	struct MeshQueryWrapper
	{
		bool Report(uint32 proxyId)
		{
			void* userData = wrapper->meshB->m_mesh->tree.GetUserData(proxyId);
			b3MeshTriangle* triangle = (b3MeshTriangle*)userData;
			uint32 triangleIndex = triangle->GetIndex();

			b3Body* bodyB = wrapper->fixtureB->GetBody();
			b3Transform xfB = bodyB->GetTransform();
			b3ShapeGJKProxy proxyB(wrapper->meshB, triangleIndex);

			b3TOIOutput toi = b3TimeOfImpact(wrapper->xfA, *wrapper->proxyA, wrapper->dA, xfB, proxyB, b3Vec3_zero);

			b3TOIOutput::State state = toi.state;
			scalar fraction = toi.t;

			if (state == b3TOIOutput::e_touching)
			{
				if (fraction > wrapper->maxFraction)
				{
					return true;
				}

				if (fraction < wrapper->fraction0)
				{
					wrapper->fraction0 = fraction;
					wrapper->fixture0 = wrapper->fixtureB;
					wrapper->childIndex0 = triangleIndex;
				}

				if (wrapper->listener)
				{
					b3Transform xf;
					xf.rotation = wrapper->xfA.rotation;
					xf.translation = wrapper->xfA.translation + fraction * wrapper->dA;

					b3Vec3 point, normal;
					wrapper->Evaluate(&point, &normal, xf, *wrapper->proxyA, xfB, proxyB);

					wrapper->maxFraction = wrapper->listener->ReportFixture(wrapper->fixtureB, point, normal, fraction);
					if (wrapper->maxFraction == scalar(0))
					{
						return false;
					}
				}

				return true;
			}

			return true;
		}

		b3WorldShapeCastQueryWrapper* wrapper;
	};

	bool Report(uint32 proxyId)
	{
		void* userData = broadPhase->GetUserData(proxyId);
		fixtureB = (b3Fixture*)userData;
		if (filter->ShouldShapeCast(fixtureB) == false)
		{
			return true;
		}

		b3Shape* shapeB = fixtureB->GetShape();
		b3Body* bodyB = fixtureB->GetBody();
		b3Transform xfB = bodyB->GetTransform();

		if (shapeB->GetType() == b3Shape::e_mesh)
		{
			meshB = (b3MeshShape*)fixtureB->GetShape();

			B3_ASSERT(meshB->m_scale.x != scalar(0));
			B3_ASSERT(meshB->m_scale.y != scalar(0));
			B3_ASSERT(meshB->m_scale.z != scalar(0));

			b3Vec3 inv_scale;
			inv_scale.x = scalar(1) / meshB->m_scale.x;
			inv_scale.y = scalar(1) / meshB->m_scale.y;
			inv_scale.z = scalar(1) / meshB->m_scale.z;

			// Compute the swept AABB in the space of the unscaled tree
			b3Transform xf1 = b3MulT(xfB, xfA);
			b3Transform xf2 = b3MulT(xfB, xfA2);

			b3AABB aabb1, aabb2;
			shapeA->ComputeAABB(&aabb1, xf1);
			shapeA->ComputeAABB(&aabb2, xf2);

			aabb1.Scale(inv_scale);
			aabb2.Scale(inv_scale);

			b3AABB aabb = b3Combine(aabb1, aabb2);

			MeshQueryWrapper wrapper;
			wrapper.wrapper = this;
			meshB->m_mesh->tree.QueryAABB(&wrapper, aabb);

			if (maxFraction == scalar(0))
			{
				return false;
			}

			return true;
		}

		// The shape B is convex.
		b3ShapeGJKProxy proxyB(shapeB, 0);

		b3TOIOutput toi = b3TimeOfImpact(xfA, *proxyA, dA, xfB, proxyB, b3Vec3_zero);

		b3TOIOutput::State state = toi.state;
		scalar fraction = toi.t;

		if (state == b3TOIOutput::e_touching)
		{
			if (fraction > maxFraction)
			{
				return true;
			}

			if (fraction < fraction0)
			{
				fraction0 = fraction;
				fixture0 = fixtureB;
				childIndex0 = 0;
			}

			if (listener)
			{
				b3Transform xf;
				xf.rotation = xfA.rotation;
				xf.translation = xfA.translation + fraction * dA;

				b3Vec3 point, normal;
				Evaluate(&point, &normal, xf, *proxyA, xfB, proxyB);

				maxFraction = listener->ReportFixture(fixtureB, point, normal, fraction);
				if (maxFraction == scalar(0))
				{
					return false;
				}
			}

			return true;
		}

		return true;
	}

	// Evaluate when touching.
	void Evaluate(b3Vec3* pointB, b3Vec3* normalB, 
		const b3Transform& xA, const b3GJKProxy& proxyA, const b3Transform& xB, const b3GJKProxy& proxyB) const
	{
		b3GJKOutput query = b3GJK(xA, proxyA, xB, proxyB, false);

		b3Vec3 pA = query.point1;
		b3Vec3 pB = query.point2;
		
		b3Vec3 normal = b3Normalize(pA - pB);

		*pointB = pB + proxyB.radius * normal;
		*normalB = normal;
	}

	b3ShapeCastListener* listener;
	b3ShapeCastFilter* filter;
	const b3BroadPhase* broadPhase;

	const b3Shape* shapeA;
	b3Transform xfA, xfA2;
	const b3ShapeGJKProxy* proxyA;
	b3Vec3 dA;
	scalar maxFraction;

	b3Fixture* fixtureB;
	b3MeshShape* meshB;

	b3Fixture* fixture0;
	uint32 childIndex0;
	scalar fraction0;
};

void b3World::ShapeCast(b3ShapeCastListener* listener, b3ShapeCastFilter* filter, 
	const b3Shape* shape, const b3Transform& xf, const b3Vec3& displacement) const
{
	// The shape must be convex.
	B3_ASSERT(shape->m_type != b3Shape::e_mesh);
	if (shape->m_type == b3Shape::e_mesh)
	{
		return;
	}

	b3ShapeGJKProxy proxyA(shape, 0);

	// Compute a AABB that covers the swept shape
	b3Transform xf2 = xf;
	xf2.translation += displacement;

	b3AABB aabb1, aabb2;
	shape->ComputeAABB(&aabb1, xf);
	shape->ComputeAABB(&aabb2, xf2);

	b3AABB aabb = b3Combine(aabb1, aabb2);

	b3WorldShapeCastQueryWrapper wrapper;
	wrapper.listener = listener;
	wrapper.filter = filter;
	wrapper.shapeA = shape;
	wrapper.xfA = xf;
	wrapper.xfA2 = xf2;
	wrapper.proxyA = &proxyA;
	wrapper.dA = displacement;
	wrapper.maxFraction = scalar(1);
	wrapper.fixture0 = nullptr;
	wrapper.fraction0 = B3_MAX_SCALAR;
	wrapper.childIndex0 = B3_MAX_U32;
	wrapper.broadPhase = &m_contactManager.m_broadPhase;

	// Query against the swept AABB
	m_contactManager.m_broadPhase.QueryAABB(&wrapper, aabb);
}

bool b3World::ShapeCastSingle(b3ShapeCastSingleOutput* output, b3ShapeCastFilter* filter, 
	const b3Shape* shape, const b3Transform& xf, const b3Vec3& displacement) const
{
	B3_ASSERT(shape->m_type != b3Shape::e_mesh);
	if (shape->m_type == b3Shape::e_mesh)
	{
		return false;
	}

	b3ShapeGJKProxy proxyA(shape, 0);
	
	b3Transform xf2 = xf;
	xf2.translation += displacement;

	b3AABB aabb1, aabb2;
	shape->ComputeAABB(&aabb1, xf);
	shape->ComputeAABB(&aabb2, xf2);

	b3AABB aabb = b3Combine(aabb1, aabb2);

	b3WorldShapeCastQueryWrapper wrapper;
	wrapper.listener = nullptr;
	wrapper.filter = filter;
	wrapper.proxyA = &proxyA;
	wrapper.xfA = xf;
	wrapper.xfA2 = xf2;
	wrapper.shapeA = shape;
	wrapper.dA = displacement;
	wrapper.maxFraction = scalar(1);
	wrapper.fixture0 = nullptr;
	wrapper.childIndex0 = B3_MAX_U32;
	wrapper.fraction0 = B3_MAX_SCALAR;
	wrapper.broadPhase = &m_contactManager.m_broadPhase;

	// Query against the swept AABB
	m_contactManager.m_broadPhase.QueryAABB(&wrapper, aabb);

	if (wrapper.fixture0 == nullptr)
	{
		return false;
	}

	b3ShapeGJKProxy proxyB(wrapper.fixture0->GetShape(), wrapper.childIndex0);

	b3Body* bodyB = wrapper.fixture0->GetBody();
	b3Transform xfB = bodyB->GetTransform();
	
	b3Transform xft;
	xft.translation = xf.translation + wrapper.fraction0 * displacement;
	xft.rotation = xf.rotation;

	b3Vec3 point, normal;
	wrapper.Evaluate(&point, &normal, xft, proxyA, xfB, proxyB);

	output->fixture = wrapper.fixture0;
	output->fraction = wrapper.fraction0;
	output->point = point;
	output->normal = normal;

	return true;
}

struct b3WorldQueryWrapper
{
	bool Report(uint32 proxyID)
	{
		b3Fixture* fixture = (b3Fixture*)broadPhase->GetUserData(proxyID);
		if (filter->ShouldReport(fixture))
		{
			return listener->ReportFixture(fixture);
		}
		return true;
	}

	b3QueryListener* listener;
	b3QueryFilter* filter;
	const b3BroadPhase* broadPhase;
};

void b3World::QueryAABB(b3QueryListener* listener, b3QueryFilter* filter, const b3AABB& aabb) const
{
	b3WorldQueryWrapper wrapper;
	wrapper.listener = listener;
	wrapper.filter = filter;
	wrapper.broadPhase = &m_contactManager.m_broadPhase;
	m_contactManager.m_broadPhase.QueryAABB(&wrapper, aabb);
}

void b3World::DebugDraw() const
{
	if (m_debugDraw == nullptr)
	{
		return;
	}

	uint32 flags = m_debugDraw->GetFlags();

	if (flags & b3Draw::e_centerOfMassesFlag)
	{
		for (b3Body* b = m_bodyList; b; b = b->m_next)
		{
			b3Transform xf;
			xf.rotation = b->m_sweep.orientation;
			xf.translation = b->m_sweep.worldCenter;
			m_debugDraw->DrawTransform(xf);
		}
	}

	if (flags & b3Draw::e_shapesFlag)
	{
		for (b3Body* b = m_bodyList; b; b = b->m_next)
		{
			b3Color color;
			if (b->IsAwake() == false)
			{
				color = b3Color(scalar(0.5), scalar(0.25), scalar(0.25), scalar(1));
			}
			else if (b->GetType() == e_staticBody)
			{
				color = b3Color(scalar(0.5), scalar(0.5), scalar(0.5), scalar(1));
			}
			else if (b->GetType() == e_dynamicBody)
			{
				color = b3Color(scalar(1), scalar(0.5), scalar(0.5), scalar(1));
			}
			else
			{
				color = b3Color(scalar(0.5), scalar(0.5), scalar(1), scalar(1));
			}

			for (b3Fixture* f = b->m_fixtureList; f; f = f->m_next)
			{
				f->Draw(m_debugDraw, b3Color_black);
				f->DrawSolid(m_debugDraw, color);
			}
		}
	}

	if (flags & b3Draw::e_aabbsFlag)
	{
		for (b3Body* b = m_bodyList; b; b = b->m_next)
		{
			for (b3Fixture* f = b->m_fixtureList; f; f = f->m_next)
			{
				const b3AABB& aabb = m_contactManager.m_broadPhase.GetFatAABB(f->m_proxyId);
				m_debugDraw->DrawAABB(aabb, b3Color_pink);
			}
		}
	}

	if (flags & b3Draw::e_jointsFlag)
	{
		for (b3Joint* j = m_jointManager.m_jointList; j; j = j->m_next)
		{
			j->Draw(m_debugDraw);
		}
	}

	for (b3Contact* c = m_contactManager.m_contactList; c; c = c->m_next)
	{
		uint32 manifoldCount = c->m_manifoldCount;
		const b3Manifold* manifolds = c->m_manifolds;

		for (uint32 i = 0; i < manifoldCount; ++i)
		{
			const b3Manifold* m = manifolds + i;
			b3WorldManifold wm;
			c->GetWorldManifold(&wm, i);

			b3Vec3 t1 = wm.tangent1;
			b3Vec3 t2 = wm.tangent2;

			b3Vec3 points[B3_MAX_MANIFOLD_POINTS];
			for (uint32 j = 0; j < m->pointCount; ++j)
			{
				const b3ManifoldPoint* mp = m->points + j;
				const b3WorldManifoldPoint* wmp = wm.points + j;

				b3Vec3 n = wmp->normal;
				b3Vec3 p = wmp->point;

				points[j] = p;

				if (flags & b3Draw::e_contactPointsFlag)
				{
					m_debugDraw->DrawPoint(p, scalar(4), mp->persisting ? b3Color_green : b3Color_red);
				}

				if (flags & b3Draw::e_contactNormalsFlag)
				{
					m_debugDraw->DrawSegment(p, p + n, b3Color_white);
				}
			}

			if (m->pointCount > 0)
			{
				b3Vec3 p = wm.center;
				b3Vec3 n = wm.normal;
				t1 = wm.tangent1;
				t2 = wm.tangent2;

				if (flags & b3Draw::e_contactNormalsFlag)
				{
					m_debugDraw->DrawSegment(p, p + n, b3Color_yellow);
				}

				if (flags & b3Draw::e_contactTangentsFlag)
				{
					m_debugDraw->DrawSegment(p, p + t1, b3Color_yellow);
					m_debugDraw->DrawSegment(p, p + t2, b3Color_yellow);
				}

				if (m->pointCount > 2)
				{
					if (flags & b3Draw::e_contactPolygonsFlag)
					{
						m_debugDraw->DrawSolidPolygon(wm.normal, points, sizeof(b3Vec3), m->pointCount, b3Color_pink, false);
					}
				}
			}
		}
	}
}