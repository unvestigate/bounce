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

#include <bounce/dynamics/body.h>
#include <bounce/dynamics/world.h>
#include <bounce/dynamics/fixture.h>
#include <bounce/dynamics/joints/joint.h>
#include <bounce/dynamics/contacts/contact.h>

b3Body::b3Body(const b3BodyDef& def, b3World* world) 
{
	m_world = world;
	m_prev = nullptr;
	m_next = nullptr;

	m_fixtureList = nullptr;
	m_fixtureCount = 0;

	m_jointList = nullptr;

	m_type = def.type;
	m_flags = 0;
	
	if (def.awake)
	{
		m_flags |= e_awakeFlag;
	}
	
	if (def.allowSleep)
	{
		m_flags |= e_autoSleepFlag;
	}

	if (m_type == e_dynamicBody) 
	{
		m_mass = scalar(1);
		m_invMass = scalar(1);

		if (def.fixedRotationX)
		{
			m_flags |= e_fixedRotationX;
		}

		if (def.fixedRotationY)
		{
			m_flags |= e_fixedRotationY;
		}

		if (def.fixedRotationZ)
		{
			m_flags |= e_fixedRotationZ;
		}
	}
	else 
	{
		m_mass = scalar(0);
		m_invMass = scalar(0);
	}
		
	m_I.SetZero();
	m_invI.SetZero();
	m_worldInvI.SetZero();

	m_force.SetZero();
	m_torque.SetZero();
	
	m_linearVelocity = def.linearVelocity;
	m_angularVelocity = def.angularVelocity;

	m_sweep.localCenter.SetZero();
	m_sweep.worldCenter = def.position;
	m_sweep.orientation = def.orientation;
	m_sweep.worldCenter0 = def.position;
	m_sweep.orientation0 = def.orientation;
	m_sweep.t0 = scalar(0);

	m_xf.translation = m_sweep.worldCenter;
	m_xf.rotation = m_sweep.orientation;
	
	m_linearDamping = def.linearDamping;
	m_angularDamping = def.angularDamping;
	m_gravityScale = def.gravityScale;
	m_userData = def.userData;
	
	m_linearSleepTolerance = def.linearSleepTolerance;
	m_angularSleepTolerance = def.angularSleepTolerance;
	m_sleepTime = scalar(0);	
}

b3Fixture* b3Body::CreateFixture(const b3FixtureDef& def) 
{
	b3BlockAllocator* allocator = &m_world->m_blockAllocator;
	
	void* memory = allocator->Allocate(sizeof(b3Fixture));
	b3Fixture* fixture = new (memory) b3Fixture();
	fixture->Create(allocator, this, &def);

	b3BroadPhase* broadPhase = &m_world->m_contactManager.m_broadPhase;
	
	b3AABB aabb;
	fixture->ComputeAABB(&aabb);
	fixture->m_proxyId = broadPhase->CreateProxy(aabb, fixture);

	fixture->m_next = m_fixtureList;
	m_fixtureList = fixture;
	++m_fixtureCount;

	fixture->m_body = this;

	// Adjust mass properties if needed.
	if (fixture->m_density > scalar(0))
	{
		ResetMass();
	}

	// Let the world know we have a new fixture. This will cause new contacts
	// to be created at the beginning of the next time step.
	m_world->m_newContacts = true;

	return fixture;
}

void b3Body::DestroyFixture(b3Fixture* fixture) 
{
	if (fixture == nullptr)
	{
		return;
	}

	B3_ASSERT(fixture->m_body == this);
	
	// Remove the fixture from this body's singly linked list.
	B3_ASSERT(m_fixtureCount > 0);
	b3Fixture** node = &m_fixtureList;
	bool found = false;
	while (*node != nullptr)
	{
		if (*node == fixture)
		{
			*node = fixture->m_next;
			found = true;
			break;
		}

		node = &(*node)->m_next;
	}

	// You tried to remove a shape that is not attached to this body.
	B3_ASSERT(found);

	// Destroy any contacts associated with the fixture.
	b3ContactEdge* edge = fixture->m_contactList;
	while (edge)
	{
		b3Contact* c = edge->contact;
		edge = edge->next;
		m_world->m_contactManager.Destroy(c);
	}

	b3BlockAllocator* allocator = &m_world->m_blockAllocator;

	// Destroy the broad-phase proxy associated with the fixture.
	b3BroadPhase* broadPhase = &m_world->m_contactManager.m_broadPhase;
	broadPhase->DestroyProxy(fixture->m_proxyId);
	
	// Destroy the fixture.
	fixture->m_body = nullptr;
	fixture->m_next = nullptr;
	fixture->Destroy(allocator);
	fixture->~b3Fixture();
	allocator->Free(fixture, sizeof(b3Fixture));

	--m_fixtureCount;

	// Recalculate the new inertial properties of this body.
	ResetMass();
}

void b3Body::SetTransform(const b3Vec3& position, const b3Quat& orientation)
{
	m_xf.translation = position;
	m_xf.rotation = orientation;

	m_sweep.worldCenter = b3Mul(m_xf, m_sweep.localCenter);
	m_sweep.orientation = m_xf.rotation;

	m_sweep.worldCenter0 = m_sweep.worldCenter;
	m_sweep.orientation0 = m_sweep.orientation;

	m_worldInvI = b3RotateToFrame(m_invI, m_xf.rotation);

	b3BroadPhase* broadPhase = &m_world->m_contactManager.m_broadPhase;
	for (b3Fixture* f = m_fixtureList; f; f = f->m_next)
	{
		f->Synchronize(broadPhase, m_xf, m_xf);
	}

	m_world->m_newContacts = true;
}

void b3Body::SetTransform(const b3Vec3& position, const b3Mat33& orientation)
{
	SetTransform(position, b3Mat33Quat(orientation));
}

void b3Body::SynchronizeFixtures() 
{
	b3BroadPhase* broadPhase = &m_world->m_contactManager.m_broadPhase;
	
	if (m_flags & e_awakeFlag)
	{
		b3Transform xf1;
		xf1.rotation = m_sweep.orientation0;
		xf1.translation = m_sweep.worldCenter0 - b3Mul(xf1.rotation, m_sweep.localCenter);

		for (b3Fixture* f = m_fixtureList; f; f = f->m_next)
		{
			f->Synchronize(broadPhase, xf1, m_xf);
		}
	}
	else
	{
		for (b3Fixture* f = m_fixtureList; f; f = f->m_next)
		{
			f->Synchronize(broadPhase, m_xf, m_xf);
		}
	}
}

bool b3Body::ShouldCollide(const b3Body* other) const
{
	// At least one body must be dynamic.
	if (m_type != e_dynamicBody && other->m_type != e_dynamicBody)
	{
		return false;
	}

	// Check if there are joints that connects this body with the other body 
	// and if the joint was configured to let not their collision occur.
	for (b3JointEdge* je = m_jointList; je; je = je->next)
	{
		b3Joint* j = je->joint;
		if (je->other == other)
		{
			if (j->m_collideConnected == false)
			{
				return false;
			}
		}
	}

	return true;
}

void b3Body::ResetMass() 
{
	m_mass = scalar(0);
	m_invMass = scalar(0);
	m_I.SetZero();
	m_invI.SetZero();
	m_worldInvI.SetZero();
	m_sweep.localCenter.SetZero();

	// Static and kinematic bodies have zero mass.
	if (m_type == e_staticBody || m_type == e_kinematicBody)
	{
		m_sweep.worldCenter0 = m_xf.translation;
		m_sweep.worldCenter = m_xf.translation;
		m_sweep.orientation0 = m_sweep.orientation;
		return;
	}

	// Accumulate the mass about the body origin of all fixtures.
	b3Vec3 localCenter;
	localCenter.SetZero();
	for (b3Fixture* f = m_fixtureList; f; f = f->m_next)
	{
		if (f->m_density == scalar(0)) 
		{
			continue;
		}
	
		b3MassData massData;
		f->ComputeMass(&massData);
		
		localCenter += massData.mass * massData.center;
		m_mass += massData.mass;
		m_I += massData.I;
	}

	if (m_mass > scalar(0)) 
	{
		// Compute local center of mass.
		m_invMass = scalar(1) / m_mass;
		localCenter *= m_invMass;

		// Shift inertia about the body origin into the body local center of mass.
		m_I = m_I - m_mass * b3Steiner(localCenter);
		
		// The moments of inertia must be positive.
		B3_ASSERT(m_I.x.x > scalar(0));
		B3_ASSERT(m_I.y.y > scalar(0));
		B3_ASSERT(m_I.z.z > scalar(0));

		// Compute inverse inertia about the body local center of mass.
		m_invI = b3Inverse(m_I);

		// Align the inverse inertia with the world frame of the body.
		m_worldInvI = b3RotateToFrame(m_invI, m_xf.rotation);

		// Fix rotation.
		if (m_flags & e_fixedRotationX)
		{
			m_invI.y.y = scalar(0);
			m_invI.z.y = scalar(0);
			m_invI.y.z = scalar(0);
			m_invI.z.z = scalar(0);

			m_worldInvI.y.y = scalar(0);
			m_worldInvI.z.y = scalar(0);
			m_worldInvI.y.z = scalar(0);
			m_worldInvI.z.z = scalar(0);
		}

		if (m_flags & e_fixedRotationY)
		{
			m_invI.x.x = scalar(0);
			m_invI.x.z = scalar(0);
			m_invI.z.x = scalar(0);
			m_invI.z.z = scalar(0);

			m_worldInvI.x.x = scalar(0);
			m_worldInvI.x.z = scalar(0);
			m_worldInvI.z.x = scalar(0);
			m_worldInvI.z.z = scalar(0);
		}

		if (m_flags & e_fixedRotationZ)
		{
			m_invI.x.x = scalar(0);
			m_invI.x.y = scalar(0);
			m_invI.y.x = scalar(0);
			m_invI.y.y = scalar(0);

			m_worldInvI.x.x = scalar(0);
			m_worldInvI.x.y = scalar(0);
			m_worldInvI.y.x = scalar(0);
			m_worldInvI.y.y = scalar(0);
		}
	}
	else 
	{
		// Force all dynamic bodies to have positive mass.
		m_mass = scalar(1);
		m_invMass = scalar(1);
	}

	// Move center of mass.
	b3Vec3 oldCenter = m_sweep.worldCenter;
	m_sweep.localCenter = localCenter;
	m_sweep.worldCenter = b3Mul(m_xf, m_sweep.localCenter);
	m_sweep.worldCenter0 = m_sweep.worldCenter;

	// Update center of mass velocity.
	m_linearVelocity += b3Cross(m_angularVelocity, m_sweep.worldCenter - oldCenter);
}

void b3Body::GetMassData(b3MassData* data) const
{
	data->mass = m_mass;
	data->I = m_I;
	data->center = m_sweep.localCenter;
}

void b3Body::SetMassData(const b3MassData* massData)
{
	if (m_type != e_dynamicBody)
	{
		return;
	}

	m_invMass = scalar(0);
	m_I.SetZero();
	m_invI.SetZero();
	m_worldInvI.SetZero();

	m_mass = massData->mass;
	if (m_mass > scalar(0))
	{
		m_invMass = scalar(1) / m_mass;
		m_I = massData->I - m_mass * b3Steiner(massData->center);
		
		//B3_ASSERT(m_I.x.x > scalar(0));
		//B3_ASSERT(m_I.y.y > scalar(0));
		//B3_ASSERT(m_I.z.z > scalar(0));

		m_invI = b3Inverse(m_I);
		m_worldInvI = b3RotateToFrame(m_invI, m_xf.rotation);

		if (m_flags & e_fixedRotationX)
		{
			m_invI.y.y = scalar(0);
			m_invI.z.y = scalar(0);
			m_invI.y.z = scalar(0);
			m_invI.z.z = scalar(0);

			m_worldInvI.y.y = scalar(0);
			m_worldInvI.z.y = scalar(0);
			m_worldInvI.y.z = scalar(0);
			m_worldInvI.z.z = scalar(0);
		}

		if (m_flags & e_fixedRotationY)
		{
			m_invI.x.x = scalar(0);
			m_invI.x.z = scalar(0);
			m_invI.z.x = scalar(0);
			m_invI.z.z = scalar(0);

			m_worldInvI.x.x = scalar(0);
			m_worldInvI.x.z = scalar(0);
			m_worldInvI.z.x = scalar(0);
			m_worldInvI.z.z = scalar(0);
		}

		if (m_flags & e_fixedRotationZ)
		{
			m_invI.x.x = scalar(0);
			m_invI.x.y = scalar(0);
			m_invI.y.x = scalar(0);
			m_invI.y.y = scalar(0);

			m_worldInvI.x.x = scalar(0);
			m_worldInvI.x.y = scalar(0);
			m_worldInvI.y.x = scalar(0);
			m_worldInvI.y.y = scalar(0);
		}
	}
	else
	{
		m_mass = scalar(1);
		m_invMass = scalar(1);
	}

	// Move center of mass.
	b3Vec3 oldCenter = m_sweep.worldCenter;
	m_sweep.localCenter = massData->center;
	m_sweep.worldCenter = b3Mul(m_xf, m_sweep.localCenter);
	m_sweep.worldCenter0 = m_sweep.worldCenter;

	// Update center of mass velocity.
	m_linearVelocity += b3Cross(m_angularVelocity, m_sweep.worldCenter - oldCenter);
}

void b3Body::SetType(b3BodyType type)
{
	if (m_type == type)
	{
		return;
	}

	m_type = type;

	ResetMass();

	if (m_type == e_staticBody)
	{
		m_linearVelocity.SetZero();
		m_angularVelocity.SetZero();
		m_sweep.worldCenter0 = m_sweep.worldCenter;
		m_sweep.orientation0 = m_sweep.orientation;
		SynchronizeFixtures();
	}

	SetAwake(true);

	m_force.SetZero();
	m_torque.SetZero();

	b3BroadPhase* broadPhase = &m_world->m_contactManager.m_broadPhase;
	for (b3Fixture* f = m_fixtureList; f; f = f->m_next)
	{
		// Delete the attached contacts.
		b3ContactEdge* edge = f->m_contactList;
		while (edge)
		{
			b3Contact* c = edge->contact;
			edge = edge->next;
			m_world->m_contactManager.Destroy(c);
		}

		// Touch the proxies so that new contacts will be created (when appropriate)
		broadPhase->TouchProxy(f->m_proxyId);
	}
}

void b3Body::SetFixedRotation(bool flagX, bool flagY, bool flagZ)
{
	bool statusX = (m_flags & e_fixedRotationX) == e_fixedRotationX;
	bool statusY = (m_flags & e_fixedRotationY) == e_fixedRotationY;
	bool statusZ = (m_flags & e_fixedRotationZ) == e_fixedRotationZ;

	if (statusX == flagX && statusY == flagY && statusZ == flagZ)
	{
		return;
	}

	if (flagX)
	{
		m_flags |= e_fixedRotationX;
	}
	else
	{
		m_flags &= ~e_fixedRotationX;
	}

	if (flagY)
	{
		m_flags |= e_fixedRotationY;
	}
	else
	{
		m_flags &= ~e_fixedRotationY;
	}
	
	if (flagZ)
	{
		m_flags |= e_fixedRotationZ;
	}
	else
	{
		m_flags &= ~e_fixedRotationZ;
	}
	
	m_angularVelocity.SetZero();

	ResetMass();
}

void b3Body::SetLinearSleepTolerance(scalar tolerance)
{
	B3_ASSERT(tolerance >= scalar(0));
	m_linearSleepTolerance = tolerance;
}

void b3Body::SetAngularSleepTolerance(scalar tolerance)
{
	B3_ASSERT(tolerance >= scalar(0));
	m_angularSleepTolerance = tolerance;
}

void b3Body::Dump() const
{
	uint32 bodyIndex = m_islandIndex;

	b3Log("		{\n");
	b3Log("		b3BodyDef bd;\n");
	b3Log("		bd.type = (b3BodyType) %d;\n", m_type);
	b3Log("		bd.position.Set(%f, %f, %f);\n", m_sweep.worldCenter.x, m_sweep.worldCenter.y, m_sweep.worldCenter.z);
	b3Log("		bd.orientation.Set(%f, %f, %f, %f);\n", m_sweep.orientation.v.x, m_sweep.orientation.v.y, m_sweep.orientation.v.z, m_sweep.orientation.s);
	b3Log("		bd.linearVelocity.Set(%f, %f, %f);\n", m_linearVelocity.x, m_linearVelocity.y, m_linearVelocity.z);
	b3Log("		bd.angularVelocity.Set(%f, %f, %f);\n", m_angularVelocity.x, m_angularVelocity.y, m_angularVelocity.z);
	b3Log("		bd.gravityScale.Set(%f, %f, %f);\n", m_gravityScale.x, m_gravityScale.y, m_gravityScale.z);
	b3Log("		bd.linearDamping.Set(%f, %f, %f);\n", m_linearDamping.x, m_linearDamping.y, m_linearDamping.z);
	b3Log("		bd.angularDamping.Set(%f, %f, %f);\n", m_angularDamping.x, m_angularDamping.y, m_angularDamping.z);
	b3Log("		bd.awake = %d;\n", m_flags & e_awakeFlag);
	b3Log("		bd.allowSleep = %d;\n", m_flags & e_autoSleepFlag);
	b3Log("		bd.fixedRotationX = %d;\n", m_flags & e_fixedRotationX);
	b3Log("		bd.fixedRotationY = %d;\n", m_flags & e_fixedRotationY);
	b3Log("		bd.fixedRotationZ = %d;\n", m_flags & e_fixedRotationZ);
	b3Log("		bd.linearSleepTolerance = %f;\n", m_linearSleepTolerance);
	b3Log("		bd.angularSleepTolerance = %f;\n", m_angularSleepTolerance);
	b3Log("		\n");
	b3Log("		bodies[%d] = world.CreateBody(bd);\n");
	b3Log("		\n");
	for (b3Fixture* f = m_fixtureList; f; f = f->m_next)
	{
		b3Log("		{\n");
		f->Dump(bodyIndex);
		b3Log("		}\n");
	}
	b3Log("		}\n");
}
