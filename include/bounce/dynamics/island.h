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

#ifndef B3_ISLAND_H
#define B3_ISLAND_H

#include <bounce/common/math/mat33.h>
#include <bounce/dynamics/time_step.h>
#include <bounce/dynamics/body.h>

class b3StackAllocator;
class b3ContactListener;
struct b3Position;
struct b3Velocity;
class b3Profiler;

struct b3ContactVelocityConstraint;

class b3Island 
{
public:
	b3Island(uint32 bodyCapacity, uint32 contactCapacity, uint32 jointCapacity, b3StackAllocator* allocator, b3ContactListener* listener, b3Profiler* profiler);
	~b3Island();

	void Clear()
	{
		m_bodyCount = 0;
		m_contactCount = 0;
		m_jointCount = 0;
	}

	void Add(b3Body* b)
	{
		B3_ASSERT(m_bodyCount < m_bodyCapacity);
		b->m_islandIndex = m_bodyCount;
		m_bodies[m_bodyCount] = b;
		++m_bodyCount;
	}

	void Add(b3Contact* c)
	{
		B3_ASSERT(m_contactCount < m_contactCapacity);
		m_contacts[m_contactCount] = c;
		++m_contactCount;
	}

	void Add(b3Joint* j)
	{
		B3_ASSERT(m_jointCount < m_jointCapacity);
		m_joints[m_jointCount] = j;
		++m_jointCount;
	}

	void Solve(const b3TimeStep& step, const b3Vec3& gravity, bool allowSleep);

	void Report();

	b3StackAllocator* m_allocator;
	b3ContactListener* m_listener;
	b3Profiler* m_profiler;

	b3Body** m_bodies;
	uint32 m_bodyCapacity;
	uint32 m_bodyCount;

	b3Contact** m_contacts;
	uint32 m_contactCapacity;
	uint32 m_contactCount;

	b3Joint** m_joints;
	uint32 m_jointCapacity;
	uint32 m_jointCount;
	
	b3Position* m_positions;
	b3Velocity* m_velocities;
};

#endif