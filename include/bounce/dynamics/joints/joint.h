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

#ifndef B3_JOINT_H
#define B3_JOINT_H

#include <bounce/common/math/transform.h>
#include <bounce/common/math/mat22.h>
#include <bounce/common/math/mat33.h>
#include <bounce/common/template/list.h>
#include <bounce/dynamics/time_step.h>

class b3Body;
class b3Joint;
class b3BlockAllocator;
class b3Draw;
struct b3SolverData;

enum b3JointType
{
	e_unknownJoint,
	e_mouseJoint,
	e_springJoint,
	e_weldJoint,
	e_revoluteJoint,
	e_sphereJoint,
	e_coneJoint,
	e_frictionJoint,
	e_motorJoint,
	e_prismaticJoint,
	e_wheelJoint,
	e_typeCount,
};

struct b3JointDef
{
	b3JointDef()
	{
		type = e_unknownJoint;
		bodyA = nullptr;
		bodyB = nullptr;
		userData = nullptr;
		collideLinked = false;
	}

	b3JointType type;
	b3Body* bodyA;
	b3Body* bodyB;
	void* userData;
	bool collideLinked;
};

// A joint edge to a joint graph, where a body is a vertex and a joint an edge.
struct b3JointEdge
{
	b3Body* m_other;
	b3Joint* m_joint;
	b3JointEdge* m_prev;
	b3JointEdge* m_next;
};

// Base joint class. For simplicity, a joint is unique per body pair. 
// There are many joint types, some of them provide motors and limits.
class b3Joint
{
public:
	// Get this joint type.
	b3JointType GetType() const;

	// Get the body A connected to this joint.
	const b3Body* GetBodyA() const;
	b3Body* GetBodyA();

	// Get the body B connected to this joint.
	const b3Body* GetBodyB() const;
	b3Body* GetBodyB();

	// Get the user data associated with this joint.
	void* GetUserData();
	const void* GetUserData() const;

	// Set the user data to be associated with the joint.
	void SetUserData(void* data);

	// Should the bodies linked by this joint collide with each other?
	bool CollideLinked() const;
	
	// Set if the bodies linked by this joint should collide with each other.
	void SetCollideLinked(bool bit);
	
	// Dump this joint to the log file.
	virtual void Dump() const
	{
		b3Log("Dump feature not implemented for this joint type.\n");
	}

	// Draw this joint.
	virtual void Draw(b3Draw* draw) const
	{
		b3Log("Draw feature not implemented for this joint type.\n");
	}

	// Get the next joint in the world joint list.
	const b3Joint* GetNext() const;
	b3Joint* GetNext();
protected:
	friend class b3Body;
	friend class b3World;
	friend class b3Island;
	friend class b3JointManager;
	friend class b3JointSolver;
	friend class b3List<b3Joint>;
	
	static b3Joint* Create(const b3JointDef* def, b3BlockAllocator* allocator);
	static void Destroy(b3Joint* j, b3BlockAllocator* allocator);

	b3Joint() {	}
	virtual ~b3Joint() { }
	
	virtual void InitializeConstraints(const b3SolverData* data) = 0;
	virtual void WarmStart(const b3SolverData* data) = 0;
	virtual void SolveVelocityConstraints(const b3SolverData* data) = 0;
	virtual bool SolvePositionConstraints(const b3SolverData* data) = 0;

	enum 
	{
		e_islandFlag = 0x0001,
		e_activeFlag = 0x0002
	};
	
	b3JointType m_type;
	u32 m_flags;

	b3Body* m_bodyA;
	b3Body* m_bodyB;

	// To the body A joint edge list.
	b3JointEdge m_edgeA;

	// To the body B joint edge list.
	b3JointEdge m_edgeB;

	void* m_userData;
	bool m_collideLinked;

	// Links to the world joint list.
	b3Joint* m_prev;
	b3Joint* m_next;
};

inline b3JointType b3Joint::GetType() const 
{ 
	return m_type; 
}

inline const b3Body* b3Joint::GetBodyA() const
{
	return m_bodyA;
}

inline b3Body* b3Joint::GetBodyA()
{
	return m_bodyA;
}

inline const b3Body* b3Joint::GetBodyB() const
{
	return m_bodyB;
}

inline b3Body* b3Joint::GetBodyB() 
{
	return m_bodyB;
}

inline void* b3Joint::GetUserData() 
{
	return m_userData;
}

inline const void* b3Joint::GetUserData() const 
{
	return m_userData;
}

inline void b3Joint::SetUserData(void* data)
{
	m_userData = data;
}

inline void b3Joint::SetCollideLinked(bool bit)
{
	m_collideLinked = bit;
}

inline bool b3Joint::CollideLinked() const
{
	return m_collideLinked;
}

inline b3Joint* b3Joint::GetNext()
{
	return m_next;
}

inline const b3Joint* b3Joint::GetNext() const
{
	return m_next;
}

#endif
