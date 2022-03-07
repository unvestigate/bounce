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

#include <bounce/dynamics/joints/joint.h>
#include <bounce/dynamics/joints/mouse_joint.h>
#include <bounce/dynamics/joints/spring_joint.h>
#include <bounce/dynamics/joints/weld_joint.h>
#include <bounce/dynamics/joints/revolute_joint.h>
#include <bounce/dynamics/joints/sphere_joint.h>
#include <bounce/dynamics/joints/cone_joint.h>
#include <bounce/dynamics/joints/friction_joint.h>
#include <bounce/dynamics/joints/motor_joint.h>
#include <bounce/dynamics/joints/prismatic_joint.h>
#include <bounce/dynamics/joints/wheel_joint.h>
#include <bounce/common/memory/block_allocator.h>

b3Joint* b3Joint::Create(const b3JointDef* def, b3BlockAllocator* allocator)
{
	b3Joint* joint = nullptr;
	switch (def->type)
	{
	case e_mouseJoint:
	{
		void* mem = allocator->Allocate(sizeof(b3MouseJoint));
		joint = new (mem) b3MouseJoint(static_cast<const b3MouseJointDef*>(def));
		break;
	}
	case e_springJoint:
	{
		void* mem = allocator->Allocate(sizeof(b3SpringJoint));
		joint = new (mem) b3SpringJoint(static_cast<const b3SpringJointDef*>(def));
		break;
	}
	case e_weldJoint:
	{
		void* mem = allocator->Allocate(sizeof(b3WeldJoint));
		joint = new (mem) b3WeldJoint(static_cast<const b3WeldJointDef*>(def));
		break;
	}case e_revoluteJoint:
	{
		void* mem = allocator->Allocate(sizeof(b3RevoluteJoint));
		joint = new (mem) b3RevoluteJoint(static_cast<const b3RevoluteJointDef*>(def));
		break;
	}
	case e_sphereJoint:
	{
		void* mem = allocator->Allocate(sizeof(b3SphereJoint));
		joint = new (mem) b3SphereJoint(static_cast<const b3SphereJointDef*>(def));
		break;
	}
	case e_coneJoint:
	{
		void* mem = allocator->Allocate(sizeof(b3ConeJoint));
		joint = new (mem) b3ConeJoint(static_cast<const b3ConeJointDef*>(def));
		break;
	}
	case e_frictionJoint:
	{
		void* mem = allocator->Allocate(sizeof(b3FrictionJoint));
		joint = new (mem) b3FrictionJoint(static_cast<const b3FrictionJointDef*>(def));
		break;
	}
	case e_motorJoint:
	{
		void* mem = allocator->Allocate(sizeof(b3MotorJoint));
		joint = new (mem) b3MotorJoint(static_cast<const b3MotorJointDef*>(def));
		break;
	}
	case e_prismaticJoint:
	{
		void* mem = allocator->Allocate(sizeof(b3PrismaticJoint));
		joint = new (mem) b3PrismaticJoint(static_cast<const b3PrismaticJointDef*>(def));
		break;
	}
	case e_wheelJoint:
	{
		void* mem = allocator->Allocate(sizeof(b3WheelJoint));
		joint = new (mem) b3WheelJoint(static_cast<const b3WheelJointDef*>(def));
		break;
	}
	default:
	{
		B3_ASSERT(false);
		break;
	}
	}
	return joint;
}

void b3Joint::Destroy(b3Joint* joint, b3BlockAllocator* allocator)
{
	joint->~b3Joint();
	switch (joint->m_type)
	{
	case e_mouseJoint:
	{
		allocator->Free(joint, sizeof(b3MouseJoint));
		break;
	}
	case e_springJoint:
	{
		allocator->Free(joint, sizeof(b3SpringJoint));
		break;
	}
	case e_weldJoint:
	{
		allocator->Free(joint, sizeof(b3WeldJoint));
		break;
	}
	case e_revoluteJoint:
	{
		allocator->Free(joint, sizeof(b3RevoluteJoint));
		break;
	}
	case e_sphereJoint:
	{
		allocator->Free(joint, sizeof(b3SphereJoint));
		break;
	}
	case e_coneJoint:
	{
		allocator->Free(joint, sizeof(b3ConeJoint));
		break;
	}
	case e_frictionJoint:
	{
		allocator->Free(joint, sizeof(b3FrictionJoint));
		break;
	}
	case e_motorJoint:
	{
		allocator->Free(joint, sizeof(b3MotorJoint));
		break;
	}
	case e_prismaticJoint:
	{
		allocator->Free(joint, sizeof(b3PrismaticJoint));
		break;
	}
	case e_wheelJoint:
	{
		allocator->Free(joint, sizeof(b3WheelJoint));
		break;
	}
	default:
	{
		B3_ASSERT(false);
		break;
	}
	}
}

b3Joint::b3Joint(const b3JointDef* def)
{
	B3_ASSERT(def->bodyA != def->bodyB);

	m_type = def->type;
	m_prev = nullptr;
	m_next = nullptr;
	m_bodyA = def->bodyA;
	m_bodyB = def->bodyB;
	m_collideConnected = def->collideConnected;
	m_islandFlag = false;
	m_userData = def->userData;

	m_edgeA.m_joint = nullptr;
	m_edgeA.m_other = nullptr;
	m_edgeA.m_prev = nullptr;
	m_edgeA.m_next = nullptr;

	m_edgeB.m_joint = nullptr;
	m_edgeB.m_other = nullptr;
	m_edgeB.m_prev = nullptr;
	m_edgeB.m_next = nullptr;
}
