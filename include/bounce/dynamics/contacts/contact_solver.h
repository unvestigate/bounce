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

#ifndef B3_CONTACT_SOLVER_H
#define B3_CONTACT_SOLVER_H

#include <bounce/common/math/vec2.h>
#include <bounce/common/math/mat22.h>
#include <bounce/dynamics/time_step.h>

class b3StackAllocator;
class b3Contact;
struct b3Position;
struct b3Velocity;

struct b3PositionConstraintPoint
{
	b3Vec3 localNormalA;
	b3Vec3 localPointA;
	b3Vec3 localPointB;
};

struct b3PositionConstraintManifold
{
	b3PositionConstraintPoint* points;
	uint32 pointCount;
};

struct b3ContactPositionConstraint 
{
	uint32 indexA;
	scalar invMassA;
	b3Mat33 localInvIA;
	scalar radiusA;
	b3Vec3 localCenterA;
	uint32 indexB;
	b3Vec3 localCenterB;
	scalar invMassB;
	b3Mat33 localInvIB;
	scalar radiusB;
	b3PositionConstraintManifold* manifolds;
	uint32 manifoldCount;
};

struct b3VelocityConstraintPoint 
{
	b3Vec3 rA;
	b3Vec3 rB;

	b3Vec3 normal;
	scalar normalMass;
	scalar normalImpulse;
	scalar velocityBias;
};

struct b3VelocityConstraintManifold
{
	b3Vec3 rA;
	b3Vec3 rB;
	
	b3Vec3 normal;

	b3Vec3 tangent1;
	b3Vec3 tangent2;
	scalar tangentSpeed1;
	scalar tangentSpeed2;
	b3Mat22 tangentMass;
	b3Vec2 tangentImpulse;

	scalar motorImpulse;
	scalar motorMass;
	scalar motorSpeed;

	b3VelocityConstraintPoint* points;
	uint32 pointCount;
};

struct b3ContactVelocityConstraint 
{
	uint32 indexA;
	scalar invMassA;
	b3Mat33 invIA;
	scalar invMassB;
	uint32 indexB;
	b3Mat33 invIB;
	scalar friction;
	scalar restitution;
	b3VelocityConstraintManifold* manifolds;
	uint32 manifoldCount;
};

struct b3ContactSolverDef 
{
	b3TimeStep step;
	b3Contact** contacts;
	uint32 count;
	b3Position* positions;
	b3Velocity* velocities;
	b3StackAllocator* allocator;
};

class b3ContactSolver 
{
public:
	b3ContactSolver(const b3ContactSolverDef* def);
	~b3ContactSolver();

	void InitializeVelocityConstraints();
	void WarmStart();
	void SolveVelocityConstraints();
	void StoreImpulses();

	bool SolvePositionConstraints();

	b3TimeStep m_step;
	b3Position* m_positions;
	b3Velocity* m_velocities;
	b3Contact** m_contacts;
	uint32 m_count;
	b3StackAllocator* m_allocator;
	b3ContactPositionConstraint* m_positionConstraints;
	b3ContactVelocityConstraint* m_velocityConstraints;
};

#endif
