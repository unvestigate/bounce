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

#include <bounce/dynamics/island.h>
#include <bounce/dynamics/world_callbacks.h>
#include <bounce/dynamics/joints/joint.h>
#include <bounce/dynamics/joints/joint_solver.h>
#include <bounce/dynamics/contacts/contact.h>
#include <bounce/dynamics/contacts/contact_solver.h>
#include <bounce/common/memory/stack_allocator.h>
#include <bounce/common/profiler.h>

b3Island::b3Island(uint32 bodyCapacity, uint32 contactCapacity, uint32 jointCapacity, b3StackAllocator* allocator, b3ContactListener* listener, b3Profiler* profiler)
{
	m_allocator = allocator;
	m_listener = listener;
	m_bodyCapacity = bodyCapacity;
	m_contactCapacity = contactCapacity;
	m_jointCapacity = jointCapacity;
	
	m_bodies = (b3Body**)m_allocator->Allocate(m_bodyCapacity * sizeof(b3Body*));
	m_velocities = (b3Velocity*)m_allocator->Allocate(m_bodyCapacity * sizeof(b3Velocity));
	m_positions = (b3Position*)m_allocator->Allocate(m_bodyCapacity * sizeof(b3Position));
	m_contacts = (b3Contact**)m_allocator->Allocate(m_contactCapacity * sizeof(b3Contact*));
	m_joints = (b3Joint**)m_allocator->Allocate(m_jointCapacity * sizeof(b3Joint*));

	m_bodyCount = 0;
	m_contactCount = 0;
	m_jointCount = 0;

	m_profiler = profiler;
}

b3Island::~b3Island() 
{
	// Note: Reverse order of construction.
	m_allocator->Free(m_joints);
	m_allocator->Free(m_contacts);
	m_allocator->Free(m_positions);
	m_allocator->Free(m_velocities);
	m_allocator->Free(m_bodies);
}

// Numerical Methods (Erin, p60)
static B3_FORCE_INLINE b3Vec3 b3SolveGyro(const b3Quat& q, const b3Mat33& Ib, const b3Vec3& w1, scalar h)
{
	// Convert angular velocity to body coordinates
	b3Vec3 w1b = b3MulC(q, w1);
	
	// Jacobian of f
	b3Mat33 J = Ib + h * (b3Skew(w1b) * Ib - b3Skew(Ib * w1b));
	
	// One iteration of Newton-Raphson
	// Residual vector
	b3Vec3 f;
	{
		f = h * b3Cross(w1b, Ib * w1b);
		w1b -= J.Solve(f);
	}
	
	// Convert angular velocity back to world coordinates
	b3Vec3 w2 = b3Mul(q, w1b);
	return w2;
}

void b3Island::Solve(const b3TimeStep& step, const b3Vec3& gravity, bool allowSleep)
{
	scalar h = step.dt;

	// 1. Integrate velocities
	for (uint32 i = 0; i < m_bodyCount; ++i) 
	{
		b3Body* b = m_bodies[i];

		b3Vec3 v = b->m_linearVelocity;
		b3Vec3 w = b->m_angularVelocity;
		b3Vec3 x = b->m_sweep.worldCenter;
		b3Quat q = b->m_sweep.orientation;

		// Remember the positions for CCD
		b->m_sweep.worldCenter0 = b->m_sweep.worldCenter;
		b->m_sweep.orientation0 = b->m_sweep.orientation;

		if (b->m_type == e_dynamicBody) 
		{
			// Integrate forces
			v += h * (b3Mul(b->m_gravityScale, gravity) + b->m_invMass * b->m_force);

			// Integrate torques
			b3Vec3 dw1 = h * b->m_worldInvI * b->m_torque;
			
			// "Numerical Methods", (Erin, p71)
			// Implicit Euler on next inertia and angular velocity
			b3Vec3 w2 = b3SolveGyro(q, b->m_I, w, h);
			b3Vec3 dw2 = w2 - w;

			w += dw1 + dw2;

			// Apply local damping.
			// ODE: dv/dt + c * v = 0
			// Solution: v(t) = v0 * exp(-c * t)
			// Step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v * exp(-c * dt)
			// v2 = exp(-c * dt) * v1
			// Padé approximation:
			// 1 / (1 + c * dt) 
			v.x *= scalar(1) / (scalar(1) + h * b->m_linearDamping.x);
			v.y *= scalar(1) / (scalar(1) + h * b->m_linearDamping.y);
			v.z *= scalar(1) / (scalar(1) + h * b->m_linearDamping.z);

			w.x *= scalar(1) / (scalar(1) + h * b->m_angularDamping.x);
			w.y *= scalar(1) / (scalar(1) + h * b->m_angularDamping.y);
			w.z *= scalar(1) / (scalar(1) + h * b->m_angularDamping.z);
		}

		m_velocities[i].v = v;
		m_velocities[i].w = w;
		m_positions[i].x = x;
		m_positions[i].q = q;
		m_positions[i].I = b->m_worldInvI;
	}

	b3JointSolverDef jointSolverDef;
	jointSolverDef.step = step;
	jointSolverDef.positions = m_positions;
	jointSolverDef.velocities = m_velocities;
	jointSolverDef.joints = m_joints;
	jointSolverDef.count = m_jointCount;
	b3JointSolver jointSolver(&jointSolverDef);

	b3ContactSolverDef contactSolverDef;
	contactSolverDef.step = step;
	contactSolverDef.positions = m_positions;
	contactSolverDef.velocities = m_velocities;
	contactSolverDef.contacts = m_contacts;
	contactSolverDef.count = m_contactCount;
	contactSolverDef.allocator = m_allocator;
	b3ContactSolver contactSolver(&contactSolverDef);

	// 2. Initialize velocity constraints
	{
		B3_PROFILE(m_profiler, "Initialize Velocity Constraints");
		
		contactSolver.InitializeVelocityConstraints();

		if (step.warmStarting)
		{
			contactSolver.WarmStart();
		}

		jointSolver.InitializeVelocityConstraints();

		if (step.warmStarting)
		{
			jointSolver.WarmStart();
		}
	}

	// 3. Solve velocity constraints
	{
		B3_PROFILE(m_profiler, "Solve Velocity Constraints");

		for (uint32 i = 0; i < step.velocityIterations; ++i)
		{
			jointSolver.SolveVelocityConstraints();
			contactSolver.SolveVelocityConstraints();
		}

		if (step.warmStarting)
		{
			contactSolver.StoreImpulses();
		}
	}

	// 4. Integrate positions
	for (uint32 i = 0; i < m_bodyCount; ++i) 
	{
		b3Body* b = m_bodies[i];
		
		b3Vec3 x = m_positions[i].x;
		b3Quat q = m_positions[i].q;
		b3Mat33 I = m_positions[i].I;
		
		b3Vec3 v = m_velocities[i].v;
		b3Vec3 w = m_velocities[i].w;

		if (b->m_type != e_staticBody)
		{
			// Prevent numerical instability due to large velocity changes.		
			b3Vec3 translation = h * v;
			if (b3Dot(translation, translation) > B3_MAX_TRANSLATION_SQUARED)
			{
				scalar ratio = B3_MAX_TRANSLATION / b3Length(translation);
				v *= ratio;
			}

			b3Vec3 rotation = h * w;
			if (b3Dot(rotation, rotation) > B3_MAX_ROTATION_SQUARED)
			{
				scalar ratio = B3_MAX_ROTATION / b3Length(rotation);
				w *= ratio;
			}

			// Integrate
			x += h * v;
			q = b3Integrate(q, w, h);
			I = b3RotateToFrame(b->m_invI, q);
		}

		m_positions[i].x = x;
		m_positions[i].q = q;
		m_positions[i].I = I;
		m_velocities[i].v = v;
		m_velocities[i].w = w;
	}

	// 5. Solve position constraints
	bool positionsSolved = false;
	
	{
		B3_PROFILE(m_profiler, "Solve Position Constraints");
		
		for (uint32 i = 0; i < step.positionIterations; ++i) 
		{
			bool contactsSolved = contactSolver.SolvePositionConstraints();
			bool jointsSolved = jointSolver.SolvePositionConstraints();
			if (contactsSolved && jointsSolved)
			{
				// Early out if the position errors are small.
				positionsSolved = true;
				break;
			}
		}
	}

	// 6. Copy state buffers back to the bodies
	for (uint32 i = 0; i < m_bodyCount; ++i) 
	{
		b3Body* b = m_bodies[i];
		b->m_sweep.worldCenter = m_positions[i].x;
		b->m_sweep.orientation = m_positions[i].q;
		b->m_worldInvI = m_positions[i].I;
		b->m_linearVelocity = m_velocities[i].v;
		b->m_angularVelocity = m_velocities[i].w;	
		
		b->SynchronizeTransform();
	}

	// Post solve callback report
	Report();

	// 7. Put bodies under unconsiderable motion to sleep
	if (allowSleep) 
	{
		scalar minSleepTime = B3_MAX_SCALAR;

		for (uint32 i = 0; i < m_bodyCount; ++i) 
		{
			b3Body* b = m_bodies[i];
			if (b->m_type == e_staticBody) 
			{
				continue;
			}

			// Compute the linear and angular speed of the body.
			scalar sqrLinVel = b3Dot(b->m_linearVelocity, b->m_linearVelocity);
			scalar sqrAngVel = b3Dot(b->m_angularVelocity, b->m_angularVelocity);

			if (b->IsSleepingAllowed() == false ||
				sqrLinVel > b->m_linearSleepTolerance * b->m_linearSleepTolerance || 
				sqrAngVel > b->m_angularSleepTolerance * b->m_angularSleepTolerance) 
			{
				b->m_sleepTime = scalar(0);
			}
			else 
			{
				b->m_sleepTime += h;
			}

			minSleepTime = b3Min(minSleepTime, b->m_sleepTime);
		}

		// Put the island to sleep so long as the minimum found sleep time
		// is below the threshold. 
		if (minSleepTime >= B3_TIME_TO_SLEEP && positionsSolved) 
		{
			for (uint32 i = 0; i < m_bodyCount; ++i) 
			{
				m_bodies[i]->SetAwake(false);
			}
		}
	}
}

void b3Island::Report()
{
	if (m_listener == nullptr)
	{
		return;
	}

	for (uint32 i = 0; i < m_contactCount; ++i)
	{
		m_listener->PostSolve(m_contacts[i]);
	}
}