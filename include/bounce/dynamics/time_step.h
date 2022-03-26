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

#ifndef B3_TIME_STEP_H
#define B3_TIME_STEP_H

#include <bounce/common/math/vec3.h>
#include <bounce/common/math/mat33.h>
#include <bounce/common/math/quat.h>

// This is an internal structure.
struct b3TimeStep
{
	scalar dt; // time step
	scalar inv_dt; // inverse time step (0 if dt == 0).
	scalar dtRatio; // dt * inv_dt0
	uint32 velocityIterations;
	uint32 positionIterations;
	bool warmStarting;
};

// This is an internal structure.
struct b3Position
{
	b3Vec3 x;
	b3Quat q;
	b3Mat33 I;
};

// This is an internal structure.
struct b3Velocity
{
	b3Vec3 v;
	b3Vec3 w;
};

// Solver data
struct b3SolverData
{
	b3TimeStep step;
	b3Position* positions;
	b3Velocity* velocities;
};	

enum b3LimitState
{
	e_inactiveLimit,
	e_atLowerLimit,
	e_atUpperLimit,
	e_equalLimits
};

// Compute the inertia matrix of a body measured in 
// inertial frame (variable over time) given the 
// inertia matrix in body-fixed frame (constant) 
// and a rotation matrix representing the orientation 
// of the body frame relative to the inertial frame.
inline b3Mat33 b3RotateToFrame(const b3Mat33& inertia, const b3Quat& rotation)
{
	b3Mat33 R = rotation.GetRotationMatrix();
	return R * inertia * b3Transpose(R);
}

// Compute the time derivative of an orientation given
// the angular velocity of the rotating frame represented by the orientation.
inline b3Quat b3Derivative(const b3Quat& q, const b3Vec3& omega)
{
	b3Quat w(omega, scalar(0));
	return scalar(0.5) * w * q;
}

// Integrate an orientation over a time step given
// the current orientation, angular velocity of the rotating frame
// represented by the orientation, and the time step dt.
inline b3Quat b3Integrate(const b3Quat& q, const b3Vec3& omega, scalar dt)
{
	b3Quat qdot = b3Derivative(q, omega);
	b3Quat q2 = q + dt * qdot;
	q2.Normalize();
	return q2;
}

#endif