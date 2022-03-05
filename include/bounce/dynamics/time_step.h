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

struct b3Position
{
	b3Vec3 x;
	b3Quat q;
};

struct b3Velocity
{
	b3Vec3 v;
	b3Vec3 w;
};

struct b3SolverData
{
	b3Position* positions;
	b3Velocity* velocities;
	b3Mat33* invInertias;
	scalar dt;
	scalar invdt;
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
inline b3Quat b3Derivative(const b3Quat& orientation, const b3Vec3& velocity)
{
	b3Quat w(velocity.x, velocity.y, velocity.z, scalar(0));
	return scalar(0.5) * w * orientation;
}

// Integrate an orientation over a time step given
// the current orientation, angular velocity of the rotating frame
// represented by the orientation, and the time step dt.
inline b3Quat b3Integrate(const b3Quat& orientation, const b3Vec3& omega, scalar dt)
{
	b3Quat qdot = b3Derivative(orientation, omega);
	b3Quat q = orientation + dt * qdot;
	q.Normalize();
	return q;
}

#endif