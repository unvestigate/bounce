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
#include <bounce/common/math/quat.h>
#include <bounce/common/math/mat33.h>

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

#endif