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

#ifndef B3_JOINT_SOLVER_H
#define B3_JOINT_SOLVER_H

#include <bounce/dynamics/time_step.h>

class b3Joint;

struct b3JointSolverDef 
{
	b3TimeStep step;
	b3Position* positions;
	b3Velocity* velocities;
	b3Joint** joints;
	uint32 count;
};

class b3JointSolver 
{
public :
	b3JointSolver(const b3JointSolverDef* def);

	void InitializeVelocityConstraints();
	void WarmStart();
	void SolveVelocityConstraints();

	bool SolvePositionConstraints();
private :
	b3SolverData m_solverData;
	b3Joint** m_joints;
	uint32 m_count;
};

#endif
