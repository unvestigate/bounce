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

#include <bounce/dynamics/joints/joint_solver.h>
#include <bounce/dynamics/joints/joint.h>

b3JointSolver::b3JointSolver(const b3JointSolverDef* def) 
{
	m_count = def->count;
	m_joints = def->joints;
	m_solverData.step = def->step;
	m_solverData.positions = def->positions;
	m_solverData.velocities = def->velocities;
}

void b3JointSolver::InitializeVelocityConstraints() 
{
	for (uint32 i = 0; i < m_count; ++i) 
	{
		m_joints[i]->InitializeVelocityConstraints(m_solverData);
	}
}

void b3JointSolver::WarmStart() 
{
	for (uint32 i = 0; i < m_count; ++i) 
	{
		m_joints[i]->WarmStart(m_solverData);
	}
}

void b3JointSolver::SolveVelocityConstraints() 
{
	for (uint32 i = 0; i < m_count; ++i) 
	{
		m_joints[i]->SolveVelocityConstraints(m_solverData);
	}
}

bool b3JointSolver::SolvePositionConstraints() 
{
	bool jointsSolved = true;
	for (uint32 i = 0; i < m_count; ++i) 
	{
		bool jointSolved = m_joints[i]->SolvePositionConstraints(m_solverData);
		jointsSolved = jointsSolved && jointSolved;
	}
	return jointsSolved;
}
