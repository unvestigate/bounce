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

#include <bounce/dynamics/joints/spring_joint.h>
#include <bounce/dynamics/body.h>
#include <bounce/common/draw.h>

// C = ||x2 + r2 - x1 - r1|| - length
// Cdot = dot(n, v2 + w2 x r2 - v1 - w1 x r1)
// J = [-n^T -cross(n, r1)^T n^T cross(n, r2)^T]

void b3SpringJointDef::Initialize(b3Body* bA, b3Body* bB, const b3Vec3& anchorA, const b3Vec3& anchorB)
{
	bodyA = bA;
	bodyB = bB;
	localAnchorA = bodyA->GetLocalPoint(anchorA);
	localAnchorB = bodyB->GetLocalPoint(anchorB);
	length = b3Distance(anchorA, anchorB);
}

b3SpringJoint::b3SpringJoint(const b3SpringJointDef* def) : b3Joint(def)
{
	m_type = e_springJoint;
	m_localAnchorA = def->localAnchorA;
	m_localAnchorB = def->localAnchorB;
	m_length = def->length;
	m_frequencyHz = def->frequencyHz;
	m_dampingRatio = def->dampingRatio;
	m_impulse = scalar(0);
}

b3Vec3 b3SpringJoint::GetAnchorA() const
{
	return m_bodyA->GetWorldPoint(m_localAnchorA);
}

b3Vec3 b3SpringJoint::GetAnchorB() const
{
	return m_bodyB->GetWorldPoint(m_localAnchorB);
}

const b3Vec3& b3SpringJoint::GetLocalAnchorA() const
{
	return m_localAnchorA;
}

const b3Vec3& b3SpringJoint::GetLocalAnchorB() const
{
	return m_localAnchorB;
}

scalar b3SpringJoint::GetLength() const
{
	return m_length;
}

void b3SpringJoint::SetLength(scalar length)
{
	m_length = length;
}

scalar b3SpringJoint::GetFrequency() const
{
	return m_frequencyHz;
}

void b3SpringJoint::SetFrequency(scalar frequency)
{
	m_frequencyHz = frequency;
}

scalar b3SpringJoint::GetDampingRatio() const
{
	return m_dampingRatio;
}

void b3SpringJoint::SetDampingRatio(scalar ratio)
{
	m_dampingRatio = ratio;
}

void b3SpringJoint::InitializeVelocityConstraints(const b3SolverData& data) 
{
	m_indexA = m_bodyA->m_islandIndex;
	m_indexB = m_bodyB->m_islandIndex;

	m_mA = m_bodyA->m_invMass;
	m_mB = m_bodyB->m_invMass;

	m_localCenterA = m_bodyA->m_sweep.localCenter;
	m_localCenterB = m_bodyB->m_sweep.localCenter;

	m_localInvIA = m_bodyA->m_invI;
	m_localInvIB = m_bodyB->m_invI;

	m_iA = data.positions[m_indexA].I;
	m_iB = data.positions[m_indexB].I;

	b3Vec3 xA = data.positions[m_indexA].x;
	b3Quat qA = data.positions[m_indexA].q;
	b3Vec3 xB = data.positions[m_indexB].x;
	b3Quat qB = data.positions[m_indexB].q;

	m_rA = b3Mul(qA, m_localAnchorA - m_localCenterA);
	m_rB = b3Mul(qB, m_localAnchorB - m_localCenterB);

	// Singularity check.
	m_n = xB + m_rB - xA - m_rA;
	scalar length = b3Length(m_n);
	if (length > B3_LINEAR_SLOP)
	{
		m_n /= length;
	}
	else
	{
		m_n.SetZero();
	}
	
	// Compute the effective mass matrix
	b3Vec3 rnA = b3Cross(m_rA, m_n);
	b3Vec3 rnB = b3Cross(m_rB, m_n);

	scalar mass = m_mA + m_mB + b3Dot(m_iA * rnA, rnA) + b3Dot(m_iB * rnB, rnB);
	
	m_mass = mass > scalar(0) ? scalar(1) / mass : scalar(0);

	if (m_frequencyHz > scalar(0))
	{
		scalar C = length - m_length;
		
		// Angular frequency
		scalar omega = scalar(2) * B3_PI * m_frequencyHz;

		// Damping coefficient
		scalar d = scalar(2) * m_mass * m_dampingRatio * omega;

		// Spring stiffness
		scalar k = m_mass * omega * omega;

		// Magic formulas

		// gamma = 1 / (d + h * k)
		
		// m_gamma = gamma / h = 1 / (h * (d + h * k))

		// beta = gamma * h * k

		// bias = beta / h * C
		// bias = (m_gamma * h * k) * C

		scalar h = data.step.dt;
		m_gamma = h * (d + h * k);
		m_gamma = m_gamma != scalar(0) ? scalar(1) / m_gamma : scalar(0);
		m_bias = m_gamma * h * k * C;

		mass += m_gamma;
		m_mass = mass != scalar(0) ? scalar(1) / mass : scalar(0);
	}
	else
	{
		m_bias = scalar(0);
		m_gamma = scalar(0);
	}

	if (data.step.warmStarting)
	{
		m_impulse *= data.step.dtRatio;
	}
	else
	{
		m_impulse = scalar(0);
	}
}

void b3SpringJoint::WarmStart(const b3SolverData& data) 
{
	b3Vec3 P = m_impulse * m_n;
	
	data.velocities[m_indexA].v -= m_mA * P;
	data.velocities[m_indexA].w -= m_iA * b3Cross(m_rA, P);
	data.velocities[m_indexB].v += m_mB * P;
	data.velocities[m_indexB].w += m_iB * b3Cross(m_rB, P);
}

void b3SpringJoint::SolveVelocityConstraints(const b3SolverData& data) 
{
	b3Vec3 vA = data.velocities[m_indexA].v;
	b3Vec3 wA = data.velocities[m_indexA].w;
	b3Vec3 vB = data.velocities[m_indexB].v;
	b3Vec3 wB = data.velocities[m_indexB].w;

	b3Vec3 dv = vB + b3Cross(wB, m_rB) - vA - b3Cross(wA, m_rA);
	scalar Cdot = b3Dot(m_n, dv);
	
	scalar impulse = -m_mass * (Cdot + m_bias + m_gamma * m_impulse);
	m_impulse += impulse;
	
	b3Vec3 P = impulse * m_n;

	vA -= m_mA * P;
	wA -= m_iA * b3Cross(m_rA, P);
	
	vB += m_mB * P;
	wB += m_iB * b3Cross(m_rB, P);

	data.velocities[m_indexA].v = vA;
	data.velocities[m_indexA].w = wA;
	data.velocities[m_indexB].v = vB;
	data.velocities[m_indexB].w = wB;
}

bool b3SpringJoint::SolvePositionConstraints(const b3SolverData& data) 
{
	if (m_frequencyHz > scalar(0))
	{
		// There is no position correction for spring joints.
		B3_NOT_USED(data);
		return true;
	}

	b3Vec3 xA = data.positions[m_indexA].x;
	b3Quat qA = data.positions[m_indexA].q;
	b3Vec3 xB = data.positions[m_indexB].x;
	b3Quat qB = data.positions[m_indexB].q;
	b3Mat33 iA = data.positions[m_indexA].I;
	b3Mat33 iB = data.positions[m_indexB].I;
	scalar mA = m_mA;
	scalar mB = m_mB;

	b3Vec3 rA = b3Mul(qA, m_localAnchorA - m_localCenterA);
	b3Vec3 rB = b3Mul(qB, m_localAnchorB - m_localCenterB);
	
	b3Vec3 n = xB + rB - xA - rA;
	scalar length = b3Length(n);
	scalar C = length - m_length;
	C = b3Clamp(C, -B3_MAX_LINEAR_CORRECTION, B3_MAX_LINEAR_CORRECTION);
	
	// Compute effective mass
	n.Normalize();
	
	b3Vec3 rnA = b3Cross(rA, n);
	b3Vec3 rnB = b3Cross(rB, n);
	scalar kMass = mA + mB + b3Dot(iA * rnA, rnA) + b3Dot(iB * rnB, rnB);
	scalar mass = kMass > scalar(0) ? scalar(1) / kMass : scalar(0);
	scalar lambda = -mass * C;

	b3Vec3 impulse = lambda * n;
		
	xA -= mA * impulse;
	qA -= b3Derivative(qA, iA * b3Cross(rA, impulse));
	iA = b3RotateToFrame(m_localInvIA, qA);

	xB += mB * impulse;
	qB += b3Derivative(qB, iB * b3Cross(rB, impulse));
	iB = b3RotateToFrame(m_localInvIB, qB);

	data.positions[m_indexA].x = xA;
	data.positions[m_indexA].q = qA;
	data.positions[m_indexB].x = xB;
	data.positions[m_indexB].q = qB;
	data.positions[m_indexA].I = iA;
	data.positions[m_indexB].I = iB;

	return b3Abs(C) < B3_LINEAR_SLOP;
}

void b3SpringJoint::Draw(b3Draw* draw) const
{
	b3Vec3 a = m_bodyA->GetWorldPoint(m_localAnchorA);
	draw->DrawPoint(a, scalar(4), b3Color_red);
	
	b3Vec3 b = m_bodyB->GetWorldPoint(m_localAnchorB);
	draw->DrawPoint(b, scalar(4), b3Color_green);

	draw->DrawSegment(a, b, b3Color_yellow);
}
