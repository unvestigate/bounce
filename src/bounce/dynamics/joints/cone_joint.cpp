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

#include <bounce/dynamics/joints/cone_joint.h>
#include <bounce/dynamics/body.h>
#include <bounce/common/draw.h>

// C = dot(u2, u1) - cos(angle / 2) > 0
// Cdot = dot(u2, omega1 x u1) + dot(u1, omega2 x u2)
// Cycle:
// dot(u1 x u2, omega1) + dot(u2 x u1, omega2) = 
// dot(-u2 x u1, omega1) + dot(u2 x u1, omega2)
// n = u2 x u1
// J = [0 -n 0 n]

// Stable C: 
// C =  angle / 2 - atan2( norm(u2 x u1), dot(u2, u1) ) > 0

void b3ConeJointDef::Initialize(b3Body* bA, b3Body* bB,
	const b3Vec3& axis, const b3Vec3& anchor, scalar angle)
{
	bodyA = bA;
	bodyB = bB;

	b3Mat33 rotation;
	rotation.x = axis;
	b3ComputeBasis(rotation.x, rotation.y, rotation.z);

	b3Quat q = b3Mat33Quat(rotation);

	localAnchorA = bodyA->GetLocalPoint(anchor);
	localRotationA = bodyA->GetLocalFrame(q);

	localAnchorB = bodyB->GetLocalPoint(anchor);
	localRotationB = bodyB->GetLocalFrame(q);

	referenceRotation.SetIdentity();

	coneAngle = angle;
}

b3ConeJoint::b3ConeJoint(const b3ConeJointDef* def) : b3Joint(def)
{
	m_type = e_coneJoint;
	m_localAnchorA = def->localAnchorA;
	m_localRotationA = def->localRotationA;
	m_localAnchorB = def->localAnchorB;
	m_localRotationB = def->localRotationB;
	m_referenceRotation = def->referenceRotation;
	
	m_enableConeLimit = def->enableConeLimit;
	m_coneAngle = def->coneAngle;

	m_coneState = e_inactiveLimit;
	m_coneImpulse = scalar(0);
	m_coneAxis.SetZero();
	
	m_impulse.SetZero();
}

void b3ConeJoint::InitializeVelocityConstraints(const b3SolverData& data)
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

	b3Quat fA = qA * m_localRotationA;
	b3Quat fB = qB * m_localRotationB;

	b3Mat33 RfA = fA.GetRotationMatrix();
	b3Mat33 RfB = fB.GetRotationMatrix();

	// Linear constraint.
	{
		m_rA = b3Mul(qA, m_localAnchorA - m_localCenterA);
		m_rB = b3Mul(qB, m_localAnchorB - m_localCenterB);

		// Compute effective mass matrix.
		b3Mat33 M = b3Mat33Diagonal(m_mA + m_mB);
		b3Mat33 RA = b3Skew(m_rA);
		b3Mat33 RAT = b3Transpose(RA);
		b3Mat33 RB = b3Skew(m_rB);
		b3Mat33 RBT = b3Transpose(RB);
		m_mass = M + RA * m_iA * RAT + RB * m_iB * RBT;
	}

	// Cone limit constraint.
	if (m_enableConeLimit)
	{
		b3Vec3 u1 = RfA.x;
		b3Vec3 u2 = RfB.x;

		m_coneAxis = b3Cross(u2, u1);

		scalar mass = b3Dot((m_iA + m_iB) * m_coneAxis, m_coneAxis);
		m_coneMass = mass > scalar(0) ? scalar(1) / mass : scalar(0);

		// C = cone / 2 - angle >= 0
		scalar cosine = b3Dot(u2, u1);
		scalar sine = b3Length(m_coneAxis);
		scalar angle = atan2(sine, cosine);
		if (scalar(0.5) * m_coneAngle < angle)
		{
			if (m_coneState != e_atLowerLimit)
			{
				m_coneState = e_atLowerLimit;
				m_coneImpulse = scalar(0);
			}
		}
		else
		{
			m_coneState = e_inactiveLimit;
			m_coneImpulse = scalar(0);
		}
	}
	else
	{
		m_coneState = e_inactiveLimit;
	}

	if (data.step.warmStarting)
	{
		// Scale impulses to support a variable time step.
		m_impulse *= data.step.dtRatio;
		m_coneImpulse *= data.step.dtRatio;
	}
	else
	{
		m_impulse.SetZero();
		m_coneImpulse = scalar(0);
	}
}

void b3ConeJoint::WarmStart(const b3SolverData& data)
{
	b3Vec3 vA = data.velocities[m_indexA].v;
	b3Vec3 wA = data.velocities[m_indexA].w;
	b3Vec3 vB = data.velocities[m_indexB].v;
	b3Vec3 wB = data.velocities[m_indexB].w;

	{
		vA -= m_mA * m_impulse;
		wA -= m_iA * b3Cross(m_rA, m_impulse);

		vB += m_mB * m_impulse;
		wB += m_iB * b3Cross(m_rB, m_impulse);
	}

	if (m_enableConeLimit && m_coneState != e_inactiveLimit)
	{
		b3Vec3 L = m_coneImpulse * m_coneAxis;
		
		wA -= m_iA * L;
		wB += m_iB * L;
	}

	data.velocities[m_indexA].v = vA;
	data.velocities[m_indexA].w = wA;
	data.velocities[m_indexB].v = vB;
	data.velocities[m_indexB].w = wB;
}

void b3ConeJoint::SolveVelocityConstraints(const b3SolverData& data)
{
	b3Vec3 vA = data.velocities[m_indexA].v;
	b3Vec3 wA = data.velocities[m_indexA].w;
	b3Vec3 vB = data.velocities[m_indexB].v;
	b3Vec3 wB = data.velocities[m_indexB].w;

	// Solve linear constraint.
	{
		b3Vec3 Cdot = vB + b3Cross(wB, m_rB) - vA - b3Cross(wA, m_rA);
		b3Vec3 P = m_mass.Solve(-Cdot);

		m_impulse += P;

		vA -= m_mA * P;
		wA -= m_iA * b3Cross(m_rA, P);

		vB += m_mB * P;
		wB += m_iB * b3Cross(m_rB, P);
	}

	// Solve cone constraint.
	if (m_enableConeLimit && m_coneState != e_inactiveLimit)
	{
		scalar Cdot = b3Dot(m_coneAxis, wB - wA);
		scalar impulse = -m_coneMass * Cdot;
		scalar oldImpulse = m_coneImpulse;
		m_coneImpulse = b3Max(m_coneImpulse + impulse, scalar(0));
		impulse = m_coneImpulse - oldImpulse;

		b3Vec3 P = impulse * m_coneAxis;

		wA -= m_iA * P;
		wB += m_iB * P;
	}

	data.velocities[m_indexA].v = vA;
	data.velocities[m_indexA].w = wA;
	data.velocities[m_indexB].v = vB;
	data.velocities[m_indexB].w = wB;
}

bool b3ConeJoint::SolvePositionConstraints(const b3SolverData& data)
{
	b3Vec3 xA = data.positions[m_indexA].x;
	b3Quat qA = data.positions[m_indexA].q;
	b3Vec3 xB = data.positions[m_indexB].x;
	b3Quat qB = data.positions[m_indexB].q;
	b3Mat33 iA = data.positions[m_indexA].I;
	b3Mat33 iB = data.positions[m_indexB].I;

	scalar mA = m_mA;
	scalar mB = m_mB;
	
	// Solve limit constraints.
	scalar limitError = scalar(0);
	if (m_enableConeLimit)
	{
		b3Quat fA = qA * m_localRotationA;
		b3Quat fB = qB * m_localRotationB;
		
		b3Mat33 RfA = fA.GetRotationMatrix();
		b3Mat33 RfB = fB.GetRotationMatrix();
		
		b3Vec3 u1 = RfA.x;
		b3Vec3 u2 = RfB.x;

		b3Vec3 coneAxis = b3Cross(u2, u1);

		scalar mass = b3Dot((iA + iB) * coneAxis, coneAxis);
		scalar coneMass = mass > scalar(0) ? scalar(1) / mass : scalar(0);

		// C = cone / 2 - angle >= 0
		scalar cosine = b3Dot(u2, u1);
		scalar sine = b3Length(coneAxis);
		scalar angle = atan2(sine, cosine);

		if (scalar(0.5) * m_coneAngle < angle)
		{
			scalar C = scalar(0.5) * m_coneAngle - angle;
			
			limitError = -C;

			// Allow some slop and prevent large corrections
			C = b3Clamp(C + B3_ANGULAR_SLOP, -B3_MAX_ANGULAR_CORRECTION, scalar(0));
			
			scalar coneImpulse = -C * coneMass;

			b3Vec3 P = coneImpulse * coneAxis;

			qA -= b3Derivative(qA, iA * P);
			qA.Normalize();
			iA = b3RotateToFrame(m_localInvIA, qA);

			qB += b3Derivative(qB, iB * P);
			qB.Normalize();
			iB = b3RotateToFrame(m_localInvIB, qB);
		}
	}

	// Solve linear constraint.
	scalar linearError = scalar(0);
	{
		b3Vec3 rA = b3Mul(qA, m_localAnchorA - m_localCenterA);
		b3Vec3 rB = b3Mul(qB, m_localAnchorB - m_localCenterB);

		b3Vec3 C = xB + rB - xA - rA;

		linearError = b3Length(C);

		b3Mat33 M = b3Mat33Diagonal(mA + mB);
		b3Mat33 RA = b3Skew(rA);
		b3Mat33 RAT = b3Transpose(RA);
		b3Mat33 RB = b3Skew(rB);
		b3Mat33 RBT = b3Transpose(RB);
		b3Mat33 mass = M + RA * iA * RAT + RB * iB * RBT;

		b3Vec3 P = mass.Solve(-C);

		xA -= mA * P;
		qA -= b3Derivative(qA, iA * b3Cross(rA, P));
		qA.Normalize();
		iA = b3RotateToFrame(m_localInvIA, qA);

		xB += mB * P;
		qB += b3Derivative(qB, iB * b3Cross(rB, P));
		qB.Normalize();
		iB = b3RotateToFrame(m_localInvIB, qB);
	}

	data.positions[m_indexA].x = xA;
	data.positions[m_indexA].q = qA;
	data.positions[m_indexB].x = xB;
	data.positions[m_indexB].q = qB;
	data.positions[m_indexA].I = iA;
	data.positions[m_indexB].I = iB;

	return linearError <= B3_LINEAR_SLOP && limitError <= B3_ANGULAR_SLOP;
}

void b3ConeJoint::SetEnableConeLimit(bool bit)
{
	if (bit != m_enableConeLimit)
	{
		m_bodyA->SetAwake(true);
		m_bodyB->SetAwake(true);
		m_coneImpulse = scalar(0);
		m_coneState = e_inactiveLimit;
		m_enableConeLimit = bit;
	}
}

bool b3ConeJoint::IsConeLimitEnabled() const
{
	return m_enableConeLimit;
}

void b3ConeJoint::SetConeAngle(scalar angle)
{
	if (angle != m_coneAngle)
	{
		m_bodyA->SetAwake(true);
		m_bodyB->SetAwake(true);
		m_coneImpulse = scalar(0);
		m_coneAngle = angle;
	}
}

scalar b3ConeJoint::GetConeAngle() const
{
	return m_coneAngle;
}

void b3ConeJoint::Draw(b3Draw* draw) const
{
	b3Transform xfA(m_localAnchorA, m_localRotationA);
	xfA = m_bodyA->GetWorldFrame(xfA);
	draw->DrawTransform(xfA);

	b3Transform xfB(m_localAnchorB, m_localRotationB);
	xfB = m_bodyB->GetWorldFrame(xfB);
	draw->DrawTransform(xfB);
}
