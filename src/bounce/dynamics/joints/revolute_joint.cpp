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

#include <bounce/dynamics/joints/revolute_joint.h>
#include <bounce/dynamics/body.h>
#include <bounce/common/draw.h>

/*
Game Physics Pearls, Quaternion Based Constraints - Claude Lacoursiere, page 195.

Algebra:

Q(p) * P(q) = P(q) * Q(p)
q' = 0.5 * w * q

P = [0 1 0 0]
	[0 0 1 0]
	[0 0 0 1]

Hinge projection matrix:

P_hin = [x^T] * P = [0 1 0 0]
		[y^T]		[0 0 1 0]

Constraint:

q = conj(q1) * q2

C = P_hin * q

Chain rule:

q' =
conj(q1)' * q2 + conj(q1) * q2' =
conj(q1') * q2 + conj(q1) * q2'

1st term:

conj(q1') * q2 =
0.5 * conj(w1 * q1) * w2 =
0.5 * conj(q1) * conj(w1) * q2 =
0.5 * conj(q1) * -w1 * q2 =
-0.5 * conj(q1) * w1 * q2 =
-0.5 * Q(conj(q1)) * P(q2) * Q(w1)

J1 = -0.5 * Q(conj(qA)) * P(qB)

2nd term:

conj(q1) * q2' =
0.5 * conj(q1) * w2 * q2 =
0.5 * Q(conj(q1)) * Q(w2) * Q(q2) =
0.5 * Q(conj(q1)) * P(q2) * Q(w2)

J2 = 0.5 * Q(conj(q1)) * P(q2)

C' = P_hin * q' =
P_hin * (J1 * P^T * w1 + J2 * P^T * w2) =
P_hin * J1 * P^T * w1 + P_hin * J2 * P^T * w2

New Jacobians:

J1 = P_hin * J1 * P^T
J2 = P_hin * J2 * P^T

Limit constraint:

q = conj(q1) * q2

C = 2 * atan(q.z / q.w)

Chain rule:

f( g( q(t) ) ) = 2 * atan( g( q(t) ) )
g( q(t) ) = q.z / q.w

df / dt = del_f / del_g * del_g / del_q * dq / dt

del_f / del_g =
1 / (g^2 + 1) =
1 / ((q.z / q.w)^2 + 1) =
q.w^2 / (q.w^2 + q.z^2) ~
q.w^2

del_g / del_q =
[del_g / del_w | del_g / del_x | del_g / del_y | del_g / del_z] =
[-q.z/q.w^2  0  0  q.w/q.w^2] =
[-q.z/q.w^2  0  0  1/q.w] =
1 / q.w^2 * [-q.z  0  0  q.w]

df / dt =
q.w^2 * 1 / q.w^2 * [-q.z  0  0  q.w] * dq / dt =
[-q.z  0  0  q.w] * dq / dt

P_lim = [-q.z  0  0  q.w]

C' = P_lim * (P_hinge * q') - target_speed

*/

static B3_FORCE_INLINE b3Mat44 b3_iQ_mat(const b3Quat& q)
{
	scalar x = q.v.x, y = q.v.y, z = q.v.z, w = q.s;

	b3Mat44 Q;
	Q.x = b3Vec4(w, x, y, z);
	Q.y = b3Vec4(-x, w, z, -y);
	Q.z = b3Vec4(-y, -z, w, x);
	Q.w = b3Vec4(-z, y, -x, w);
	
	return Q;
}

static B3_FORCE_INLINE b3Mat44 b3_iP_mat(const b3Quat& q)
{
	scalar x = q.v.x, y = q.v.y, z = q.v.z, w = q.s;
	
	b3Mat44 P;
	P.x = b3Vec4(w, x, y, z);
	P.y = b3Vec4(-x, w, -z, y);
	P.z = b3Vec4(-y, z, w, -x);
	P.w = b3Vec4(-z, -y, x, w);
	
	return P;
}

static B3_FORCE_INLINE b3Mat34 b3_P_mat()
{
	b3Mat34 P;
	P.x = b3Vec3(0, 0, 0);
	P.y = b3Vec3(1, 0, 0);
	P.z = b3Vec3(0, 1, 0);
	P.w = b3Vec3(0, 0, 1);
	return P;
}

static B3_FORCE_INLINE b3Mat24 b3_P_hinge_mat()
{
	b3Mat24 P;
	P.x = b3Vec2(0, 0);
	P.y = b3Vec2(1, 0);
	P.z = b3Vec2(0, 1);
	P.w = b3Vec2(0, 0);
	return P;
}

// 1x4 
static B3_FORCE_INLINE b3Vec4 b3_P_hinge_limit_mat(const b3Quat& q)
{
	return b3Vec4(-q.v.z, 0.0f, 0.0f, q.s);
}

// 4x1 
static B3_FORCE_INLINE b3Vec4 b3_q_to_v(const b3Quat& q)
{
	return b3Vec4(q.s, q.v.x, q.v.y, q.v.z);
}

static const b3Mat34 b3Mat34_P = b3_P_mat();
static const b3Mat43 b3Mat43_PT = b3Transpose(b3Mat34_P);
static const b3Mat24 b3Mat24_P_hinge = b3_P_hinge_mat();

void b3RevoluteJointDef::Initialize(b3Body* bA, b3Body* bB,
	const b3Vec3& axis, const b3Vec3& anchor,
	scalar lower, scalar upper)
{
	B3_ASSERT(lower <= upper);

	bodyA = bA;
	bodyB = bB;

	b3Mat33 rotation;
	rotation.z = axis;
	b3ComputeBasis(rotation.z, rotation.x, rotation.y);

	b3Quat q = b3Mat33Quat(rotation);

	localAnchorA = bodyA->GetLocalPoint(anchor);
	localRotationA = bodyA->GetLocalFrame(q);

	localAnchorB = bodyB->GetLocalPoint(anchor);
	localRotationB = bodyB->GetLocalFrame(q);

	referenceRotation.SetIdentity();

	lowerAngle = lower;
	upperAngle = upper;
}

b3RevoluteJoint::b3RevoluteJoint(const b3RevoluteJointDef* def)
{
	m_type = e_revoluteJoint;
	m_referenceRotation = def->referenceRotation;
	m_localAnchorA = def->localAnchorA;
	m_localRotationA = def->localRotationA;
	m_localAnchorB = def->localAnchorB;
	m_localRotationB = def->localRotationB;

	m_enableMotor = def->enableMotor;
	m_motorSpeed = def->motorSpeed;
	m_maxMotorTorque = def->maxMotorTorque;

	m_enableLimit = def->enableLimit;
	m_lowerAngle = def->lowerAngle;
	m_upperAngle = def->upperAngle;
	B3_ASSERT(m_lowerAngle <= m_upperAngle);
	
	m_motorImpulse = scalar(0);

	m_limitState = e_inactiveLimit;
	m_limitImpulse = scalar(0);
	
	m_linearImpulse.SetZero();
	m_angularImpulse.SetZero();
}

void b3RevoluteJoint::InitializeConstraints(const b3SolverData* data)
{
	b3Body* m_bodyA = GetBodyA();
	b3Body* m_bodyB = GetBodyB();

	m_indexA = m_bodyA->m_islandID;
	m_indexB = m_bodyB->m_islandID;
	m_mA = m_bodyA->m_invMass;
	m_mB = m_bodyB->m_invMass;
	m_localCenterA = m_bodyA->m_sweep.localCenter;
	m_localCenterB = m_bodyB->m_sweep.localCenter;
	m_localInvIA = m_bodyA->m_invI;
	m_localInvIB = m_bodyB->m_invI;
	
	m_iA = data->invInertias[m_indexA];
	m_iB = data->invInertias[m_indexB];

	b3Quat qA = data->positions[m_indexA].q;
	b3Quat qB = data->positions[m_indexB].q;

	scalar mA = m_mA;
	b3Mat33 iA = m_iA;
	scalar mB = m_mB;
	b3Mat33 iB = m_iB;

	b3Quat fA = qA * m_localRotationA;
	b3Quat fB = qB * m_localRotationB;

	// Joint rotation
	b3Quat dq = b3Conjugate(m_referenceRotation) * b3Conjugate(fA) * fB;

	// Motor constraint.
	if (m_enableMotor || m_enableLimit)
	{
		b3Vec4 P_hinge_limit = b3_P_hinge_limit_mat(dq);

		b3Mat44 G1 = -scalar(0.5) * b3_iQ_mat(b3Conjugate(fA)) * b3_iP_mat(fB);
		b3Mat44 G2 = scalar(0.5) * b3_iQ_mat(b3Conjugate(fA)) * b3_iP_mat(fB);

		b3Vec3 J1 = P_hinge_limit * G1 * b3Mat43_PT;
		b3Vec3 J2 = P_hinge_limit * G2 * b3Mat43_PT;

		b3Vec3 J1T = J1;
		b3Vec3 J2T = J2;

		m_motorJ1 = J1;
		m_motorJ2 = J2;

		scalar K = J1 * iA * J1T + J2 * iB * J2T;
		
		m_motorMass = K > scalar(0) ? scalar(1) / K : scalar(0);
	}

	// Limit constraint.
	if (m_enableLimit)
	{
		// Joint angle
		scalar angle = scalar(2) * atan2(dq.v.z, dq.s);

		if (b3Abs(m_upperAngle - m_lowerAngle) < scalar(2) * B3_ANGULAR_SLOP)
		{
			if (m_limitState != e_equalLimits)
			{
				m_limitState = e_equalLimits;
				m_limitImpulse = scalar(0);
			}
		}
		else if (angle <= m_lowerAngle)
		{
			if (m_limitState != e_atLowerLimit)
			{
				m_limitState = e_atLowerLimit;
				m_limitImpulse = scalar(0);
			}
		}
		else if (angle >= m_upperAngle)
		{
			if (m_limitState != e_atUpperLimit)
			{
				m_limitState = e_atUpperLimit;
				m_limitImpulse = scalar(0);
			}
		}
		else
		{
			m_limitState = e_inactiveLimit;
			m_limitImpulse = scalar(0);
		}
	}
	else
	{
		m_limitState = e_inactiveLimit;
	}

	// Linear constraints.
	{
		m_rA = b3Mul(qA, m_localAnchorA - m_localCenterA);
		m_rB = b3Mul(qB, m_localAnchorB - m_localCenterB);

		b3Mat33 RA = b3Skew(m_rA);
		b3Mat33 RAT = b3Transpose(RA);
		b3Mat33 RB = b3Skew(m_rB);
		b3Mat33 RBT = b3Transpose(RB);
		b3Mat33 M = b3Mat33Diagonal(mA + mB);

		m_linearMass = M + RA * iA * RAT + RB * iB * RBT;
	}

	// Angular constraints.
	{
		b3Mat44 G1 = -scalar(0.5) * b3_iQ_mat(b3Conjugate(fA)) * b3_iP_mat(fB);
		b3Mat44 G2 = scalar(0.5) * b3_iQ_mat(b3Conjugate(fA)) * b3_iP_mat(fB);
		
		b3Mat23 J1 = b3Mat24_P_hinge * G1 * b3Mat43_PT;
		b3Mat23 J2 = b3Mat24_P_hinge * G2 * b3Mat43_PT;

		b3Mat32 J1T = b3Transpose(J1);
		b3Mat32 J2T = b3Transpose(J2);

		m_angularJ1 = J1;
		m_angularJ2 = J2;
		
		m_angularJ1T = J1T;
		m_angularJ2T = J2T;

		b3Mat22 K = J1 * iA * J1T + J2 * iB * J2T;
		
		m_angularMass = b3Inverse(K);
	}
}

void b3RevoluteJoint::WarmStart(const b3SolverData* data)
{
	b3Vec3 vA = data->velocities[m_indexA].v;
	b3Vec3 wA = data->velocities[m_indexA].w;
	b3Vec3 vB = data->velocities[m_indexB].v;
	b3Vec3 wB = data->velocities[m_indexB].w;

	if (m_enableMotor && m_limitState != e_equalLimits)
	{
		b3Vec3 P1 = m_motorJ1 * m_motorImpulse;
		b3Vec3 P2 = m_motorJ2 * m_motorImpulse;

		wA += m_iA * P1;
		wB += m_iB * P2;
	}

	if (m_enableLimit && m_limitState != e_inactiveLimit)
	{
		b3Vec3 P1 = m_motorJ1 * m_limitImpulse;
		b3Vec3 P2 = m_motorJ2 * m_limitImpulse;

		wA += m_iA * P1;
		wB += m_iB * P2;
	}

	{
		vA -= m_mA * m_linearImpulse;
		wA -= m_iA * b3Cross(m_rA, m_linearImpulse);

		vB += m_mB * m_linearImpulse;
		wB += m_iB * b3Cross(m_rB, m_linearImpulse);
	}

	{
		b3Vec3 P1 = m_angularJ1T * m_angularImpulse;
		b3Vec3 P2 = m_angularJ2T * m_angularImpulse;

		wA += m_iA * P1;
		wB += m_iB * P2;
	}

	data->velocities[m_indexA].v = vA;
	data->velocities[m_indexA].w = wA;
	data->velocities[m_indexB].v = vB;
	data->velocities[m_indexB].w = wB;
}

void b3RevoluteJoint::SolveVelocityConstraints(const b3SolverData* data)
{
	b3Vec3 vA = data->velocities[m_indexA].v;
	b3Vec3 wA = data->velocities[m_indexA].w;
	b3Vec3 vB = data->velocities[m_indexB].v;
	b3Vec3 wB = data->velocities[m_indexB].w;

	b3Mat33 iA = m_iA;
	b3Mat33 iB = m_iB;

	// Solve motor constraint.
	if (m_enableMotor && m_limitState != e_equalLimits)
	{
		scalar dw = m_motorJ1 * wA + m_motorJ2 * wB;
		scalar Cdot = dw - m_motorSpeed;
		scalar impulse = -m_motorMass * Cdot;
		scalar oldImpulse = m_motorImpulse;
		scalar maxImpulse = data->dt * m_maxMotorTorque;
		m_motorImpulse = b3Clamp(m_motorImpulse + impulse, -maxImpulse, maxImpulse);
		impulse = m_motorImpulse - oldImpulse;

		b3Vec3 P1 = m_motorJ1 * impulse;
		b3Vec3 P2 = m_motorJ2 * impulse;

		wA += iA * P1;
		wB += iB * P2;
	}

	// Solve limit constraint.
	if (m_enableLimit && m_limitState != e_inactiveLimit)
	{
		scalar Cdot = m_motorJ1 * wA + m_motorJ2 * wB;
		scalar impulse = -m_motorMass * Cdot;

		if (m_limitState == e_equalLimits)
		{
			m_limitImpulse += impulse;
		}
		else if (m_limitState == e_atLowerLimit)
		{
			scalar oldImpulse = m_limitImpulse;
			m_limitImpulse = b3Max(m_limitImpulse + impulse, scalar(0));
			impulse = m_limitImpulse - oldImpulse;
		}
		else if (m_limitState == e_atUpperLimit)
		{
			scalar oldImpulse = m_limitImpulse;
			m_limitImpulse = b3Min(m_limitImpulse + impulse, scalar(0));
			impulse = m_limitImpulse - oldImpulse;
		}

		b3Vec3 P1 = m_motorJ1 * impulse;
		b3Vec3 P2 = m_motorJ2 * impulse;

		wA += iA * P1;
		wB += iB * P2;
	}

	// Solve linear constraints.
	{
		b3Vec3 Cdot = vB + b3Cross(wB, m_rB) - vA - b3Cross(wA, m_rA);
		b3Vec3 impulse = m_linearMass.Solve(-Cdot);

		m_linearImpulse += impulse;

		vA -= m_mA * impulse;
		wA -= m_iA * b3Cross(m_rA, impulse);

		vB += m_mB * impulse;
		wB += m_iB * b3Cross(m_rB, impulse);
	}

	// Solve angular constraints.
	{
		b3Vec2 Cdot = m_angularJ1 * wA + m_angularJ2 * wB;
		b3Vec2 impulse = m_angularMass * -Cdot;

		m_angularImpulse += impulse;

		b3Vec3 P1 = m_angularJ1T * impulse;
		b3Vec3 P2 = m_angularJ2T * impulse;

		wA += m_iA * P1;
		wB += m_iB * P2;
	}

	data->velocities[m_indexA].v = vA;
	data->velocities[m_indexA].w = wA;
	data->velocities[m_indexB].v = vB;
	data->velocities[m_indexB].w = wB;
}

bool b3RevoluteJoint::SolvePositionConstraints(const b3SolverData* data)
{
	b3Vec3 xA = data->positions[m_indexA].x;
	b3Quat qA = data->positions[m_indexA].q;
	b3Vec3 xB = data->positions[m_indexB].x;
	b3Quat qB = data->positions[m_indexB].q;
	b3Mat33 iA = data->invInertias[m_indexA];
	b3Mat33 iB = data->invInertias[m_indexB];

	scalar mA = m_mA;
	scalar mB = m_mB;

	// Solve linear constraints.
	scalar linearError = scalar(0);

	{
		b3Vec3 rA = b3Mul(qA, m_localAnchorA - m_localCenterA);
		b3Vec3 rB = b3Mul(qB, m_localAnchorB - m_localCenterB);

		b3Vec3 C = xB + rB - xA - rA;

		linearError += b3Length(C);

		// Compute effective mass
		b3Mat33 M = b3Mat33Diagonal(mA + mB);
		b3Mat33 RA = b3Skew(rA);
		b3Mat33 RAT = b3Transpose(RA);
		b3Mat33 RB = b3Skew(rB);
		b3Mat33 RBT = b3Transpose(RB);

		b3Mat33 mass = M + RA * iA * RAT + RB * iB * RBT;

		b3Vec3 impulse = mass.Solve(-C);

		xA -= mA * impulse;
		qA -= b3Derivative(qA, iA * b3Cross(rA, impulse));
		qA.Normalize();
		iA = b3RotateToFrame(m_localInvIA, qA);

		xB += mB * impulse;
		qB += b3Derivative(qB, iB * b3Cross(rB, impulse));
		qB.Normalize();
		iB = b3RotateToFrame(m_localInvIB, qB);
	}

	// Solve angular constraints.
	scalar angularError = scalar(0);

	if (m_enableLimit)
	{
		// Solve limit constraint.
		b3Quat fA = qA * m_localRotationA;
		b3Quat fB = qB * m_localRotationB;
		
		b3Quat dq = b3Conjugate(m_referenceRotation) * b3Conjugate(fA) * fB;

		b3Vec4 P_hinge_limit = b3_P_hinge_limit_mat(dq);

		b3Mat44 G1 = -scalar(0.5) * b3_iQ_mat(b3Conjugate(fA)) * b3_iP_mat(fB);
		b3Mat44 G2 = scalar(0.5) * b3_iQ_mat(b3Conjugate(fA)) * b3_iP_mat(fB);

		b3Vec3 J1 = P_hinge_limit * G1 * b3Mat43_PT;
		b3Vec3 J2 = P_hinge_limit * G2 * b3Mat43_PT;

		b3Vec3 J1T = J1;
		b3Vec3 J2T = J2;

		scalar K = J1 * iA * J1T + J2 * iB * J2T;
		scalar limitMass = K > scalar(0) ? scalar(1) / K : scalar(0);

		scalar limitImpulse = scalar(0);

		scalar angle = scalar(2) * atan2(dq.v.z, dq.s);

		if (b3Abs(m_upperAngle - m_lowerAngle) < scalar(2) * B3_ANGULAR_SLOP)
		{
			scalar C = angle - m_lowerAngle;
			angularError += b3Abs(C);

			// Prevent large corrections
			C = b3Clamp(C, -B3_MAX_ANGULAR_CORRECTION, B3_MAX_ANGULAR_CORRECTION);
			limitImpulse = -C * limitMass;
		}
		else if (angle <= m_lowerAngle)
		{
			scalar C = angle - m_lowerAngle;
			angularError += -C;

			// Allow some slop and prevent large corrections
			C = b3Clamp(C + B3_ANGULAR_SLOP, -B3_MAX_ANGULAR_CORRECTION, scalar(0));
			limitImpulse = -C * limitMass;
		}
		else if (angle >= m_upperAngle)
		{
			scalar C = angle - m_upperAngle;
			angularError += C;

			// Allow some slop and prevent large corrections
			C = b3Clamp(C - B3_ANGULAR_SLOP, scalar(0), B3_MAX_ANGULAR_CORRECTION);
			limitImpulse = -C * limitMass;
		}

		b3Vec3 P1 = J1T * limitImpulse;
		b3Vec3 P2 = J2T * limitImpulse;

		qA += b3Derivative(qA, iA * P1);
		qA.Normalize();
		iA = b3RotateToFrame(m_localInvIA, qA);

		qB += b3Derivative(qB, iB * P2);
		qB.Normalize();
		iB = b3RotateToFrame(m_localInvIB, qB);
	}

	{
		// Solve hinge constraint.
		b3Quat fA = qA * m_localRotationA;
		b3Quat fB = qB * m_localRotationB;

		b3Quat dq = b3Conjugate(m_referenceRotation) * b3Conjugate(fA) * fB;

		b3Vec4 dv = b3_q_to_v(dq);

		b3Vec2 C = b3Mat24_P_hinge * dv;

		angularError += b3Length(C);

		// Compute effective mass
		b3Mat44 G1 = -scalar(0.5) * b3_iQ_mat(b3Conjugate(fA)) * b3_iP_mat(fB);
		b3Mat44 G2 = scalar(0.5) * b3_iQ_mat(b3Conjugate(fA)) * b3_iP_mat(fB);

		b3Mat23 J1 = b3Mat24_P_hinge * G1 * b3Mat43_PT;
		b3Mat23 J2 = b3Mat24_P_hinge * G2 * b3Mat43_PT;

		b3Mat32 J1T = b3Transpose(J1);
		b3Mat32 J2T = b3Transpose(J2);

		b3Mat22 mass = J1 * iA * J1T + J2 * iB * J2T;
		b3Vec2 impulse = mass.Solve(-C);

		b3Vec3 P1 = J1T * impulse;
		b3Vec3 P2 = J2T * impulse;

		qA += b3Derivative(qA, iA * P1);
		qA.Normalize();
		iA = b3RotateToFrame(m_localInvIA, qA);

		qB += b3Derivative(qB, iB * P2);
		qB.Normalize();
		iB = b3RotateToFrame(m_localInvIB, qB);
	}

	data->positions[m_indexA].x = xA;
	data->positions[m_indexA].q = qA;
	data->positions[m_indexB].x = xB;
	data->positions[m_indexB].q = qB;
	data->invInertias[m_indexA] = iA;
	data->invInertias[m_indexB] = iB;

	return linearError <= B3_LINEAR_SLOP && angularError <= B3_ANGULAR_SLOP;
}

b3Transform b3RevoluteJoint::GetFrameA() const
{
	b3Transform xf(m_localAnchorA, m_localRotationA);
	return GetBodyA()->GetWorldFrame(xf);
}

b3Transform b3RevoluteJoint::GetFrameB() const
{
	b3Transform xf(m_localAnchorB, m_localRotationB);
	return GetBodyB()->GetWorldFrame(xf);
}

b3Transform b3RevoluteJoint::GetLocalFrameA() const
{
	return b3Transform(m_localAnchorA, m_localRotationA);
}

b3Transform b3RevoluteJoint::GetLocalFrameB() const
{
	return b3Transform(m_localAnchorB, m_localRotationB);
}

bool b3RevoluteJoint::IsLimitEnabled() const
{
	return m_enableLimit;
}

void b3RevoluteJoint::SetEnableLimit(bool bit)
{
	if (bit != m_enableLimit)
	{
		GetBodyA()->SetAwake(true);
		GetBodyB()->SetAwake(true);
		m_limitImpulse = scalar(0);
		m_limitState = e_inactiveLimit;
		m_enableLimit = bit;
	}
}

scalar b3RevoluteJoint::GetLowerLimit() const
{
	return m_lowerAngle;
}

scalar b3RevoluteJoint::GetUpperLimit() const
{
	return m_upperAngle;
}

void b3RevoluteJoint::SetLimits(scalar lower, scalar upper)
{
	B3_ASSERT(lower <= upper);
	if (lower != m_lowerAngle || upper != m_upperAngle)
	{
		GetBodyA()->SetAwake(true);
		GetBodyB()->SetAwake(true);
		m_limitImpulse = scalar(0);
		m_lowerAngle = lower;
		m_upperAngle = upper;
	}
}

bool b3RevoluteJoint::IsMotorEnabled() const
{
	return m_enableMotor;
}

void b3RevoluteJoint::SetEnableMotor(bool bit)
{
	if (bit != m_enableMotor)
	{
		GetBodyA()->SetAwake(true);
		GetBodyB()->SetAwake(true);
		m_motorImpulse = scalar(0);
		m_enableMotor = bit;
	}
}

scalar b3RevoluteJoint::GetMotorSpeed() const
{
	return m_motorSpeed;
}

void b3RevoluteJoint::SetMotorSpeed(scalar speed)
{
	GetBodyA()->SetAwake(true);
	GetBodyB()->SetAwake(true);
	m_motorSpeed = speed;
}

scalar b3RevoluteJoint::GetMaxMotorTorque() const
{
	return m_maxMotorTorque;
}

void b3RevoluteJoint::SetMaxMotorTorque(scalar torque)
{
	GetBodyA()->SetAwake(true);
	GetBodyB()->SetAwake(true);
	m_maxMotorTorque = torque;
}

void b3RevoluteJoint::Draw(b3Draw* draw) const
{
	b3Transform xfA = GetFrameA();
	draw->DrawTransform(xfA);
	
	b3Transform xfB = GetFrameB();
	draw->DrawTransform(xfB);
}
