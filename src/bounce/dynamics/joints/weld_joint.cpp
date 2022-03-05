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

#include <bounce/dynamics/joints/weld_joint.h>
#include <bounce/dynamics/body.h>
#include <bounce/common/math/mat.h>
#include <bounce/common/draw.h>

/*
Game Physics Pearls, Quaternion Based Constraints - Claude Lacoursiere, page 195.

Algebra:

Q(p) * P(q) = P(q) * Q(p)
q' = 0.5 * w * q

P = [0 1 0 0]
	[0 0 1 0]
	[0 0 0 1]

Lock projection matrix:

P_lock = [0 1 0 0]
		 [0 0 1 0]
		 [0 0 0 1]

Constraint:

q = conj(q1) * q2

C = P_lock * q

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

G1 = -0.5 * Q(conj(q1)) * P(q2)

2nd term:

conj(q1) * q2' =
0.5 * conj(q1) * w2 * q2 =
0.5 * Q(conj(q1)) * Q(w2) * Q(q2) =
0.5 * Q(conj(q1)) * P(q2) * Q(w2)

G2 = 0.5 * Q(conj(q1)) * P(q2)

C' = P_lock * q' =
P_lock * (G1 * P^T * w1 + G2 * P^T * w2) =
P_lock * G1 * P^T * w1 + P_lock * G2 * P^T * w2

Jacobians:

J1 = P_lock * G1 * P^T
J2 = P_lock * G2 * P^T

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

static B3_FORCE_INLINE b3Mat34 b3_P_lock_mat()
{
	b3Mat34 P;
	P.x = b3Vec3(0, 0, 0);
	P.y = b3Vec3(1, 0, 0);
	P.z = b3Vec3(0, 1, 0);
	P.w = b3Vec3(0, 0, 1);
	return P;
}

static B3_FORCE_INLINE b3Vec4 b3_q_to_v(const b3Quat& q)
{
	return b3Vec4(q.s, q.v.x, q.v.y, q.v.z);
}

static const b3Mat34 b3Mat34_P = b3_P_mat();
static const b3Mat43 b3Mat43_PT = b3Transpose(b3Mat34_P);
static const b3Mat34 b3Mat34_P_lock = b3_P_lock_mat();

void b3WeldJointDef::Initialize(b3Body* bA, b3Body* bB, const b3Vec3& anchor)
{
	bodyA = bA;
	bodyB = bB;
	
	localAnchorA = bodyA->GetLocalPoint(anchor);
	localAnchorB = bodyB->GetLocalPoint(anchor);

	b3Quat qA = bodyA->GetOrientation();
	b3Quat qB = bodyB->GetOrientation();

	referenceRotation = b3Conjugate(qA) * qB;
}

b3WeldJoint::b3WeldJoint(const b3WeldJointDef* def)
{
	m_type = e_weldJoint;
	m_localAnchorA = def->localAnchorA;
	m_localAnchorB = def->localAnchorB;
	m_referenceRotation = def->referenceRotation;
	m_frequencyHz = def->frequencyHz;
	m_dampingRatio = def->dampingRatio;

	m_linearImpulse.SetZero();
	m_angularImpulse.SetZero();
}

void b3WeldJoint::SetAnchor(const b3Vec3& anchor)
{
	const b3Body* bodyA = GetBodyA();
	const b3Body* bodyB = GetBodyB();

	m_localAnchorA = bodyA->GetLocalPoint(anchor);
	m_localAnchorB = bodyB->GetLocalPoint(anchor);

	b3Quat qA = bodyA->GetOrientation();
	b3Quat qB = bodyB->GetOrientation();

	m_referenceRotation = b3Conjugate(qA) * qB;
}

void b3WeldJoint::SetReferenceRotation(const b3Quat& referenceRotation)
{
	m_referenceRotation = referenceRotation;
}

static B3_FORCE_INLINE void b3ComputeSoftConstraintCoefficients(scalar& gamma, scalar& bias, 
	scalar frequencyHz, scalar dampingRatio, scalar m, scalar C, scalar h)
{
	// Frequency
	scalar omega = scalar(2) * B3_PI * frequencyHz;

	// Spring stiffness
	scalar k = omega * omega * m;

	// Damping coefficient
	scalar d = scalar(2) * dampingRatio * omega * m;

	// Magic formulas
	gamma = h * (d + h * k);
	gamma = gamma != scalar(0) ? scalar(1) / gamma : scalar(0);
	bias = gamma * h * k * C;
}

void b3WeldJoint::InitializeConstraints(const b3SolverData* data)
{
	m_indexA = m_bodyA->m_islandID;
	m_indexB = m_bodyB->m_islandID;
	m_mA = m_bodyA->m_invMass;
	m_mB = m_bodyB->m_invMass;
	m_iA = data->invInertias[m_indexA];
	m_iB = data->invInertias[m_indexB];
	m_localCenterA = m_bodyA->m_sweep.localCenter;
	m_localCenterB = m_bodyB->m_sweep.localCenter;
	m_localInvIA = m_bodyA->m_invI;
	m_localInvIB = m_bodyB->m_invI;

	b3Quat qA = data->positions[m_indexA].q;
	b3Quat qB = data->positions[m_indexB].q;

	{
		// Compute effective mass for the block solver
		m_rA = b3Mul(qA, m_localAnchorA - m_localCenterA);
		m_rB = b3Mul(qB, m_localAnchorB - m_localCenterB);

		b3Mat33 RA = b3Skew(m_rA);
		b3Mat33 RAT = b3Transpose(RA);
		b3Mat33 RB = b3Skew(m_rB);
		b3Mat33 RBT = b3Transpose(RB);
		b3Mat33 M = b3Mat33Diagonal(m_mA + m_mB);

		m_linearMass = M + RA * m_iA * RAT + RB * m_iB * RBT;
	}

	if (m_frequencyHz > scalar(0))
	{
		b3Mat33 invM = m_iA + m_iB;
		b3Mat33 m = b3Inverse(invM);

		b3Quat q1 = b3Conjugate(qA) * qB;
		b3Quat q2 = m_referenceRotation;
		
		scalar sign = b3Sign(b3Dot(q1, q2));

		q1 = sign * q1;

		// Apply finite difference
		b3Quat q = scalar(2) * (q2 - q1) * b3Conjugate(q1);

		// Convert the relative angular error to world's frame
		// Negate so we solve impulse = -m^1 * (Cdot - bias)
		b3Vec3 C = b3Mul(qA, -q.v);

		scalar h = data->dt;

		b3Vec3 gamma, bias;
		
		b3ComputeSoftConstraintCoefficients(gamma.x, bias.x, m_frequencyHz, m_dampingRatio, m.x.x, C.x, h);
		b3ComputeSoftConstraintCoefficients(gamma.y, bias.y, m_frequencyHz, m_dampingRatio, m.y.y, C.y, h);
		b3ComputeSoftConstraintCoefficients(gamma.z, bias.z, m_frequencyHz, m_dampingRatio, m.z.z, C.z, h);

		m_gamma = b3Mat33Diagonal(gamma.x, gamma.y, gamma.z);
		m_bias = bias;

		invM += m_gamma;
		m_angularMass = b3Inverse(invM);
	}
	else
	{
		m_gamma.SetZero();
		m_bias.SetZero();
		m_angularMass = b3Inverse(m_iA + m_iB);
	}
}

void b3WeldJoint::WarmStart(const b3SolverData* data)
{
	b3Vec3 vA = data->velocities[m_indexA].v;
	b3Vec3 wA = data->velocities[m_indexA].w;
	b3Vec3 vB = data->velocities[m_indexB].v;
	b3Vec3 wB = data->velocities[m_indexB].w;

	{
		vA -= m_mA * m_linearImpulse;
		wA -= m_iA * b3Cross(m_rA, m_linearImpulse);

		vB += m_mB * m_linearImpulse;
		wB += m_iB * b3Cross(m_rB, m_linearImpulse);
	}

	{
		wA -= m_iA * m_angularImpulse;
		wB += m_iB * m_angularImpulse;
	}

	data->velocities[m_indexA].v = vA;
	data->velocities[m_indexA].w = wA;
	data->velocities[m_indexB].v = vB;
	data->velocities[m_indexB].w = wB;
}

void b3WeldJoint::SolveVelocityConstraints(const b3SolverData* data)
{
	b3Vec3 vA = data->velocities[m_indexA].v;
	b3Vec3 wA = data->velocities[m_indexA].w;
	b3Vec3 vB = data->velocities[m_indexB].v;
	b3Vec3 wB = data->velocities[m_indexB].w;

	b3Quat qA = data->positions[m_indexA].q;
	b3Quat qB = data->positions[m_indexB].q;

	{
		b3Vec3 Cdot = vB + b3Cross(wB, m_rB) - vA - b3Cross(wA, m_rA);
		b3Vec3 impulse = m_linearMass.Solve(-Cdot);

		m_linearImpulse += impulse;

		vA -= m_mA * impulse;
		wA -= m_iA * b3Cross(m_rA, impulse);

		vB += m_mB * impulse;
		wB += m_iB * b3Cross(m_rB, impulse);
	}

	{
		b3Vec3 Cdot = wB - wA;
		b3Vec3 impulse = -m_angularMass * (Cdot + m_bias + m_gamma * m_angularImpulse);
		m_angularImpulse += impulse;

		wA -= m_iA * impulse;
		wB += m_iB * impulse;
	}

	data->velocities[m_indexA].v = vA;
	data->velocities[m_indexA].w = wA;
	data->velocities[m_indexB].v = vB;
	data->velocities[m_indexB].w = wB;
}

bool b3WeldJoint::SolvePositionConstraints(const b3SolverData* data)
{
	b3Vec3 xA = data->positions[m_indexA].x;
	b3Quat qA = data->positions[m_indexA].q;
	b3Vec3 xB = data->positions[m_indexB].x;
	b3Quat qB = data->positions[m_indexB].q;
	b3Mat33 iA = data->invInertias[m_indexA];
	b3Mat33 iB = data->invInertias[m_indexB];

	scalar linearError = scalar(0);

	{
		// Compute effective mass
		b3Vec3 rA = b3Mul(qA, m_localAnchorA - m_localCenterA);
		b3Vec3 rB = b3Mul(qB, m_localAnchorB - m_localCenterB);

		b3Mat33 M = b3Mat33Diagonal(m_mA + m_mB);
		b3Mat33 RA = b3Skew(rA);
		b3Mat33 RAT = b3Transpose(RA);
		b3Mat33 RB = b3Skew(rB);
		b3Mat33 RBT = b3Transpose(RB);

		b3Mat33 mass = M + RA * iA * RAT + RB * iB * RBT;

		b3Vec3 C = xB + rB - xA - rA;
		b3Vec3 impulse = mass.Solve(-C);

		xA -= m_mA * impulse;
		qA -= b3Derivative(qA, b3Mul(iA, b3Cross(rA, impulse)));
		qA.Normalize();
		iA = b3RotateToFrame(m_localInvIA, qA);

		xB += m_mB * impulse;
		qB += b3Derivative(qB, b3Mul(iB, b3Cross(rB, impulse)));
		qB.Normalize();
		iB = b3RotateToFrame(m_localInvIB, qB);

		linearError += b3Length(C);
	}

	scalar angularError = scalar(0);

	if (m_frequencyHz == scalar(0))
	{
		b3Quat dq = b3Conjugate(m_referenceRotation) * b3Conjugate(qA) * qB;
		
		b3Vec4 dv = b3_q_to_v(dq);
		
		b3Vec3 C = b3Mat34_P * dv;

		angularError += b3Length(C);

		b3Mat44 G1 = -scalar(0.5) * b3_iQ_mat(b3Conjugate(qA)) * b3_iP_mat(qB);
		b3Mat44 G2 = scalar(0.5) * b3_iQ_mat(b3Conjugate(qA)) * b3_iP_mat(qB);

		b3Mat33 J1 = b3Mat34_P_lock * G1 * b3Mat43_PT;
		b3Mat33 J2 = b3Mat34_P_lock * G2 * b3Mat43_PT;

		b3Mat33 J1T = b3Transpose(J1);
		b3Mat33 J2T = b3Transpose(J2);

		b3Mat33 mass = J1 * iA * J1T + J2 * iB * J2T;
		
		b3Vec3 impulse = mass.Solve(-C);

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

b3Vec3 b3WeldJoint::GetAnchorA() const
{
	return GetBodyA()->GetWorldPoint(m_localAnchorA);
}

b3Vec3 b3WeldJoint::GetAnchorB() const
{
	return GetBodyB()->GetWorldPoint(m_localAnchorB);
}

void b3WeldJoint::Draw(b3Draw* draw) const
{
	b3Vec3 a = GetAnchorA();
	draw->DrawPoint(a, scalar(4), b3Color_red);
	
	b3Vec3 b = GetAnchorB();
	draw->DrawPoint(b, scalar(4), b3Color_green);
	
	draw->DrawSegment(a, b, b3Color_yellow);
}
