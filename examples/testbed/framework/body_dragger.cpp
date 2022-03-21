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

#include "body_dragger.h"
#include <bounce/collision/geometry/ray.h>
#include <bounce/dynamics/fixture.h>
#include <bounce/dynamics/body.h>
#include <bounce/dynamics/world.h>
#include <bounce/dynamics/world_callbacks.h>
#include <bounce/dynamics/joints/mouse_joint.h>

BodyDragger::BodyDragger(b3Ray* ray, b3World* world)
{
	m_ray = ray;
	m_world = world;
	m_fixture = nullptr;
	m_mouseJoint = nullptr;
}

bool BodyDragger::StartDragging()
{
	B3_ASSERT(IsDragging() == false);

	class RayCastFilter : public b3RayCastFilter
	{
	public:
		bool ShouldRayCast(b3Fixture* fixture)
		{
			return true;
		}
	};

	RayCastFilter filter;

	b3RayCastSingleOutput out;
	if (m_world->RayCastSingle(&out, &filter, m_ray->A(), m_ray->B()) == false)
	{
		return false;
	}
	
	m_fraction = out.fraction;
	m_fixture = out.fixture;

	b3BodyDef bd;
	b3Body* groundBody = m_world->CreateBody(bd);

	b3Body* body = m_fixture->GetBody();
	body->SetAwake(true);

	b3MouseJointDef jd;
	jd.bodyA = groundBody;
	jd.bodyB = body;
	jd.target = out.point;
	jd.maxForce = 1000.0f * body->GetMass();

	m_mouseJoint = (b3MouseJoint*)m_world->CreateJoint(jd);

	m_localPoint = body->GetLocalPoint(out.point);

	return true;
}

void BodyDragger::Drag()
{
	B3_ASSERT(IsDragging() == true);
	m_mouseJoint->SetTarget(GetPointB());
}

void BodyDragger::StopDragging()
{
	B3_ASSERT(IsDragging() == true);
	b3Body* groundBody = m_mouseJoint->GetBodyA();
	m_world->DestroyJoint(m_mouseJoint);
	m_mouseJoint = nullptr;
	m_world->DestroyBody(groundBody);
	m_fixture = nullptr;
}

b3Fixture* BodyDragger::GetFixture() const
{
	B3_ASSERT(IsDragging() == true);
	return m_fixture;
}

b3Vec3 BodyDragger::GetPointA() const
{
	B3_ASSERT(IsDragging() == true);
	return m_fixture->GetBody()->GetWorldPoint(m_localPoint);
}

b3Vec3 BodyDragger::GetPointB() const
{
	B3_ASSERT(IsDragging() == true);
	return (1.0f - m_fraction) * m_ray->A() + m_fraction * m_ray->B();
}
