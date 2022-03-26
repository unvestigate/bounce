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

#ifndef RAY_CAST_H
#define RAY_CAST_H

class RayCast : public Test
{
public:
	RayCast()
	{
		m_boxHull.SetIdentity();

		{
			b3BodyDef bd;
			b3Body* body = m_world.CreateBody(bd);

			b3HullShape hs;
			hs.m_hull = &m_groundHull;

			b3FixtureDef fd;
			fd.shape = &hs;

			body->CreateFixture(fd);
		}

		{
			b3BodyDef bd;
			bd.position.Set(0.0f, 2.0f, 10.0f);
			bd.orientation = b3QuatRotationY(0.25f * B3_PI);

			b3Body* body = m_world.CreateBody(bd);

			b3HullShape hs;
			hs.m_hull = &m_boxHull;

			b3FixtureDef fd;
			fd.shape = &hs;

			body->CreateFixture(fd);
		}

		{
			b3BodyDef bd;
			bd.position.Set(-10.0f, 6.0f, -10.0f);
			bd.orientation = b3QuatRotationY(0.25f * B3_PI);

			b3Body* body = m_world.CreateBody(bd);
			
			static b3BoxHull boxHull(2.0f, 4.0f, 0.5f);

			b3HullShape hs;
			hs.m_hull = &boxHull;

			b3FixtureDef fd;
			fd.shape = &hs;

			body->CreateFixture(fd);
		}

		{
			b3BodyDef bd;
			bd.position.Set(10.0f, 2.0f, 0.0f);
			bd.orientation = b3QuatRotationY(0.20f * B3_PI);

			b3Body* body = m_world.CreateBody(bd);

			b3HullShape hs;
			hs.m_hull = &m_boxHull;

			b3FixtureDef fd;
			fd.shape = &hs;

			body->CreateFixture(fd);
		}

		{
			b3BodyDef bd;
			bd.position.Set(-10.0f, 2.0f, 14.0f);
			bd.orientation = b3QuatRotationY(0.05f * B3_PI);

			b3Body* body = m_world.CreateBody(bd);

			b3HullShape hs;
			hs.m_hull = &m_boxHull;

			b3FixtureDef fd;
			fd.shape = &hs;

			body->CreateFixture(fd);
		}

		{
			b3BodyDef bd;
			bd.position.Set(-14.0f, 2.0f, 5.0f);
			bd.orientation = b3QuatRotationY(-0.05f * B3_PI);

			b3Body* body = m_world.CreateBody(bd);

			b3HullShape hs;
			hs.m_hull = &m_boxHull;

			b3FixtureDef fd;
			fd.shape = &hs;

			body->CreateFixture(fd);
		}

		{
			b3BodyDef bd;
			bd.position.Set(20.0f, 2.0f, 5.0f);
			bd.orientation = b3QuatRotationY(-0.05f * B3_PI);

			b3Body* body = m_world.CreateBody(bd);

			b3HullShape hs;
			hs.m_hull = &m_boxHull;

			b3FixtureDef fd;
			fd.shape = &hs;

			body->CreateFixture(fd);
		}
		
		{
			b3BodyDef bd;
			bd.position.Set(12.0f, 2.0f, 5.0f);
			bd.orientation = b3QuatRotationY(-0.35f * B3_PI);

			b3Body* body = m_world.CreateBody(bd);

			b3SphereShape hs;
			hs.m_center.SetZero();
			hs.m_radius = 2.5f;

			b3FixtureDef fd;
			fd.shape = &hs;

			body->CreateFixture(fd);
		}

		{
			b3BodyDef bd;
			bd.position.Set(0.0f, 1.0f, -12.0f);

			b3Body* body = m_world.CreateBody(bd);

			b3CapsuleShape hs;
			hs.m_vertex1.Set(0.0f, 1.0f, 0.0f);
			hs.m_vertex2.Set(0.0f, -1.0f, 0.0f);
			hs.m_radius = 3.0f;

			b3FixtureDef fd;
			fd.shape = &hs;

			body->CreateFixture(fd);
		}

		m_p1.Set(0.0f, 2.0f, 0.0f);
		m_p2.Set(50.0f, 2.0f, 0.0f);

		m_p12.Set(0.0f, 2.0f, 0.0f);
		m_p22.Set(-50.0f, 2.0f, 0.0f);
	}
	
	void CastRay(const b3Vec3 p1, const b3Vec3 p2) const
	{
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
		if (m_world.RayCastSingle(&out, &filter, p1, p2))
		{
			b3DrawSegment(g_debugDrawData, p1, out.point, b3Color_green);
			
			b3DrawPoint(g_debugDrawData, out.point, 4.0f, b3Color_red);
			b3DrawSegment(g_debugDrawData, out.point, out.point + out.normal, b3Color_white);
		}
		else
		{
			b3DrawSegment(g_debugDrawData, p1, p2, b3Color_green);
		}
	}

	void Step()
	{
		scalar dt = g_testSettings->inv_hertz;
		b3Quat dq = b3QuatRotationY(0.05f * B3_PI * dt);
		
		m_p1 = b3Mul(dq, m_p1);
		m_p2 = b3Mul(dq, m_p2);
		CastRay(m_p1, m_p2);

		m_p12 = b3Mul(dq, m_p12);
		m_p22 = b3Mul(dq, m_p22);
		CastRay(m_p12, m_p22);

		Test::Step();
	}

	static Test* Create()
	{
		return new RayCast();
	}

	b3BoxHull m_boxHull;
	b3Vec3 m_p1, m_p2;
	b3Vec3 m_p12, m_p22;
};

#endif
