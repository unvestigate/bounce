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

#ifndef SHAPE_CAST_H
#define SHAPE_CAST_H

class ShapeCast : public Test
{
public:
	ShapeCast()
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
			bd.position.Set(0.0f, 2.0f, 0.0f);

			b3Body* body = m_world.CreateBody(bd);

			b3HullShape hs;
			hs.m_hull = &m_boxHull;

			b3FixtureDef fd;
			fd.shape = &hs;

			m_fixture = body->CreateFixture(fd);
		}
		
		{
			m_grid.BuildTree();
			m_grid.BuildAdjacency();

			b3BodyDef bd;
			bd.position.Set(-10.0f, 5.0f, -2.0f);
			bd.orientation = b3QuatRotationZ(0.25f * B3_PI);

			b3Body* body = m_world.CreateBody(bd);

			b3MeshShape hs;
			hs.m_mesh = &m_grid;
			hs.m_scale.Set(-1.0f, -1.0f, -2.0f);

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

		m_d.Set(-50.0f, 2.0f, 0.0f);
	}

	void CastShape() 
	{
		class ShapeCastFilter : public b3ShapeCastFilter
		{
		public:
			bool ShouldShapeCast(b3Fixture* fixture)
			{
				return true;
			}
		};

		ShapeCastFilter filter;
		
		b3Shape* shape = m_fixture->GetShape();
		b3Body* body = m_fixture->GetBody();
		b3Transform xf = body->GetTransform();

		shape->DrawSolid(&m_draw, xf, b3Color_red);

		b3Vec3 p1 = xf.translation;

		b3Vec3 p2 = p1 + m_d;
		
		b3ShapeCastSingleOutput out;
		if (m_world.ShapeCastSingle(&out, &filter, shape, xf, m_d))
		{
			b3DrawPoint(g_debugDrawData, out.point, 4.0f, b3Color_red);
			b3DrawSegment(g_debugDrawData, out.point, out.point + out.normal, b3Color_white);
			
			b3Transform xft;
			xft.rotation = xf.rotation;
			xft.translation = xf.translation + out.fraction * m_d;

			shape->Draw(&m_draw, xft, b3Color_red);

			b3DrawSegment(g_debugDrawData, p1, xft.translation, b3Color_green);
		}
		else
		{
			b3DrawSegment(g_debugDrawData, p1, p2, b3Color_green);
			
			b3Transform xf1;
			xf1.rotation = xf.rotation;
			xf1.translation = xf.translation + m_d;
			
			shape->Draw(&m_draw, xf1, b3Color_red);
		}
	}

	void Step()
	{
		scalar dt = g_testSettings->inv_hertz;
		
		b3Quat q = b3QuatRotationY(0.05f * B3_PI * dt);

		m_d = b3Mul(q, m_d);
		
		CastShape();

		Test::Step();
	}

	static Test* Create()
	{
		return new ShapeCast();
	}

	b3GridMesh<5, 5> m_grid;
	b3BoxHull m_boxHull;

	b3Fixture* m_fixture;
	b3Vec3 m_d;
};

#endif
