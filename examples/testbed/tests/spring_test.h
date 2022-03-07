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

#ifndef SPRING_TEST_H
#define SPRING_TEST_H

class SpringTest : public Test
{
public:
	SpringTest()
	{
		{
			b3BodyDef bd;
			b3Body* ground = m_world.CreateBody(bd);

			b3HullShape hs;
			hs.m_hull = &m_groundHull;

			b3FixtureDef sd;
			sd.shape = &hs;

			ground->CreateFixture(sd);
		}

		// Car frame shape
		m_frameHull.SetExtents(2.0f, 0.5f, 5.0f);

		b3HullShape box;
		box.m_hull = &m_frameHull;

		// Wheel shape
		b3SphereShape sphere;
		sphere.m_center.SetZero();
		sphere.m_radius = 1.0f;

		// Car frame
		b3Body* frame;
		{
			b3BodyDef bd;
			bd.type = e_dynamicBody;
			bd.position.Set(0.0f, 10.0f, 0.0f);

			frame = m_world.CreateBody(bd);

			b3FixtureDef fd;
			fd.density = 0.1f;
			fd.friction = 0.3f;
			fd.shape = &box;

			frame->CreateFixture(fd);
		}

		b3Body* wheelLF;
		{
			b3BodyDef bd;
			bd.type = e_dynamicBody;
			bd.position.Set(-1.0f, 7.0f, 4.5f);
			bd.fixedRotationY = true;

			wheelLF = m_world.CreateBody(bd);

			b3FixtureDef fd;
			fd.shape = &sphere;
			fd.density = 0.1f;
			fd.friction = 1.0f;

			wheelLF->CreateFixture(fd);
		}

		{
			b3SpringJointDef def;
			def.Initialize(frame, wheelLF, b3Vec3(-1.0f, 9.0f, 4.5), b3Vec3(-1.0f, 9.0f, 4.5f));
			def.collideConnected = true;
			def.dampingRatio = 0.5f;
			def.frequencyHz = 4.0f;

			m_world.CreateJoint(def);
		}

		b3Body* wheelRF;
		{
			b3BodyDef bd;
			bd.type = e_dynamicBody;
			bd.position.Set(1.0f, 7.0, 4.5f);
			bd.fixedRotationY = true;

			wheelRF = m_world.CreateBody(bd);

			b3FixtureDef fd;
			fd.density = 0.1f;
			fd.friction = 1.0f;
			fd.shape = &sphere;

			wheelRF->CreateFixture(fd);
		}

		{
			b3SpringJointDef def;
			def.Initialize(frame, wheelRF, b3Vec3(1.0f, 9.0, 4.5), b3Vec3(1.0f, 9.0, 4.5f));
			def.collideConnected = true;
			def.dampingRatio = 0.5f;
			def.frequencyHz = 4.0f;

			m_world.CreateJoint(def);
		}

		b3Body* wheelLB;
		{
			b3BodyDef bd;
			bd.type = e_dynamicBody;
			bd.position.Set(-1.0f, 7.0f, -4.5f);
			bd.fixedRotationY = true;

			wheelLB = m_world.CreateBody(bd);

			b3FixtureDef fd;
			fd.shape = &sphere;
			fd.density = 0.1f;
			fd.friction = 1.0f;

			wheelLB->CreateFixture(fd);
		}

		{
			b3SpringJointDef def;
			def.Initialize(frame, wheelLB, b3Vec3(-1.0f, 9.0f, -4.5f), b3Vec3(-1.0f, 9.0f, -4.5f));
			def.collideConnected = true;
			def.dampingRatio = 0.8f;
			def.frequencyHz = 4.0f;

			m_world.CreateJoint(def);
		}

		b3Body* wheelRB;
		{
			b3BodyDef bd;
			bd.type = e_dynamicBody;
			bd.position.Set(1.0f, 7.0f, -4.5f);
			bd.fixedRotationY = true;

			wheelRB = m_world.CreateBody(bd);

			b3FixtureDef fd;
			fd.density = 0.1f;
			fd.friction = 1.0f;
			fd.shape = &sphere;

			wheelRB->CreateFixture(fd);
		}

		{
			b3SpringJointDef def;
			def.Initialize(frame, wheelRB, b3Vec3(1.0f, 9.0f, -4.5f), b3Vec3(1.0f, 9.0f, -4.5f));
			def.collideConnected = true;
			def.frequencyHz = 4.0f;
			def.dampingRatio = 0.8f;

			m_world.CreateJoint(def);
		}
	}

	static Test* Create()
	{
		return new SpringTest();
	}

	b3BoxHull m_frameHull;
};

#endif