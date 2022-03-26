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

#ifndef VARYING_FRICTION_H
#define VARYING_FRICTION_H

class VaryingFriction : public Test
{
public:
	VaryingFriction()
	{
		m_rampHull.SetExtents(25.0f, 0.5f, 25.0f);
		m_boxHull.SetIdentity();

		{
			b3BodyDef bd;	
			b3Body* ground = m_world.CreateBody(bd);

			b3HullShape hs;
			hs.m_hull = &m_groundHull;

			b3FixtureDef sd;
			sd.shape = &hs;
			ground->CreateFixture(sd);
		}


		{
			b3BodyDef bd;
			bd.position.Set(-20.0f, 20.0f, 0.0f);
			bd.orientation = b3QuatRotationZ(-0.1f * B3_PI);

			b3Body* ramp = m_world.CreateBody(bd);
			
			b3HullShape hs;
			hs.m_hull = &m_rampHull;

			b3FixtureDef sd;
			sd.shape = &hs;
			sd.friction = 0.4f;
			ramp->CreateFixture(sd);
		}

		{
			b3BodyDef bd;
			bd.position.Set(20.0f, 30.0f, 0.0f);
			bd.orientation = b3QuatRotationZ(0.1f * B3_PI);

			b3Body* ramp = m_world.CreateBody(bd);

			b3HullShape hs;
			hs.m_hull = &m_rampHull;

			b3FixtureDef sd;
			sd.shape = &hs;
			sd.friction = 0.3f;
			ramp->CreateFixture(sd);
		}

		{
			b3BodyDef bd;
			bd.position.Set(-20.0f, 40.0f, 0.0f);
			bd.orientation = b3QuatRotationZ(-0.1f * B3_PI);

			b3Body* ramp = m_world.CreateBody(bd);

			b3HullShape hs;
			hs.m_hull = &m_rampHull;

			b3FixtureDef sd;
			sd.shape = &hs;
			sd.friction = 0.2f;
			ramp->CreateFixture(sd);
		}

		{
			b3BodyDef bd;
			bd.position.Set(20.0f, 50.0f, 0.0f);
			bd.orientation = b3QuatRotationZ(0.1f * B3_PI);

			b3Body* ramp = m_world.CreateBody(bd);

			b3HullShape hs;
			hs.m_hull = &m_rampHull;

			b3FixtureDef sd;
			sd.shape = &hs;
			sd.friction = 0.1f;
			ramp->CreateFixture(sd);
		}

		{
			b3BodyDef bd;
			bd.type = b3BodyType::e_dynamicBody;
			bd.position.Set(40.0f, 70.0f, -10.0f);
			b3Body* body = m_world.CreateBody(bd);

			b3HullShape hs;
			hs.m_hull = &m_boxHull;

			b3FixtureDef sd;
			sd.density = 1.0f;
			sd.friction = 0.2f;
			sd.shape = &hs;

			body->CreateFixture(sd);
		}

		{
			b3BodyDef bd;
			bd.type = b3BodyType::e_dynamicBody;
			bd.position.Set(40.0f, 70.0f, 0.0f);
			b3Body* body = m_world.CreateBody(bd);

			b3HullShape hs;
			hs.m_hull = &m_boxHull;

			b3FixtureDef sd;
			sd.density = 1.0f;
			sd.friction = 0.5f;
			sd.shape = &hs;

			body->CreateFixture(sd);
		}

		{
			b3BodyDef bd;
			bd.type = b3BodyType::e_dynamicBody;
			bd.position.Set(40.0f, 70.0f, 10.0f);
			b3Body* body = m_world.CreateBody(bd);

			b3HullShape hs;
			hs.m_hull = &m_boxHull;

			b3FixtureDef sd;
			sd.density = 1.0f;
			sd.friction = 0.8f;
			sd.shape = &hs;

			body->CreateFixture(sd);
		}
	}

	static Test* Create()
	{
		return new VaryingFriction();
	}

	b3BoxHull m_rampHull;
	b3BoxHull m_boxHull;
};

#endif