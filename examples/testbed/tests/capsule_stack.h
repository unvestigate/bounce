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

#ifndef CAPSULE_STACK_H
#define CAPSULE_STACK_H

class CapsuleStack : public Test
{
public:
	enum
	{
		e_h = 5,
		e_w = 1,
		e_d = 1
	};

	CapsuleStack()
	{
		{
			b3BodyDef bd;
			b3Body* body = m_world.CreateBody(bd);

			b3HullShape hs;
			hs.m_hull = &m_groundHull;

			b3FixtureDef fd;
			fd.shape = &hs;
			fd.friction = 1.0f;

			body->CreateFixture(fd);
		}

		scalar rx = 1.0f;
		scalar r = 1.0f;

		b3CapsuleShape capsule;
		capsule.m_vertex1.Set(-rx, 0.0f, 0.0f);
		capsule.m_vertex2.Set(rx, 0.0f, 0.0f);
		capsule.m_radius = r;

		b3Vec3 e;
		e.x = rx + r;
		e.y = r;
		e.z = r;

		b3Vec3 separation;
		separation.x = 1.0f;
		separation.y = 1.0f;
		separation.z = 1.0f;

		b3Vec3 scale;
		scale.x = 2.0f * e.x + separation.x;
		scale.y = 2.0f * e.y + separation.y;
		scale.z = 2.0f * e.z + separation.z;

		b3Vec3 size;
		size.x = 2.0f * e.x + scale.x * scalar(e_w - 1);
		size.y = 2.0f * e.y + scale.y * scalar(e_h - 1);
		size.z = 2.0f * e.z + scale.z * scalar(e_d - 1);

		b3Vec3 translation;
		translation.x = e.x - 0.5f * size.x;
		translation.y = e.y - 0.5f * size.y;
		translation.z = e.z - 0.5f * size.z;

		translation.y += 9.0f;

		for (uint32 i = 0; i < e_h; ++i)
		{
			for (uint32 j = 0; j < e_w; ++j)
			{
				for (uint32 k = 0; k < e_d; ++k)
				{
					b3BodyDef bd;
					bd.type = e_dynamicBody;

					bd.position.Set(scalar(j), scalar(i), scalar(k));

					bd.position.x *= scale.x;
					bd.position.y *= scale.y;
					bd.position.z *= scale.z;

					bd.position += translation;

					b3Body* body = m_world.CreateBody(bd);

					b3FixtureDef fd;
					fd.density = 0.1f;
					fd.friction = 0.3f;
					fd.shape = &capsule;

					body->CreateFixture(fd);

					uint32 bodyIndex = GetBodyIndex(i, j, k);

					m_bodies[bodyIndex] = body;
				}
			}
		}
	}

	uint32 GetBodyIndex(uint32 i, uint32 j, uint32 k)
	{
		B3_ASSERT(i < e_h);
		B3_ASSERT(j < e_w);
		B3_ASSERT(k < e_d);
		return k + e_d * (j + e_w * i);
	}

	static Test* Create()
	{
		return new CapsuleStack();
	}

	b3Body* m_bodies[e_h * e_w * e_d];
};

#endif