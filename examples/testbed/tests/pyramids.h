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

#ifndef PYRAMIDS_H
#define PYRAMIDS_H

class Pyramids : public Test
{
public:
	enum
	{
		e_count = 10,
		e_depthCount = 10,
	};

	Pyramids()
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

		m_boxHull.SetIdentity();

		b3Vec3 boxSize;
		boxSize.Set(2.0f, 2.0f, 2.0f);

		// shift to ground center
		b3Vec3 translation;
		translation.x = -0.5f * scalar(e_count - 1) * 4.0f * boxSize.x;
		translation.y = 1.5f * boxSize.y;
		translation.z = -0.5f * scalar(e_depthCount) * boxSize.z;
		
		for (uint32 i = 0; i < e_count; ++i)
		{
			// reset 
			translation.y = 1.5f * boxSize.y;
			translation.z = -0.5f * scalar(e_depthCount) * boxSize.z;

			for (uint32 j = 0; j < e_depthCount; ++j)
			{
				for (uint32 k = j; k < e_depthCount; ++k)
				{
					b3BodyDef bd;
					bd.type = e_dynamicBody;
					bd.position.x = 0.0f;
					bd.position.y = 0.0f;
					bd.position.z = 1.05f * scalar(k) * boxSize.z;
					bd.position += translation;

					b3Body* body = m_world.CreateBody(bd);

					b3HullShape hs;
					hs.m_hull = &m_boxHull;

					b3FixtureDef sd;
					sd.shape = &hs;
					sd.density = 0.5f;
					sd.friction = 0.5f;

					body->CreateFixture(sd);
				}

				// increment column
				translation.y += 1.5f * boxSize.y;
				// track offset
				translation.z -= 0.5f * boxSize.z;
			}

			// increment row
			translation.x += 4.0f * boxSize.x;
		}
	}

	static Test* Create()
	{
		return new Pyramids();
	}

	b3BoxHull m_boxHull;
};

#endif
