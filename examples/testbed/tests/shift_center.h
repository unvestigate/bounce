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

#ifndef SHIFT_CENTER_H
#define SHIFT_CENTER_H

class ShiftCenter : public Test
{
public:
	ShiftCenter()
	{
		{
			b3BodyDef bd;
			b3Body* body = m_world.CreateBody(bd);

			// Transform grid into a terrain
			for (uint32 i = 0; i < m_terrain.vertexCount; ++i)
			{
				m_terrain.vertices[i].x *= 4.0f;
				m_terrain.vertices[i].y = RandomFloat(0.0f, 1.0f);
				m_terrain.vertices[i].z *= 4.0f;
			}

			// Rotate the terrain body
			b3Quat orientation;
			orientation.SetAxisAngle(b3Vec3_y, 0.25f * B3_PI);

			b3Transform transform(body->GetPosition(), orientation);
			body->SetTransform(transform);

			m_terrain.BuildTree();
			m_terrain.BuildAdjacency();

			b3MeshShape ms;
			ms.m_mesh = &m_terrain;

			b3FixtureDef fd;
			fd.shape = &ms;

			body->CreateFixture(fd);
		}

		{
			b3BodyDef bd;
			bd.type = e_dynamicBody;
			bd.position.Set(0.0f, 5.0f, 0.0f);

			b3Body* body = m_world.CreateBody(bd);

			m_box.SetExtents(2.0f, 0.5f, 1.0f);

			b3HullShape hs;
			hs.m_hull = &m_box;

			b3FixtureDef fd;
			fd.shape = &hs;
			fd.density = 0.1f;

			body->CreateFixture(fd);

			// Get the current mass data
			b3MassData md;
			body->GetMassData(&md);

			// Shift inertia to old center of mass
			b3Mat33 Ic = md.I - md.mass * b3Steiner(md.center);

			// Change the center of mass
			md.center.Set(0.0f, -3.0f, 0.0f);

			// Shift inertia to local body origin
			b3Mat33 I2 = Ic + md.mass * b3Steiner(md.center);

			md.I = I2;

			// Now Bounce will find Ic correctly
			body->SetMassData(&md);
		}
	}

	static Test* Create()
	{
		return new ShiftCenter();
	}

	b3GridMesh<25, 25> m_terrain;
	b3BoxHull m_box;
};

#endif