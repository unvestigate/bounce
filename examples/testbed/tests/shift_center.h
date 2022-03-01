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
		// Transform grid into a terrain
		for (u32 i = 0; i < m_terrainMesh.vertexCount; ++i)
		{
			m_terrainMesh.vertices[i].y = RandomFloat(0.0f, 1.0f);
		}

		m_terrainMesh.BuildTree();
		m_terrainMesh.BuildAdjacency();

		{
			b3BodyDef bd;
			b3Body* body = m_world.CreateBody(bd);

			b3MeshShape hs;
			hs.m_mesh = &m_terrainMesh;

			b3FixtureDef sd;
			sd.shape = &hs;

			body->CreateFixture(sd);
		}

		{
			b3BodyDef bd;
			bd.type = e_dynamicBody;
			bd.position.Set(0.0f, 5.0f, 0.0f);

			b3Body* body = m_world.CreateBody(bd);

			m_box.SetExtents(1.0f, 1.0f, 1.0f);

			b3HullShape hs;
			hs.m_hull = &m_box;

			b3FixtureDef sd;
			sd.shape = &hs;
			sd.density = 0.1f;

			body->CreateFixture(sd);

			b3MassData md;
			body->GetMassData(&md);

			// Shift inertia to old center of mass
			b3Mat33 Ic = md.I - md.mass * b3Steiner(md.center);

			// Change center of mass
			md.center.Set(4.0f, 0.0f, 0.0f);

			// Shift inertia to local body origin
			b3Mat33 I2 = Ic + md.mass * b3Steiner(md.center);

			md.I = I2;

			body->SetMassData(&md);
		}
	}

	static Test* Create()
	{
		return new ShiftCenter();
	}

	b3BoxHull m_box;
	b3GridMesh<25, 25> m_terrainMesh;
};

#endif