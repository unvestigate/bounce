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

#ifndef MESH_TEST_H
#define MESH_TEST_H

class MeshContactTest : public Test
{
public:
	MeshContactTest()
	{
		m_boxHull.SetIdentity();

		m_gridMesh.BuildTree();
		m_gridMesh.BuildAdjacency();

		// Transform grid into a terrain
		for (uint32 i = 0; i < m_terrainMesh.vertexCount; ++i)
		{
			m_terrainMesh.vertices[i].y = RandomFloat(0.0f, 1.0f);
		}

		m_terrainMesh.BuildTree();
		m_terrainMesh.BuildAdjacency();

		{
			b3BodyDef bd;
			b3Body* groundBody = m_world.CreateBody(bd);

			b3MeshShape ms;
			ms.m_mesh = &m_gridMesh;
			ms.m_scale.Set(2.0f, 1.0f, 2.0f);

			b3FixtureDef sd;
			sd.shape = &ms;

			m_groundFixture = groundBody->CreateFixture(sd);
		}

		{
			b3BodyDef bd;
			bd.type = b3BodyType::e_dynamicBody;
			bd.position.Set(0.0f, 5.0f, 0.0f);

			b3Body* body = m_world.CreateBody(bd);

			{
				b3SphereShape sphere;
				sphere.m_center.SetZero();
				sphere.m_radius = 1.0f;

				b3FixtureDef sd;
				sd.shape = &sphere;
				sd.density = 1.0f;
				sd.friction = 0.5f;

				m_bodyFixture = body->CreateFixture(sd);
			}
		}
	}

	void KeyDown(int key)
	{
		if (key == GLFW_KEY_S || key == GLFW_KEY_C || key == GLFW_KEY_H)
		{
			b3Body* body = m_bodyFixture->GetBody();

			m_world.DestroyBody(body);

			b3BodyDef bd;
			bd.type = b3BodyType::e_dynamicBody;
			bd.position.Set(0.0f, 5.0f, 0.0f);

			body = m_world.CreateBody(bd);

			if (key == GLFW_KEY_S)
			{
				b3SphereShape sphere;
				sphere.m_center.SetZero();
				sphere.m_radius = 1.0f;

				b3FixtureDef sd;
				sd.shape = &sphere;
				sd.density = 1.0f;
				sd.friction = 0.5f;

				m_bodyFixture = body->CreateFixture(sd);
			}

			if (key == GLFW_KEY_C)
			{
				b3CapsuleShape capsule;
				capsule.m_vertex1.Set(0.0f, -1.0f, 0.0f);
				capsule.m_vertex2.Set(0.0f, 1.0f, 0.0f);
				capsule.m_radius = 1.0f;

				b3FixtureDef sd;
				sd.shape = &capsule;
				sd.density = 1.0f;
				sd.friction = 0.5f;

				m_bodyFixture = body->CreateFixture(sd);
			}

			if (key == GLFW_KEY_H)
			{
				b3HullShape hull;
				hull.m_hull = &m_boxHull;

				b3FixtureDef sd;
				sd.shape = &hull;
				sd.density = 1.0f;
				sd.friction = 0.5f;

				m_bodyFixture = body->CreateFixture(sd);
			}
		}

		if (key == GLFW_KEY_G || key == GLFW_KEY_T)
		{
			b3Body* groundBody = m_groundFixture->GetBody();
			m_world.DestroyBody(groundBody);

			b3BodyDef bd;
			groundBody = m_world.CreateBody(bd);

			if (key == GLFW_KEY_G)
			{
				b3MeshShape ms;
				ms.m_mesh = &m_gridMesh;
				ms.m_scale.Set(2.0f, 1.0f, 2.0f);

				b3FixtureDef sd;
				sd.shape = &ms;

				m_groundFixture = groundBody->CreateFixture(sd);
			}

			if (key == GLFW_KEY_T)
			{
				b3MeshShape ms;
				ms.m_mesh = &m_terrainMesh;
				ms.m_scale.Set(2.0f, 1.5f, 2.0f);

				b3FixtureDef sd;
				sd.shape = &ms;

				m_groundFixture = groundBody->CreateFixture(sd);
			}
		}
	}

	void Step()
	{
		Test::Step();

		DrawString(b3Color_white, "S - Sphere");
		DrawString(b3Color_white, "C - Capsule");
		DrawString(b3Color_white, "H - Hull");
		DrawString(b3Color_white, "G - Grid");
		DrawString(b3Color_white, "T - Terrain");
	}

	static Test* Create()
	{
		return new MeshContactTest();
	}

	b3GridMesh<25, 25> m_terrainMesh;
	b3GridMesh<25, 25> m_gridMesh;
	b3BoxHull m_boxHull;

	b3Fixture* m_groundFixture;
	b3Fixture* m_bodyFixture;
};

#endif
