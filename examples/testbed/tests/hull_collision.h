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

#ifndef HULL_COLLISION_H
#define HULL_COLLISION_H

#include <bounce/collision/sat/sat.h>

class HullCollision : public Test
{
public:
	enum
	{
		e_count = 256
	};

	HullCollision()
	{
		m_hullA = nullptr;
		m_xfA.translation.Set(0.0f, 1.5f, 0.0f);
		m_xfA.rotation.SetIdentity();

		m_hullB = nullptr;
		m_xfB.SetIdentity();

		Generate();
	}

	~HullCollision()
	{
		if (m_hullA)
		{
			DestroyHull(m_hullA);
		}

		if (m_hullB)
		{
			DestroyHull(m_hullB);
		}
	}

	void KeyDown(int key)
	{
		if (key == GLFW_KEY_G)
		{
			Generate();
		}
		
		if (key == GLFW_KEY_LEFT)
		{
			m_xfB.translation.x -= 0.05f;
		}

		if (key == GLFW_KEY_RIGHT)
		{
			m_xfB.translation.x += 0.05f;
		}

		if (key == GLFW_KEY_UP)
		{
			m_xfB.translation.y += 0.05f;
		}

		if (key == GLFW_KEY_DOWN)
		{
			m_xfB.translation.y -= 0.05f;
		}

		if (key == GLFW_KEY_X)
		{
			b3Quat qx = b3QuatRotationX(0.05f * B3_PI);

			m_xfB.rotation = m_xfB.rotation * qx;
		}

		if (key == GLFW_KEY_Y)
		{
			b3Quat qy = b3QuatRotationY(0.05f * B3_PI);

			m_xfB.rotation = m_xfB.rotation * qy;
		}

		if (key == GLFW_KEY_Z)
		{
			b3Quat qz = b3QuatRotationZ(0.05f * B3_PI);

			m_xfB.rotation = m_xfB.rotation * qz;
		}
	}

	void Generate()
	{
		if (m_hullA)
		{
			DestroyHull(m_hullA);
			m_hullA = nullptr;
		}

		if (m_hullB)
		{
			DestroyHull(m_hullB);
			m_hullB = nullptr;
		}

		b3Vec3 points1[e_count];
		for (uint32 i = 0; i < e_count; ++i)
		{
			float x = 3.0f * RandomFloat(-1.0f, 1.0f);
			float y = 3.0f * RandomFloat(-1.0f, 1.0f);
			float z = 3.0f * RandomFloat(-1.0f, 1.0f);

			x = b3Clamp(x, -2.5f, 2.5f);
			y = b3Clamp(y, -2.5f, 2.5f);
			z = b3Clamp(z, -2.5f, 2.5f);

			b3Vec3 p(x, y, z);

			points1[i] = p;
		}

		m_hullA = CreateHull(points1, e_count);
		
		b3Vec3 points2[e_count];
		for (uint32 i = 0; i < e_count; ++i)
		{
			float x = 3.0f * RandomFloat(-1.0f, 1.0f);
			float y = 3.0f * RandomFloat(-1.0f, 1.0f);
			float z = 3.0f * RandomFloat(-1.0f, 1.0f);

			x = b3Clamp(x, -2.5f, 2.5f);
			y = b3Clamp(y, -2.5f, 2.5f);
			z = b3Clamp(z, -2.5f, 2.5f);

			b3Vec3 p(x, y, z);

			points2[i] = p;
		}

		m_hullB = CreateHull(points2, e_count);
	}
	
	void Step()
	{
		DrawString(b3Color_white, "G - Generate random convex hulls");

		if (!m_hullA || !m_hullB)
		{
			return;
		}

		b3HullShape sA;
		sA.m_hull = m_hullA;

		b3HullShape sB;
		sB.m_hull = m_hullB;

		b3Manifold manifold;
		manifold.Initialize();

		b3FeatureCache cache;

		b3CollideHulls(manifold, m_xfA, &sA, m_xfB, &sB, cache, m_xfA, m_xfB);

		for (uint32 i = 0; i < manifold.pointCount; ++i)
		{
			b3WorldManifold wm;
			wm.Initialize(&manifold, sA.m_radius, m_xfA, sB.m_radius, m_xfB);

			b3Vec3 pw = wm.points[i].point;

			b3DrawPoint(g_debugDrawData, pw, 4.0f, b3Color_green, false);
			b3DrawSegment(g_debugDrawData, pw, pw + wm.points[i].normal, b3Color_white, false);
		}

		sA.Draw(&m_draw, m_xfA, b3Color_black);
		sB.Draw(&m_draw, m_xfB, b3Color_black);

		sA.DrawSolid(&m_draw, m_xfA, b3Color(1.0f, 1.0f, 1.0f, 0.25f));
		sB.DrawSolid(&m_draw, m_xfB, b3Color(1.0f, 1.0f, 1.0f, 0.25f));

		DrawString(b3Color_white, "Left/Right/Up/Down Arrow - Translate shape");
		DrawString(b3Color_white, "X/Y/Z - Rotate shape");
	}

	static Test* Create()
	{
		return new HullCollision();
	}

	b3Hull* m_hullA;
	b3Transform m_xfA;
	
	b3Hull* m_hullB;
	b3Transform m_xfB;
};

#endif
