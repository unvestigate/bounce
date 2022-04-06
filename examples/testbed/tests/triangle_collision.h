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

#ifndef TRIANGLE_COLLISION_H
#define TRIANGLE_COLLISION_H

#include <bounce/collision/geometry/triangle_hull.h>

class TriangleCollision : public Test
{
public:
	TriangleCollision()
	{
		b3Vec3 v1(-5.0f, 0.0f, 5.0f);
		b3Vec3 v2(5.0f, 0.0f, 5.0f);
		b3Vec3 v3(0.0f, 0.0f, -5.0f);
		m_triangleA.Set(v1, v2, v3);
		m_xfA.SetIdentity();

		m_boxB.SetIdentity();
		m_xfB.translation.Set(-8.47039191e-05, 0.815274179, 0.000109703680);
		m_xfB.rotation.v.Set(1.14654631e-06, -0.00395884318, 4.41140147e-09);
		m_xfB.rotation.s = 0.9999921920;
	}

	void KeyDown(int key)
	{
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

	void Step()
	{
		DrawString(b3Color_white, "Left/Right/Up/Down Arrow - Translate shape");
		DrawString(b3Color_white, "X/Y/Z - Rotate shape");

		b3HullShape hA;
		hA.m_hull = &m_triangleA;
		hA.m_radius = 0.0f;

		b3HullShape hB;
		hB.m_hull = &m_boxB;
		hB.m_radius = 0.0f;

		b3Manifold manifold;
		manifold.Initialize();

		b3CollideHulls(manifold, m_xfA, &hA, m_xfB, &hB);

		for (uint32 i = 0; i < manifold.pointCount; ++i)
		{
			b3WorldManifold wm;
			wm.Initialize(&manifold, hA.m_radius, m_xfA, hB.m_radius, m_xfB);

			b3Vec3 pw = wm.points[i].point;

			b3DrawPoint(g_debugDrawData, pw, 4.0f, b3Color_green, false);
			b3DrawSegment(g_debugDrawData, pw, pw + wm.points[i].normal, b3Color_white, false);
		}

		hA.Draw(&m_draw, m_xfA, b3Color_black);
		hB.Draw(&m_draw, m_xfB, b3Color_black);

		hA.DrawSolid(&m_draw, m_xfA, b3Color(1.0f, 1.0f, 1.0f, 0.25f));
		hB.DrawSolid(&m_draw, m_xfB, b3Color(1.0f, 1.0f, 1.0f, 0.25f));
	}

	static Test* Create()
	{
		return new TriangleCollision();
	}
	
	b3TriangleHull m_triangleA;
	b3BoxHull m_boxB;
	b3Transform m_xfA;
	b3Transform m_xfB;
};

#endif
