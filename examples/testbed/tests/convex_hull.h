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

#ifndef CONVEX_HULL_H
#define CONVEX_HULL_H

class ConvexHull : public Test
{
public:
	enum
	{
		e_count = 256
	};

	ConvexHull()
	{
		m_hull = nullptr;
		Generate();
	}

	~ConvexHull()
	{
		if (m_hull)
		{
			DestroyHull(m_hull);
		}
	}

	void KeyDown(int button)
	{
		if (m_hull)
		{
			if (button == GLFW_KEY_LEFT)
			{
				if (m_selection > 0)
				{
					m_selection -= 1;
				}
			}

			if (button == GLFW_KEY_RIGHT)
			{
				if (m_selection < m_hull->faceCount - 1)
				{
					m_selection += 1;
				}
			}
		}

		if (button == GLFW_KEY_G)
		{
			Generate();
		}
	}

	void Generate()
	{
		if (m_hull)
		{
			DestroyHull(m_hull);
			m_hull = nullptr;
		}

		for (uint32 i = 0; i < e_count; ++i)
		{
			float x = 3.0f * RandomFloat(-1.0f, 1.0f);
			float y = 3.0f * RandomFloat(-1.0f, 1.0f);
			float z = 3.0f * RandomFloat(-1.0f, 1.0f);
			
			// Clamp to force coplanarities.
			// This will stress the convex hull creation code.
			x = b3Clamp(x, -2.5f, 2.5f);
			y = b3Clamp(y, -2.5f, 2.5f);
			z = b3Clamp(z, -2.5f, 2.5f);

			b3Vec3 p(x, y, z);
			
			m_points[i] = p;
		}
		
		m_hull = CreateHull(m_points, e_count);
		if (m_hull)
		{
			m_selection = m_hull->GetSupportFace(b3Vec3_z);
		}
	}

	void Step()
	{
		DrawString(b3Color_white, "G - Generate a random convex hull");
		DrawString(b3Color_white, "Left/Right Arrow - Select previous/next convex hull face");

		//Generate();

		if (!m_hull)
		{
			return;
		}

		for (uint32 i = 0; i < m_hull->vertexCount; ++i)
		{
			b3DrawPoint(g_debugDrawData, m_hull->vertices[i], 4.0f, b3Color_green, false);
		}

		for (uint32 i = 0; i < e_count; ++i)
		{
			b3DrawPoint(g_debugDrawData, m_points[i], 4.0f, b3Color_black);
		}

		{
			const b3Face* face = m_hull->GetFace(m_selection);
			b3Vec3 n = m_hull->GetPlane(m_selection).normal;

			b3Vec3 c; 
			c.SetZero();
			
			uint32 vn = 0;
			
			const b3HalfEdge* begin = m_hull->GetEdge(face->edge);
			const b3HalfEdge* edge = begin;
			do
			{
				uint32 i1 = edge->origin;
				b3Vec3 v1 = m_hull->GetVertex(i1);
				
				const b3HalfEdge* twin = m_hull->GetEdge(edge->twin);
				uint32 i2 = twin->origin;
				b3Vec3 v2 = m_hull->GetVertex(i2);

				c += v1;
				++vn;

				b3DrawSegment(g_debugDrawData, v1, v2, b3Color_green, false);

				DrawString(b3Color_white, v1, "v%d", vn);
				
				edge = m_hull->GetEdge(edge->next);
			} while (edge != begin);

			c /= scalar(vn);
			b3DrawSegment(g_debugDrawData, c, c + n, b3Color_white, false);
		}

		for (uint32 i = 0; i < m_hull->faceCount; ++i)
		{
			const b3Face* face = m_hull->GetFace(i);

			b3Vec3 n = m_hull->GetPlane(i).normal;

			const b3HalfEdge* begin = m_hull->GetEdge(face->edge);
			const b3HalfEdge* edge = begin;
			do
			{
				uint32 i1 = begin->origin;
				uint32 i2 = edge->origin;
				const b3HalfEdge* next = m_hull->GetEdge(edge->next);
				uint32 i3 = next->origin;

				b3Vec3 v1 = m_hull->GetVertex(i1);
				b3Vec3 v2 = m_hull->GetVertex(i2);
				b3Vec3 v3 = m_hull->GetVertex(i3);

				b3Color solidColor(0.75f, 0.75f, 0.75f);
				if (i == m_selection)
				{
					solidColor.r = 0.5f;
					solidColor.g = 0.5f;
					solidColor.b = 1.0f;
				}
				
				b3DrawSolidTriangle(g_debugDrawData, n, v1, v2, v3, solidColor);

				edge = next;
			} while (edge != begin);
		}
	}

	static Test* Create()
	{
		return new ConvexHull();
	}

	b3Hull* m_hull;
	uint32 m_selection;
	b3Vec3 m_points[e_count];
};

#endif
