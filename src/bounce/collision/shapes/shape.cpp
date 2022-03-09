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

#include <bounce/collision/shapes/shape.h>
#include <bounce/collision/shapes/sphere_shape.h>
#include <bounce/collision/shapes/capsule_shape.h>
#include <bounce/collision/shapes/triangle_shape.h>
#include <bounce/collision/shapes/hull_shape.h>
#include <bounce/collision/shapes/mesh_shape.h>
#include <bounce/collision/geometry/hull.h>
#include <bounce/collision/geometry/mesh.h>
#include <bounce/common/draw.h>

void b3Shape::Draw(b3Draw* draw, const b3Transform& xf, const b3Color& color) const
{
	switch (m_type)
	{
	case b3Shape::e_sphere:
	{
		const b3SphereShape* sphere = (b3SphereShape*)this;
		b3Vec3 p = xf * sphere->m_center;
		draw->DrawPoint(p, scalar(4), color);
		break;
	}
	case b3Shape::e_capsule:
	{
		const b3CapsuleShape* capsule = (b3CapsuleShape*)this;
		b3Vec3 p1 = xf * capsule->m_vertex1;
		b3Vec3 p2 = xf * capsule->m_vertex2;
		draw->DrawPoint(p1, scalar(4), color);
		draw->DrawPoint(p2, scalar(4), color);
		draw->DrawSegment(p1, p2, color);
		break;
	}
	case b3Shape::e_triangle:
	{
		const b3TriangleShape* triangle = (b3TriangleShape*)this;
		b3Vec3 v1 = xf * triangle->m_vertex1;
		b3Vec3 v2 = xf * triangle->m_vertex2;
		b3Vec3 v3 = xf * triangle->m_vertex3;
		b3Vec3 n = b3Cross(v2 - v1, v3 - v1);
		n.Normalize();
		draw->DrawTriangle(v1, v2, v3, color);
		break;
	}
	case b3Shape::e_hull:
	{
		const b3HullShape* hs = (b3HullShape*)this;
		const b3Hull* hull = hs->m_hull;
		for (uint32 i = 0; i < hull->edgeCount; i += 2)
		{
			const b3HalfEdge* edge = hull->GetEdge(i);
			const b3HalfEdge* twin = hull->GetEdge(i + 1);

			b3Vec3 p1 = xf * hull->vertices[edge->origin];
			b3Vec3 p2 = xf * hull->vertices[twin->origin];

			draw->DrawSegment(p1, p2, color);
		}
		break;
	}
	case b3Shape::e_mesh:
	{
		const b3MeshShape* ms = (b3MeshShape*)this;
		const b3Mesh* mesh = ms->m_mesh;
		for (uint32 i = 0; i < mesh->triangleCount; ++i)
		{
			const b3MeshTriangle* t = mesh->triangles + i;

			b3Vec3 p1 = xf * b3Mul(ms->m_scale, mesh->vertices[t->v1]);
			b3Vec3 p2 = xf * b3Mul(ms->m_scale, mesh->vertices[t->v2]);
			b3Vec3 p3 = xf * b3Mul(ms->m_scale, mesh->vertices[t->v3]);

			draw->DrawTriangle(p1, p2, p3, color);
		}
		break;
	}
	default:
	{
		break;
	}
	};
}

void b3Shape::DrawSolid(b3Draw* draw, const b3Transform& xf, const b3Color& color) const
{
	switch (m_type)
	{
	case b3Shape::e_sphere:
	{
		const b3SphereShape* sphere = (b3SphereShape*)this;

		b3Vec3 center = xf * sphere->m_center;

		draw->DrawSolidSphere(xf.rotation.GetYAxis(), center, sphere->m_radius, color);

		break;
	}
	case b3Shape::e_capsule:
	{
		const b3CapsuleShape* capsule = (b3CapsuleShape*)this;

		b3Vec3 c1 = xf * capsule->m_vertex1;
		b3Vec3 c2 = xf * capsule->m_vertex2;

		draw->DrawSolidCapsule(xf.rotation.GetYAxis(), c1, c2, capsule->m_radius, color);

		break;
	}
	case b3Shape::e_triangle:
	{
		const b3TriangleShape* triangle = (b3TriangleShape*)this;

		b3Vec3 v1 = xf * triangle->m_vertex1;
		b3Vec3 v2 = xf * triangle->m_vertex2;
		b3Vec3 v3 = xf * triangle->m_vertex3;

		b3Vec3 n = b3Cross(v2 - v1, v3 - v1);
		n.Normalize();

		draw->DrawSolidTriangle(-n, v3, v2, v1, color);
		draw->DrawSolidTriangle(n, v1, v2, v3, color);

		break;
	}
	case b3Shape::e_hull:
	{
		const b3HullShape* hullShape = (b3HullShape*)this;

		const b3Hull* hull = hullShape->m_hull;

		for (uint32 i = 0; i < hull->faceCount; ++i)
		{
			const b3Face* face = hull->GetFace(i);
			const b3HalfEdge* begin = hull->GetEdge(face->edge);

			b3Vec3 n = b3Mul(xf.rotation, hull->planes[i].normal);

			const b3HalfEdge* edge = hull->GetEdge(begin->next);
			do
			{
				uint32 i1 = begin->origin;
				uint32 i2 = edge->origin;
				const b3HalfEdge* next = hull->GetEdge(edge->next);
				uint32 i3 = next->origin;

				b3Vec3 p1 = xf * hull->vertices[i1];
				b3Vec3 p2 = xf * hull->vertices[i2];
				b3Vec3 p3 = xf * hull->vertices[i3];

				draw->DrawSolidTriangle(n, p1, p2, p3, color);

				edge = next;
			} while (hull->GetEdge(edge->next) != begin);
		}

		break;
	}
	case b3Shape::e_mesh:
	{
		const b3MeshShape* meshShape = (b3MeshShape*)this;

		const b3Mesh* mesh = meshShape->m_mesh;
		for (uint32 i = 0; i < mesh->triangleCount; ++i)
		{
			const b3MeshTriangle* t = mesh->triangles + i;

			b3Vec3 p1 = xf * b3Mul(meshShape->m_scale, mesh->vertices[t->v1]);
			b3Vec3 p2 = xf * b3Mul(meshShape->m_scale, mesh->vertices[t->v2]);
			b3Vec3 p3 = xf * b3Mul(meshShape->m_scale, mesh->vertices[t->v3]);

			b3Vec3 n1 = b3Cross(p2 - p1, p3 - p1);
			n1.Normalize();
			draw->DrawSolidTriangle(n1, p1, p2, p3, color);

			b3Vec3 n2 = -n1;
			draw->DrawSolidTriangle(n2, p3, p2, p1, color);
		}

		break;
	}
	default:
	{
		break;
	}
	};
}
