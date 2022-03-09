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

#include <bounce/collision/collide/collide.h>
#include <bounce/collision/shapes/sphere_shape.h>
#include <bounce/collision/shapes/capsule_shape.h>
#include <bounce/collision/shapes/triangle_shape.h>
#include <bounce/collision/shapes/hull_shape.h>
#include <bounce/collision/shapes/mesh_shape.h>
#include <bounce/collision/geometry/sphere.h>
#include <bounce/collision/geometry/capsule.h>
#include <bounce/collision/geometry/hull.h>
#include <bounce/collision/geometry/mesh.h>
#include <bounce/collision/collision.h>

void b3ShapeGJKProxy::Set(const b3Shape* shape, uint32 index)
{
	switch (shape->GetType())
	{
	case b3Shape::e_sphere:
	{
		const b3SphereShape* sphere = (b3SphereShape*)shape;
		vertexCount = 1;
		vertices = &sphere->m_center;
		radius = sphere->m_radius;
		break;
	}
	case b3Shape::e_capsule:
	{
		const b3CapsuleShape* capsule = (b3CapsuleShape*)shape;
		vertexCount = 2;
		vertices = &capsule->m_vertex1;
		radius = capsule->m_radius;
		break;
	}
	case b3Shape::e_triangle:
	{
		const b3TriangleShape* triangle = (b3TriangleShape*)shape;
		vertexCount = 3;
		vertices = &triangle->m_vertex1;
		radius = triangle->m_radius;
		break;
	}
	case b3Shape::e_hull:
	{
		const b3HullShape* hull = (b3HullShape*)shape;
		vertexCount = hull->m_hull->vertexCount;
		vertices = hull->m_hull->vertices;
		radius = hull->m_radius;
		break;
	}
	case b3Shape::e_mesh:
	{
		const b3MeshShape* mesh = (b3MeshShape*)shape;

		B3_ASSERT(index >= 0);
		B3_ASSERT(index < mesh->m_mesh->triangleCount);

		const b3MeshTriangle* triangle = mesh->m_mesh->GetTriangle(index);

		vertexBuffer[0] = b3Mul(mesh->m_scale, mesh->m_mesh->vertices[triangle->v1]);
		vertexBuffer[1] = b3Mul(mesh->m_scale, mesh->m_mesh->vertices[triangle->v2]);
		vertexBuffer[2] = b3Mul(mesh->m_scale, mesh->m_mesh->vertices[triangle->v3]);

		vertexCount = 3;
		vertices = vertexBuffer;
		radius = mesh->m_radius;
		break;
	}
	default:
	{
		B3_ASSERT(false);
		break;
	}
	}
}

bool b3TestOverlap(const b3Transform& xfA, uint32 indexA, const b3Shape* shapeA,
	const b3Transform& xfB, uint32 indexB, const b3Shape* shapeB,
	b3ConvexCache* cache)
{
	b3ShapeGJKProxy proxyA(shapeA, indexA);
	b3ShapeGJKProxy proxyB(shapeB, indexB);
	
	b3GJKOutput distance = b3GJK(xfA, proxyA, xfB, proxyB, true, &cache->simplexCache);

	const scalar kTol = scalar(10) * B3_EPSILON;
	return distance.distance <= kTol;
}