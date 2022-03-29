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

#include <bounce/collision/shapes/mesh_shape.h>
#include <bounce/collision/geometry/mesh.h>
#include <bounce/collision/shapes/triangle_shape.h>
#include <bounce/common/memory/block_allocator.h>

b3MeshShape::b3MeshShape() 
{
	m_type = e_mesh;
	m_radius = B3_HULL_RADIUS;
	m_mesh = nullptr;
	m_scale.Set(scalar(1), scalar(1), scalar(1));
}

b3Shape* b3MeshShape::Clone(b3BlockAllocator* allocator) const
{
	void* mem = allocator->Allocate(sizeof(b3MeshShape));
	b3MeshShape* clone = new (mem)b3MeshShape;
	*clone = *this;
	return clone;
}

void b3MeshShape::ComputeMass(b3MassData* massData, scalar density) const 
{
	B3_NOT_USED(density);	
	massData->center.SetZero();
	massData->mass = scalar(0);
	massData->I.SetZero();
}

void b3MeshShape::ComputeAABB(b3AABB* output, const b3Transform& xf) const 
{
	b3AABB aabb;
	aabb.Compute(m_mesh->vertices, m_mesh->vertexCount, m_scale, xf);
	aabb.Extend(m_radius);

	*output = aabb;
}

void b3MeshShape::ComputeAABB(b3AABB* output, const b3Transform& xf, uint32 index) const
{
	B3_ASSERT(index < m_mesh->triangleCount);
	const b3MeshTriangle* triangle = m_mesh->triangles + index;
	
	b3Vec3 v1 = b3Mul(xf, b3Mul(m_scale, m_mesh->vertices[triangle->v1]));
	b3Vec3 v2 = b3Mul(xf, b3Mul(m_scale, m_mesh->vertices[triangle->v2]));
	b3Vec3 v3 = b3Mul(xf, b3Mul(m_scale, m_mesh->vertices[triangle->v3]));
	
	b3AABB aabb;
	aabb.lowerBound = b3Min(v1, b3Min(v2, v3));
	aabb.upperBound = b3Max(v1, b3Max(v2, v3));
	aabb.Extend(m_radius);
	
	*output = aabb;
}

bool b3MeshShape::TestSphere(const b3Sphere& sphere, const b3Transform& xf) const
{
	B3_NOT_USED(sphere);
	B3_NOT_USED(xf);
	return false;
}

bool b3MeshShape::RayCast(b3RayCastOutput* output, const b3RayCastInput& input, const b3Transform& xf, uint32 index) const
{
	b3TriangleShape triangle;
	GetChildTriangle(&triangle, index);
	return triangle.RayCast(output, input, xf);
}

struct b3MeshShapeRayCastCallback
{
	scalar Report(const b3RayCastInput& subInput, uint32 nodeId)
	{
		B3_NOT_USED(subInput);

		uint32 childIndex = mesh->m_mesh->tree.GetIndex(nodeId);
		
		b3RayCastOutput childOutput;
		if (mesh->RayCast(&childOutput, input, xf, childIndex))
		{
			// Track minimum time of impact to require less memory.
			if (childOutput.fraction < output.fraction)
			{
				hit = true;
				output = childOutput;
			}
		}
		
		return scalar(1);
	}

	b3RayCastInput input;
	const b3MeshShape* mesh;
	b3Transform xf;
	
	bool hit;
	b3RayCastOutput output;
};

bool b3MeshShape::RayCast(b3RayCastOutput* output, const b3RayCastInput& input, const b3Transform& xf) const 
{
	b3MeshShapeRayCastCallback callback;
	callback.input = input;
	callback.mesh = this;
	callback.xf = xf;
	callback.hit = false;
	callback.output.fraction = B3_MAX_SCALAR;
	
	B3_ASSERT(m_scale.x != scalar(0));
	B3_ASSERT(m_scale.y != scalar(0));
	B3_ASSERT(m_scale.z != scalar(0));

	b3Vec3 inv_scale;
	inv_scale.x = scalar(1) / m_scale.x;
	inv_scale.y = scalar(1) / m_scale.y;
	inv_scale.z = scalar(1) / m_scale.z;

	b3RayCastInput treeInput;
	treeInput.p1 = b3Mul(inv_scale, b3MulT(xf, input.p1));
	treeInput.p2 = b3Mul(inv_scale, b3MulT(xf, input.p2));
	treeInput.maxFraction = input.maxFraction;
	m_mesh->tree.RayCast(&callback, treeInput);

	output->fraction = callback.output.fraction;
	output->normal = callback.output.normal;

	return callback.hit;
}

void b3MeshShape::GetChildTriangle(b3TriangleShape* triangleShape, uint32 index) const
{
	B3_ASSERT(index < m_mesh->triangleCount);
	b3MeshTriangle* triangle = m_mesh->triangles + index;
	
	uint32 u1 = triangle->u1;
	uint32 u2 = triangle->u2;
	uint32 u3 = triangle->u3;

	b3Vec3 v1 = b3Mul(m_scale, m_mesh->vertices[triangle->v1]);
	b3Vec3 v2 = b3Mul(m_scale, m_mesh->vertices[triangle->v2]);
	b3Vec3 v3 = b3Mul(m_scale, m_mesh->vertices[triangle->v3]);

	triangleShape->m_vertex1 = v1;
	triangleShape->m_vertex2 = v2;
	triangleShape->m_vertex3 = v3;
	triangleShape->m_radius = m_radius;

	if (u1 != B3_NULL_VERTEX)
	{
		triangleShape->m_hasE1Vertex = true;
		triangleShape->m_e1Vertex = b3Mul(m_scale, m_mesh->vertices[u1]);
	}

	if (u2 != B3_NULL_VERTEX)
	{
		triangleShape->m_hasE2Vertex = true;
		triangleShape->m_e2Vertex = b3Mul(m_scale, m_mesh->vertices[u2]);
	}

	if (u3 != B3_NULL_VERTEX)
	{
		triangleShape->m_hasE3Vertex = true;
		triangleShape->m_e3Vertex = b3Mul(m_scale, m_mesh->vertices[u3]);
	}
}
