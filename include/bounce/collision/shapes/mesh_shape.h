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

#ifndef B3_MESH_SHAPE_H
#define B3_MESH_SHAPE_H

#include <bounce/collision/shapes/shape.h>

struct b3Mesh;
class b3TriangleShape;

// A triangle mesh shape.
class b3MeshShape : public b3Shape 
{
public:
	b3MeshShape();
	
	b3Shape* Clone(b3BlockAllocator* allocator) const;

	void ComputeMass(b3MassData* data, scalar density) const;

	void ComputeAABB(b3AABB* aabb, const b3Transform& xf) const;

	void ComputeAABB(b3AABB* aabb, const b3Transform& xf, uint32 childIndex) const;

	bool TestSphere(const b3Sphere& sphere, const b3Transform& xf) const;

	bool RayCast(b3RayCastOutput* output, const b3RayCastInput& input, const b3Transform& xf) const;

	bool RayCast(b3RayCastOutput* output, const b3RayCastInput& input, const b3Transform& xf, uint32 childIndex) const;

	void GetChildTriangle(b3TriangleShape* triangle, uint32 childIndex) const;
	
	const b3Mesh* m_mesh;

	// The mesh shape scale. It can be non-uniform and negative.
	b3Vec3 m_scale;
};

#endif