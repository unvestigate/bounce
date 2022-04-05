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

#ifndef B3_MESH_CONTACT_H
#define B3_MESH_CONTACT_H

#include <bounce/dynamics/contacts/contact.h>
#include <bounce/collision/geometry/aabb.h>

class b3MeshContact : public b3Contact
{
public:
	b3MeshContact(b3Fixture* fixtureA, b3Fixture* fixtureB, bool meshIsA = true);
	~b3MeshContact();

	bool TestOverlap() override;

	void SynchronizeFixture() override;

	void FindPairs() override;

	void Collide() override;

	virtual void Evaluate(b3Manifold& manifold, const b3Transform& xfA, const b3Transform& xfB, uint32 cacheIndex) = 0;

	bool MoveAABB(const b3AABB& aabb, const b3Vec3& displacement);

	// Static tree callback. There is no midphase. 
	bool Report(uint32 nodeId);

	// Is the mesh shape the shape A?
	bool m_meshIsA;

	// Did the other AABB move significantly?
	bool m_aabbMoved;

	// The other AABB relative to other shape origin.
	b3AABB m_aabb; 
	
	// Triangles potentially overlapping with the other shape.
	uint32 m_triangleCapacity;
	uint32* m_triangles;
	uint32 m_triangleCount;

	// Contact manifolds.
	b3Manifold m_clusterManifolds[B3_MAX_MANIFOLDS];
};

#endif