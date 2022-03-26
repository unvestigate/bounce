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

#ifndef B3_GJK_PROXY_H
#define B3_GJK_PROXY_H

#include <bounce/common/math/vec3.h>

class b3Shape;

// A GJK proxy encapsulates any convex hull to be used by the GJK algorithm.
struct b3GJKProxy
{
	b3GJKProxy() : m_vertices(nullptr), m_count(0), m_radius(scalar(0)) { }
	
	// Construct this proxy from shape. The shape
	// must remain in scope while the proxy is in use.
	b3GJKProxy(const b3Shape* shape, uint32 index) { Set(shape, index); }
	
	// Initialize the proxy using the given shape. The shape
	// must remain in scope while the proxy is in use.
	void Set(const b3Shape* shape, uint32 index);

	// Initialize the proxy using a vertex cloud and radius. The vertices
	// must remain in scope while the proxy is in use.
	void Set(const b3Vec3* vertices, uint32 count, scalar radius);

	// Get the number of vertices in this proxy.
	uint32 GetVertexCount() const;

	// Read an indexed vertex from this proxy.
	const b3Vec3& GetVertex(uint32 index) const;

	// Get the support vertex index in a given direction.
	uint32 GetSupportIndex(const b3Vec3& direction) const;

	// Get the support vertex in a given direction.
	const b3Vec3& GetSupportVertex(const b3Vec3& direction) const;

	b3Vec3 m_buffer[3]; 
	const b3Vec3* m_vertices; 
	uint32 m_count; 
	scalar m_radius; 
};

inline uint32 b3GJKProxy::GetVertexCount() const
{
	return m_count;
}

inline const b3Vec3& b3GJKProxy::GetVertex(uint32 index) const
{
	B3_ASSERT(index < m_count);
	return m_vertices[index];
}

inline uint32 b3GJKProxy::GetSupportIndex(const b3Vec3& d) const
{
	uint32 maxIndex = 0;
	scalar maxProjection = b3Dot(d, m_vertices[maxIndex]);
	for (uint32 i = 1; i < m_count; ++i)
	{
		scalar projection = b3Dot(d, m_vertices[i]);
		if (projection > maxProjection)
		{
			maxIndex = i;
			maxProjection = projection;
		}
	}
	return maxIndex;
}

inline const b3Vec3& b3GJKProxy::GetSupportVertex(const b3Vec3& d) const
{
	uint32 index = GetSupportIndex(d);
	return m_vertices[index];
}

#endif