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

#ifndef B3_HULL_H
#define B3_HULL_H

#include <bounce/collision/geometry/plane.h>

struct b3Face
{
	uint32 edge;
};

struct b3HalfEdge
{
	uint32 origin;
	uint32 twin;
	uint32 face;
	uint32 prev;
	uint32 next;
};

struct b3Hull
{
	b3Vec3 centroid;
	uint32 vertexCount;
	b3Vec3* vertices;
	uint32 edgeCount;
	b3HalfEdge* edges;
	uint32 faceCount;
	b3Face* faces;
	b3Plane* planes;
	
	const b3Vec3& GetVertex(uint32 index) const;
	const b3HalfEdge* GetEdge(uint32 index) const;
	const b3Face* GetFace(uint32 index) const;
	const b3Plane& GetPlane(uint32 index) const;

	uint32 GetSupportVertex(const b3Vec3& direction) const;
	//uint32 GetSupportEdge(const b3Vec3& direction) const;
	uint32 GetSupportFace(const b3Vec3& direction) const;
	
	b3Plane GetEdgeSidePlane(uint32 index) const;
	
	uint32 GetSize() const;

	void Validate() const;
	void Validate(const b3Face* face) const;
	void Validate(const b3HalfEdge* edge) const;

	void Dump() const;

	void Scale(const b3Vec3& scale);
	void Rotate(const b3Quat& rotation);
	void Translate(const b3Vec3& translation);

	// Scale -> Rotate -> Translate
	void Transform(const b3Transform& xf, const b3Vec3& scale);
};

inline b3HalfEdge b3MakeEdge(uint32 origin, uint32 twin, uint32 face, uint32 prev, uint32 next)
{
	b3HalfEdge edge;
	edge.origin = origin;
	edge.twin = twin;
	edge.face = face;
	edge.prev = prev;
	edge.next = next;
	return edge;
}

inline const b3Vec3& b3Hull::GetVertex(uint32 index) const
{
	return vertices[index];
}

inline const b3HalfEdge* b3Hull::GetEdge(uint32 index) const
{
	return edges + index;
}

inline const b3Face* b3Hull::GetFace(uint32 index) const
{
	return faces + index;
}

inline const b3Plane& b3Hull::GetPlane(uint32 index) const
{
	return planes[index];
}

inline uint32 b3Hull::GetSupportVertex(const b3Vec3& direction) const
{
	uint32 maxIndex = 0;
	scalar maxProjection = b3Dot(direction, vertices[maxIndex]);
	for (uint32 i = 1; i < vertexCount; ++i)
	{
		scalar projection = b3Dot(direction, vertices[i]);
		if (projection > maxProjection)
		{
			maxIndex = i;
			maxProjection = projection;
		}
	}
	return maxIndex;
}

inline uint32 b3Hull::GetSupportFace(const b3Vec3& direction) const
{
	uint32 maxIndex = 0;
	scalar maxProjection = b3Dot(direction, planes[maxIndex].normal);
	for (uint32 i = 1; i < faceCount; ++i)
	{
		scalar projection = b3Dot(direction, planes[i].normal);
		if (projection > maxProjection)
		{
			maxIndex = i;
			maxProjection = projection;
		}
	}
	return maxIndex;
}

inline b3Plane b3Hull::GetEdgeSidePlane(uint32 index) const
{
	const b3HalfEdge* edge = edges + index;
	const b3HalfEdge* twin = edges + edge->twin;
	const b3Plane* facePlane = planes + edge->face;

	b3Vec3 P = vertices[edge->origin];
	b3Vec3 Q = vertices[twin->origin];
	b3Vec3 E = Q - P;
	b3Vec3 D = b3Cross(E, facePlane->normal);

	b3Plane plane;
	plane.normal = b3Normalize(D);
	plane.offset = b3Dot(plane.normal, P);
	return plane;
}

inline uint32 b3Hull::GetSize() const
{
	uint32 size = 0;
	size += sizeof(b3Hull);
	size += vertexCount * sizeof(b3Vec3);
	size += edgeCount * sizeof(b3HalfEdge);
	size += faceCount * sizeof(b3Face);
	size += faceCount * sizeof(b3Plane);
	return size;
}

#endif