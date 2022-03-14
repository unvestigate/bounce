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

#include <bounce/collision/collide/clip.h>
#include <bounce/collision/geometry/capsule.h>
#include <bounce/collision/geometry/hull.h>

void b3BuildSegment(b3ClipVertex vOut[2],
	const b3Capsule* segment)
{
	vOut[0].position = segment->vertex1;
	vOut[0].pair = b3MakePair(B3_NULL_EDGE, B3_NULL_EDGE, 0, 0);
	
	vOut[1].position = segment->vertex2;
	vOut[1].pair = b3MakePair(B3_NULL_EDGE, B3_NULL_EDGE, 1, 0);
}

void b3BuildPolygon(b3ClipPolygon& pOut,
	const b3Transform& xf, uint32 index, const b3Hull* hull)
{
	B3_ASSERT(pOut.Count() == 0);

	const b3Face* face = hull->GetFace(index);
	const b3HalfEdge* begin = hull->GetEdge(face->edge);
	const b3HalfEdge* edge = begin;
	do
	{
		const b3HalfEdge* twin = hull->GetEdge(edge->twin);		
		
		uint32 inEdge = edge->prev;
		uint32 outEdge = twin->twin;

		b3ClipVertex clipVertex;
		clipVertex.position = b3Mul(xf, hull->GetVertex(edge->origin));
		clipVertex.pair = b3MakePair(B3_NULL_EDGE, B3_NULL_EDGE, inEdge, outEdge);

		pOut.PushBack(clipVertex);

		edge = hull->GetEdge(edge->next);
	} while (edge != begin);

	B3_ASSERT(pOut.Count() > 2);
}

// Sutherland-Hodgman clipping.
uint32 b3ClipSegmentToPlane(b3ClipVertex vOut[2],
	const b3ClipVertex vIn[2], const b3ClipPlane& plane)
{
	uint32 numOut = 0;

	scalar distance1 = b3Distance(vIn[0].position, plane.plane);
	scalar distance2 = b3Distance(vIn[1].position, plane.plane);

	// If the points are behind the plane keep them
	if (distance1 <= scalar(0))
	{
		vOut[numOut++] = vIn[0];
	}

	if (distance2 <= scalar(0))
	{
		vOut[numOut++] = vIn[1];
	}

	// If the points are on opposite sides keep intersection
	if (distance1 <= scalar(0) && distance2 > scalar(0))
	{
		scalar fraction = distance1 / (distance1 - distance2);
		vOut[numOut].position = vIn[0].position + fraction * (vIn[1].position - vIn[0].position);
		vOut[numOut].pair = b3MakePair(plane.edge, B3_NULL_EDGE, vIn[0].pair.outEdge2, B3_NULL_EDGE);
		++numOut;
	}
	else if (distance1 > scalar(0) && distance2 <= scalar(0))
	{
		scalar fraction = distance1 / (distance1 - distance2);
		vOut[numOut].position = vIn[0].position + fraction * (vIn[1].position - vIn[0].position);
		vOut[numOut].pair = b3MakePair(B3_NULL_EDGE, plane.edge, B3_NULL_EDGE, vIn[1].pair.inEdge2);
		++numOut;
	}

	return numOut;
}

// Sutherland-Hodgman clipping.
void b3ClipPolygonToPlane(b3ClipPolygon& pOut,
	const b3ClipPolygon& pIn, const b3ClipPlane& plane)
{
	B3_ASSERT(pIn.Count() > 0);
	B3_ASSERT(pOut.Count() == 0);

	b3ClipVertex v1 = pIn.Back();
	scalar distance1 = b3Distance(v1.position, plane.plane);

	for (uint32 i = 0; i < pIn.Count(); ++i)
	{
		b3ClipVertex v2 = pIn[i];
		scalar distance2 = b3Distance(v2.position, plane.plane);

		if (distance1 <= scalar(0) && distance2 <= scalar(0))
		{
			// Both vertices are behind or lying on the plane.
			// Keep v2
			pOut.PushBack(v2);
		}
		else if (distance1 <= scalar(0) && distance2 > scalar(0))
		{
			// v1 is behind and v2 in front
			// Keep intersection
			scalar fraction = distance1 / (distance1 - distance2);

			b3ClipVertex vertex;
			vertex.position = v1.position + fraction * (v2.position - v1.position);
			vertex.pair = b3MakePair(plane.edge, B3_NULL_EDGE, v1.pair.outEdge2, B3_NULL_EDGE);

			pOut.PushBack(vertex);
		}
		else if (distance1 > scalar(0) && distance2 <= scalar(0))
		{
			// v2 is behind and v1 in front
			// Keep intersection and v2
			scalar fraction = distance1 / (distance1 - distance2);

			b3ClipVertex vertex;
			vertex.position = v1.position + fraction * (v2.position - v1.position);
			vertex.pair = b3MakePair(B3_NULL_EDGE, plane.edge, B3_NULL_EDGE, v2.pair.inEdge2);

			pOut.PushBack(vertex);
			pOut.PushBack(v2);
		}

		// Make v2 as the starting vertex of the next edge
		v1 = v2;
		distance1 = distance2;
	}
}

// Clip a segment to segment face side planes.
uint32 b3ClipSegmentToFaceSidePlanes(b3ClipVertex vOut[2],
	const b3ClipVertex vIn[2], const b3Capsule* segment)
{
	// Start from somewhere.
	vOut[0] = vIn[0];
	vOut[1] = vIn[1];
	uint32 numOut = 0;

	b3Vec3 P1 = segment->vertex1;
	b3Vec3 Q1 = segment->vertex2;
	b3Vec3 E1 = Q1 - P1;

	B3_ASSERT(b3Dot(E1, E1) > B3_EPSILON * B3_EPSILON);
	b3ClipPlane clipPlane1;
	clipPlane1.plane.normal = b3Normalize(E1);
	clipPlane1.plane.offset = b3Dot(clipPlane1.plane.normal, Q1);
	clipPlane1.edge = 0;

	b3ClipVertex clipSegment1[2];
	numOut = b3ClipSegmentToPlane(clipSegment1, vOut, clipPlane1);

	vOut[0] = clipSegment1[0];
	vOut[1] = clipSegment1[1];

	if (numOut < 2)
	{
		return numOut;
	}

	b3ClipPlane clipPlane2;
	clipPlane2.plane.normal = -clipPlane1.plane.normal;
	clipPlane2.plane.offset = b3Dot(clipPlane2.plane.normal, P1);
	clipPlane2.edge = 1;

	b3ClipVertex clipSegment2[2];
	numOut = b3ClipSegmentToPlane(clipSegment2, vOut, clipPlane2);

	vOut[0] = clipSegment2[0];
	vOut[1] = clipSegment2[1];

	if (numOut < 2)
	{
		return numOut;
	}

	return numOut;
}

// Clip a segment to a face side planes.
uint32 b3ClipSegmentToFaceSidePlanes(b3ClipVertex vOut[2],
	const b3ClipVertex vIn[2], const b3Transform& xf, scalar r, uint32 index, const b3Hull* hull)
{
	// Start from somewhere.
	vOut[0] = vIn[0];
	vOut[1] = vIn[1];
	uint32 numOut = 0;

	const b3Face* face = hull->GetFace(index);
	const b3HalfEdge* begin = hull->GetEdge(face->edge);
	const b3HalfEdge* edge = begin;
	do
	{
		const b3HalfEdge* twin = hull->GetEdge(edge->twin);
		uint32 edgeId = uint32(twin->twin);

		b3Plane plane = hull->GetEdgeSidePlane(edgeId);
		plane.offset += r;

		b3ClipPlane clipPlane;
		clipPlane.edge = edgeId;
		clipPlane.plane = b3Mul(xf, plane);

		b3ClipVertex clipSegment[2];
		numOut = b3ClipSegmentToPlane(clipSegment, vOut, clipPlane);

		vOut[0] = clipSegment[0];
		vOut[1] = clipSegment[1];

		if (numOut == 0)
		{
			return numOut;
		}

		edge = hull->GetEdge(edge->next);
	} while (edge != begin);

	// Now vOut contains the clipped points.
	return numOut;
}

// Clip a polygon to face side planes.
void b3ClipPolygonToFaceSidePlanes(b3ClipPolygon& pOut,
	const b3ClipPolygon& pIn, const b3Transform& xf, scalar r, uint32 index, const b3Hull* hull)
{
	B3_ASSERT(pIn.Count() > 0);
	B3_ASSERT(pOut.Count() == 0);

	// Start from somewhere.
	pOut = pIn;

	const b3Face* face = hull->GetFace(index);
	const b3HalfEdge* begin = hull->GetEdge(face->edge);
	const b3HalfEdge* edge = begin;
	do
	{
		const b3HalfEdge* twin = hull->GetEdge(edge->twin);
		uint32 edgeId = uint32(twin->twin);

		b3Plane plane = hull->GetEdgeSidePlane(edgeId);
		plane.offset += r;

		b3ClipPlane clipPlane;
		clipPlane.edge = edgeId;
		clipPlane.plane = b3Mul(xf, plane);

		b3StackArray<b3ClipVertex, 32> clipPolygon;
		b3ClipPolygonToPlane(clipPolygon, pOut, clipPlane);
		pOut = clipPolygon;

		if (pOut.IsEmpty())
		{
			return;
		}

		edge = hull->GetEdge(edge->next);
	} while (edge != begin);

	// Now pOut contains the clipped points.
}