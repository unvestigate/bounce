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

#ifndef B3_CLIP_H
#define B3_CLIP_H

#include <bounce/common/template/array.h>
#include <bounce/collision/geometry/plane.h>

#define B3_NULL_EDGE B3_MAX_U32

// A combination of features used to uniquely identify a vertex on a feature.
struct b3FeaturePair
{
	uint32 inEdge1; // incoming side plane edge on hull 1
	uint32 outEdge1; // outgoing side plane edge on hull 1
	uint32 inEdge2; // incoming edge on hull 2 
	uint32 outEdge2; // outgoing edge on hull 2
};

inline b3FeaturePair b3MakePair(uint32 inEdge1, uint32 outEdge1, uint32 inEdge2, uint32 outEdge2)
{
	b3FeaturePair out;
	out.inEdge1 = inEdge1;
	out.outEdge1 = outEdge1;
	out.inEdge2 = inEdge2;
	out.outEdge2 = outEdge2;
	return out;
}

// A clip vertex.
struct b3ClipVertex
{
	b3Vec3 position;
	b3FeaturePair pair; // the features that built the clip point
};

// A clip polygon.
typedef b3Array<b3ClipVertex> b3ClipPolygon;

// A clip plane.
struct b3ClipPlane
{
	b3Plane plane;
	uint32 edge;
};

struct b3Hull;
struct b3Capsule;

// Build a clip segment for a segment.
void b3BuildSegment(b3ClipVertex vOut[2],
	const b3Capsule* segment);

// Build a clip polygon given an index to the polygon face.
void b3BuildPolygon(b3ClipPolygon& pOut,
	const b3Transform& xf, uint32 index, const b3Hull* hull);

// Clip a segment by a plane. 
// Output a segment whose points are behind or on the input plane.
// Return the number of output points.
uint32 b3ClipSegmentToPlane(b3ClipVertex vOut[2],
	const b3ClipVertex vIn[2], const b3ClipPlane& plane);

// Clip a polygon by a plane.  
// Output a polygon whose points are behind or on the input plane.
void b3ClipPolygonToPlane(b3ClipPolygon& pOut,
	const b3ClipPolygon& pIn, const b3ClipPlane& plane);

// Clip a segment by a segment face side planes.
// Return the number of output points.
uint32 b3ClipSegmentToSegmentSidePlanes(b3ClipVertex vOut[2],
	const b3ClipVertex vIn[2], const b3Capsule* segment);

// Clip a segment by a hull face side planes.
// Return the number of output points.
uint32 b3ClipSegmentToFaceSidePlanes(b3ClipVertex vOut[2],
	const b3ClipVertex vIn[2], const b3Transform& xf, scalar radius, uint32 index, const b3Hull* hull);

// Clip a polygon by a hull face side planes.
void b3ClipPolygonToFaceSidePlanes(b3ClipPolygon& pOut,
	const b3ClipPolygon& pIn, const b3Transform& xf, scalar radius, uint32 index, const b3Hull* hull);

#endif