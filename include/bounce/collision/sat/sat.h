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

#ifndef B3_SAT_H
#define B3_SAT_H

#include <bounce/common/math/transform.h>

struct b3Hull;

///////////////////////////////////////////////////////////////////////////////////////////////////

struct b3FaceQuery
{
	uint32 index;
	scalar separation;
};

b3FaceQuery b3QueryFaceSeparation(const b3Transform& xf1, const b3Hull* hull1,
	const b3Transform& xf2, const b3Hull* hull2);

///////////////////////////////////////////////////////////////////////////////////////////////////

struct b3EdgeQuery
{
	uint32 index1;
	uint32 index2;
	scalar separation;
};

b3EdgeQuery b3QueryEdgeSeparation(const b3Transform& xf1, const b3Hull* hull1,
	const b3Transform& xf2, const b3Hull* hull2);

///////////////////////////////////////////////////////////////////////////////////////////////////

enum b3CacheType
{
	e_empty,
	e_separation,
	e_overlap,
};

enum b3FeatureType
{
	e_edges, // an edge on hull 1 and an edge on hull 2
	e_face1, // a face on hull 1 and a vertex/edge/face on hull 2
	e_face2, // a face on hull 2 and a vertex/edge/face on hull 1
};

// A cached feature pair is used to improve the performance 
// of the SAT when called more than once. 
struct b3FeatureCache
{
	b3FeatureCache() : cacheType(e_empty) { } 

	// Read the current state of the cache.
	// Return b3CacheType::e_empty if neither a separation or penetration was detected.
	b3CacheType ReadState(const b3Transform& xf1, const b3Hull* hull1,
		const b3Transform& xf2, const b3Hull* hull2, scalar totalRadius);

	b3CacheType ReadEdges(const b3Transform& xf1, const b3Hull* hull1,
		const b3Transform& xf2, const b3Hull* hull2, scalar totalRadius);

	b3CacheType ReadFace(const b3Transform& xf1, const b3Hull* hull1,
		const b3Transform& xf2, const b3Hull* hull2, scalar totalRadius);

	b3CacheType cacheType; // sat result
	b3FeatureType featureType; // feature type
	uint32 index1; // feature index on hull 1
	uint32 index2; // feature index on hull 2
};

inline b3FeatureCache b3MakeCache(b3CacheType cacheType, b3FeatureType featureType, uint32 index1, uint32 index2)
{
	b3FeatureCache cache;
	cache.cacheType = cacheType;
	cache.featureType = featureType;
	cache.index1 = index1;
	cache.index2 = index2;
	return cache;
}

#endif
