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

#ifndef B3_COLLISION_H
#define B3_COLLISION_H

#include <bounce/common/math/transform.h>
#include <bounce/common/math/vec2.h>
#include <bounce/collision/clip.h>

#define B3_NULL_TRIANGLE B3_MAX_U32

class b3Shape;
class b3SphereShape;
class b3CapsuleShape;
class b3TriangleShape;
class b3HullShape;

struct b3FeatureCache;

// Input for a ray cast.
struct b3RayCastInput
{
	b3Vec3 p1; // first point on segment
	b3Vec3 p2; // second point on segment
	scalar maxFraction; // maximum intersection
};

// Output of a ray cast.
struct b3RayCastOutput
{
	scalar fraction; // time of intersection on ray-segment
	b3Vec3 normal; // surface normal of intersection
};

// A 128-bit key.
struct b3Key128
{
	uint64 key1;
	uint64 key2;
};

// A 128-bit pair key.
union b3PairKey
{
	b3FeaturePair pair;
	b3Key128 key;
};

// A contact manifold point identifier.
struct b3ContactID
{
	bool operator==(const b3ContactID& other) const
	{
		bool tk = triangleKey == other.triangleKey;
		bool k1 = key.key.key1 == other.key.key.key1;
		bool k2 = key.key.key2 == other.key.key.key2;
		return tk && k1 && k2;
	}

	uint32 triangleKey;
	b3PairKey key;
};

inline b3ContactID b3MakeID(const b3FeaturePair& pair)
{
	b3ContactID id;
	id.triangleKey = B3_NULL_TRIANGLE;
	id.key.pair = pair;
	return id;
}

inline b3ContactID b3MakeID(uint64 key1, uint64 key2)
{
	b3ContactID id;
	id.triangleKey = B3_NULL_TRIANGLE;
	id.key.key.key1 = key1;
	id.key.key.key2 = key2;
	return id;
}

// A contact manifold point.
struct b3ManifoldPoint
{
	b3Vec3 localNormal1; // local normal on the first shape 
	b3Vec3 localPoint1; // local point on the first shape without its radius
	b3Vec3 localPoint2; // local point on the other shape without its radius
	scalar normalImpulse; // the non-penetration impulse
	b3ContactID id; // uniquely identifies a contact point between two shapes
	bool persisting; // is the point persisting over the time steps
	bool edgeContact; // edge contact flag - for internal use
};

// A contact manifold is a group of contact points with similar contact normal.
struct b3Manifold
{
	// Clear the manifold.
	// Initialize impulses arbitrarily for warm starting.
	void Initialize();

	// Initialize impulses for warm starting from the old manifold.
	void Initialize(const b3Manifold& old);

	b3ManifoldPoint points[B3_MAX_MANIFOLD_POINTS]; // manifold points
	uint32 pointCount; // number of manifold points

	b3Vec2 tangentImpulse; // the central friction impulses
	scalar motorImpulse; // the central normal impulse

	scalar motorSpeed; // target angular speed along the normal
	scalar tangentSpeed1; // target speed along the first tangent
	scalar tangentSpeed2; // target speed along the second tangent
};

struct b3WorldManifoldPoint
{
	void Initialize(const b3ManifoldPoint* p, scalar rA, const b3Transform& xfA, scalar rB, const b3Transform& xfB);

	b3Vec3 point;
	b3Vec3 normal;
	scalar separation;
};

struct b3WorldManifold
{
	void Initialize(const b3Manifold* m, scalar rA, const b3Transform& xfA, scalar rB, const b3Transform& xfB);

	uint32 pointCount;
	b3WorldManifoldPoint points[B3_MAX_MANIFOLD_POINTS];

	b3Vec3 center;
	b3Vec3 normal;
	b3Vec3 tangent1;
	b3Vec3 tangent2;
};

// Compute a manifold for two spheres.
void b3CollideSpheres(b3Manifold& manifold,
	const b3Transform& xf1, const b3SphereShape* sphere1,
	const b3Transform& xf2, const b3SphereShape* sphere2);

// Compute a manifold for a capsule and a sphere.
void b3CollideCapsuleAndSphere(b3Manifold& manifold,
	const b3Transform& xf1, const b3CapsuleShape* capsule1,
	const b3Transform& xf2, const b3SphereShape* sphere2);

// Compute a manifold for two capsules.
void b3CollideCapsules(b3Manifold& manifold,
	const b3Transform& xf1, const b3CapsuleShape* capsule1,
	const b3Transform& xf2, const b3CapsuleShape* capsule2);

// Compute a manifold for a triangle and a sphere.
void b3CollideTriangleAndSphere(b3Manifold& manifold,
	const b3Transform& xf1, const b3TriangleShape* triangle1,
	const b3Transform& xf2, const b3SphereShape* sphere2);

// Compute a manifold for a triangle and a capsule.
void b3CollideTriangleAndCapsule(b3Manifold& manifold,
	const b3Transform& xf1, const b3TriangleShape* triangle1,
	const b3Transform& xf2, const b3CapsuleShape* capsule2);

// Compute a manifold for a hull and a sphere.
void b3CollideHullAndSphere(b3Manifold& manifold,
	const b3Transform& xf1, const b3HullShape* hull1,
	const b3Transform& xf2, const b3SphereShape* sphere2);

// Compute a manifold for a hull and a capsule.
void b3CollideHullAndCapsule(b3Manifold& manifold,
	const b3Transform& xf1, const b3HullShape* hull1,
	const b3Transform& xf2, const b3CapsuleShape* capsule2);

// Compute a manifold for a triangle and a hull.
void b3CollideHullAndTriangle(b3Manifold& manifold,
	const b3Transform& xf1, const b3HullShape* hull1,
	const b3Transform& xf2, const b3TriangleShape* triangle2);

// Compute a manifold for two hulls. 
void b3CollideHulls(b3Manifold& manifold,
	const b3Transform& xf1, const b3HullShape* hull1,
	const b3Transform& xf2, const b3HullShape* hull2);

// Compute a manifold for two hulls. 
void b3CollideHulls(b3Manifold& manifold,
	const b3Transform& xf1, const b3HullShape* hull1,
	const b3Transform& xf2, const b3HullShape* hull2,
	b3FeatureCache& cache,
	const b3Transform& xf01, const b3Transform& xf02);

// Test if two generic shapes are overlapping.
bool b3TestOverlap(const b3Transform& xf1, uint32 index1, const b3Shape* shape1,
	const b3Transform& xf2, uint32 index2, const b3Shape* shape2);

#endif