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

#ifndef BODY_DRAGGER_H
#define BODY_DRAGGER_H

#include <bounce/common/math/vec3.h>

struct b3Ray;
class b3World;
class b3Fixture;
class b3MouseJoint;

// A body dragger.
class BodyDragger
{
public:
	BodyDragger(b3Ray* ray, b3World* world);
	
	bool StartDragging();

	void Drag();

	void StopDragging();

	bool IsDragging() const;

	b3Ray* GetRay() const;

	b3Fixture* GetFixture() const;

	b3MouseJoint* GetMouseJoint();

	b3Vec3 GetPointA() const;

	b3Vec3 GetPointB() const;
private:
	b3Ray * m_ray;
	scalar m_fraction;
	b3World* m_world;
	b3Fixture* m_fixture;
	b3Vec3 m_localPoint;
	b3MouseJoint* m_mouseJoint;
};

inline bool BodyDragger::IsDragging() const
{
	return m_fixture != nullptr;
}

inline b3Ray* BodyDragger::GetRay() const
{
	return m_ray;
}

inline b3MouseJoint* BodyDragger::GetMouseJoint()
{
	return m_mouseJoint;
}

#endif