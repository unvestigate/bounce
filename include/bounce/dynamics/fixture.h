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

#ifndef B3_FIXTURE_H
#define B3_FIXTURE_H

#include <bounce/common/graphics/color.h>
#include <bounce/collision/shapes/shape.h>
#include <bounce/dynamics/body.h>

struct b3ContactEdge;

class b3Body;
class b3Shape;
class b3BlockAllocator;
class b3BroadPhase;
class b3Draw;

// Fixture definition. This requires providing a shape pointer that will 
// be cloned and some physics-related parameters. 
// As the shape will be cloned internally you can create shapes in the stack.
struct b3FixtureDef 
{
	// Default constructor fills in some default parameters.
	b3FixtureDef()
	{
		shape = nullptr;
		userData = nullptr;
		isSensor = false;
		density = scalar(0);
		friction = scalar(0.3);
		restitution = scalar(0);
	}

	// The shape pointer. The shape will be cloned.
	// Therefore, you can create shapes in the stack and they can be temporary.
	const b3Shape* shape;
	
	// User data. It typically stores a pointer to a model mesh or a game entity.
	void* userData;
	
	// Is the shape a sensor? 
	// Sensor shapes aren't able to respond to collisions.
	// However, sensor shapes are reported to the contact callbacks.
	bool isSensor;
	
	// Density in kg/m^3 (kilogram per cubic metre).
	scalar density;
	
	// Coefficient of restitution in the range [0, 1].
	scalar restitution;
	
	// Coefficient of friction in the range [0, 1].
	scalar friction;
};

// This class extends a collision shape by containing simulation-related parameters 
// such as density, coefficients of friction and restitution.
class b3Fixture
{
public:
	// Get the shape type.
	b3Shape::Type GetType() const;

	// Get the child shape.
	// Manipulating the shape may lead to physics undefined behaviour.
	b3Shape* GetShape();
	const b3Shape* GetShape() const;

	// Get the parent body of this shape.
	const b3Body* GetBody() const;
	b3Body* GetBody();

	// Calculate the mass data for this shape given the shape density.
	void ComputeMass(b3MassData* data) const;

	// Compute the shape world AABB.
	void ComputeAABB(b3AABB* aabb) const;

	// Test if a sphere is contained inside this shape.
	bool TestSphere(const b3Sphere& sphere) const;

	// Compute the ray intersection point, normal of surface, and fraction.
	bool RayCast(b3RayCastOutput* output, const b3RayCastInput& input) const;
	
	// Set if this shape is a sensor.
	void SetSensor(bool bit);

	// Is this shape a sensor?
	bool IsSensor() const;

	// Get the shape density.
	scalar GetDensity() const;

	// Set the shape density.
	void SetDensity(scalar density);

	// Get the shape coefficient of restitution.
	scalar GetRestitution() const;

	// Set the shape coefficient of restitution.
	// This is a value in the range [0, 1].
	void SetRestitution(scalar restitution);

	// Get the shape coefficient of friction.
	scalar GetFriction() const;
	
	// Set the shape coefficient of friction.
	// This is a value in the range [0, 1].
	void SetFriction(scalar friction);

	// Get the user data associated with this shape.
	void* GetUserData() const;

	// Set the user data associated with this shape.
	void SetUserData(void* data);

	// Get broadphase AABB.
	const b3AABB& GetFatAABB() const;

	// Get the list of contacts that contains this body.
	const b3ContactEdge* GetContactList() const;
	b3ContactEdge* GetContactList();

	// Dump this shape to the log file.
	void Dump(u32 bodyIndex) const;

	// Draw this shape geometry.
	void Draw(b3Draw* draw, const b3Color& color);
	
	// Draw this shape in solid mode.
	void DrawSolid(b3Draw* draw, const b3Color& color);
	
	// Get the next fixture in the body fixture list.
	const b3Fixture* GetNext() const;
	b3Fixture* GetNext();
protected:
	friend class b3World;
	friend class b3Body;
	friend class b3Contact;
	friend class b3ContactManager;
	friend class b3MeshContact;
	friend class b3ContactSolver;
	
	b3Fixture();

	// We need separation create/destroy functions from the constructor/destructor because
	// the destructor cannot access the allocator (no destructor arguments allowed by C++).
	void Create(b3BlockAllocator* allocator, b3Body* body, const b3FixtureDef* def);
	void Destroy(b3BlockAllocator* allocator);

	// Synchronize proxy AABB.
	void Synchronize(b3BroadPhase* broadPhase, const b3Transform& transform1, const b3Transform& transform2);

	b3Shape* m_shape;

	scalar m_density;
	
	scalar m_restitution;
	scalar m_friction;

	u32 m_proxyId;

	bool m_isSensor;
	void* m_userData;

	// Contact edges for this fixture contact graph.
	b3ContactEdge* m_contactList;
	
	// The parent body of this fixture.
	b3Body* m_body;
	
	// Links to the body fixture list.
	b3Fixture* m_next;
};

inline b3Shape::Type b3Fixture::GetType() const 
{ 
	return m_shape->GetType(); 
}

inline b3Shape* b3Fixture::GetShape()
{
	return m_shape;
}

inline const b3Shape* b3Fixture::GetShape() const
{
	return m_shape;
}

inline scalar b3Fixture::GetDensity() const 
{ 
	return m_density; 
}

inline void b3Fixture::SetDensity(scalar density)
{ 
	m_density = density; 
}

inline scalar b3Fixture::GetRestitution() const
{ 
	return m_restitution; 
}

inline void b3Fixture::SetRestitution(scalar restitution)
{ 
	m_restitution = restitution; 
}

inline scalar b3Fixture::GetFriction() const
{
	return m_friction;
}

inline void b3Fixture::SetFriction(scalar friction)
{
	m_friction = friction;
}

inline bool b3Fixture::IsSensor() const
{
	return m_isSensor;
}

inline void* b3Fixture::GetUserData() const
{ 
	return m_userData; 
}

inline void b3Fixture::SetUserData(void* data)
{ 
	m_userData = data; 
}

inline const b3Body* b3Fixture::GetBody() const
{
	return m_body;
}

inline b3Body* b3Fixture::GetBody()
{
	return m_body;
}

inline const b3ContactEdge* b3Fixture::GetContactList() const
{
	return m_contactList;
}

inline b3ContactEdge* b3Fixture::GetContactList()
{
	return m_contactList;
}

inline const b3Fixture* b3Fixture::GetNext() const
{
	return m_next;
}

inline b3Fixture* b3Fixture::GetNext()
{
	return m_next;
}

inline void b3Fixture::ComputeAABB(b3AABB* aabb) const
{
	m_shape->ComputeAABB(aabb, m_body->GetTransform());
}

inline bool b3Fixture::TestSphere(const b3Sphere& sphere) const
{
	return m_shape->TestSphere(sphere, m_body->GetTransform());
}

inline bool b3Fixture::RayCast(b3RayCastOutput* output, const b3RayCastInput& input) const
{
	return m_shape->RayCast(output, input, m_body->GetTransform());
}

inline void b3Fixture::ComputeMass(b3MassData* massData) const
{
	m_shape->ComputeMass(massData, m_density);
}

inline void b3Fixture::Draw(b3Draw* draw, const b3Color& color)
{
	m_shape->Draw(draw, m_body->GetTransform(), color);
}

inline void b3Fixture::DrawSolid(b3Draw* draw, const b3Color& color)
{
	m_shape->DrawSolid(draw, m_body->GetTransform(), color);
}

#endif
