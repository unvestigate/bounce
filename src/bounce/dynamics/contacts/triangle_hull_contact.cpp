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

#include <bounce/dynamics/contacts/triangle_hull_contact.h>
#include <bounce/collision/shapes/triangle_shape.h>
#include <bounce/collision/shapes/hull_shape.h>
#include <bounce/common/memory/block_allocator.h>

b3Contact* b3TriangleAndHullContact::Create(b3Fixture* fixtureA, b3Fixture* fixtureB, b3BlockAllocator* allocator)
{
	void* mem = allocator->Allocate(sizeof(b3TriangleAndHullContact));
	return new (mem) b3TriangleAndHullContact(fixtureA, fixtureB);
}

void b3TriangleAndHullContact::Destroy(b3Contact* contact, b3BlockAllocator* allocator)
{
	((b3TriangleAndHullContact*)contact)->~b3TriangleAndHullContact();
	allocator->Free(contact, sizeof(b3TriangleAndHullContact));
}

b3TriangleAndHullContact::b3TriangleAndHullContact(b3Fixture* fixtureA, b3Fixture* fixtureB) : b3ConvexContact(fixtureA, fixtureB)
{
	B3_ASSERT(fixtureA->GetType() == b3Shape::e_triangle);
	B3_ASSERT(fixtureB->GetType() == b3Shape::e_hull);
}

void b3TriangleAndHullContact::Evaluate(b3Manifold& manifold, const b3Transform& xfA, const b3Transform& xfB)
{
	b3Transform xf0A = m_fixtureA->GetBody()->GetSweep().GetTransform(scalar(0));
	b3Transform xf0B = m_fixtureB->GetBody()->GetSweep().GetTransform(scalar(0));

	b3CollideTriangleAndHull(manifold, xfA, (b3TriangleShape*)m_fixtureA->GetShape(), xfB, (b3HullShape*)m_fixtureB->GetShape(), &m_cache, xf0A, xf0B);
}