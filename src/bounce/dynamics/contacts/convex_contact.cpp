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

#include <bounce/dynamics/contacts/convex_contact.h>
#include <bounce/dynamics/fixture.h>
#include <bounce/dynamics/body.h>

b3ConvexContact::b3ConvexContact(b3Fixture* fixtureA, b3Fixture* fixtureB) : b3Contact(fixtureA, fixtureB)
{
	B3_NOT_USED(fixtureA);
    B3_NOT_USED(fixtureB);

	m_manifoldCapacity = 1;
	m_manifolds = &m_manifold;
	m_manifoldCount = 0;
}

bool b3ConvexContact::TestOverlap()
{
	b3Shape* shapeA = m_fixtureA->GetShape();
	b3Transform xfA = m_fixtureA->GetBody()->GetTransform();

	b3Shape* shapeB = m_fixtureB->GetShape();
	b3Transform xfB = m_fixtureB->GetBody()->GetTransform();

	return b3TestOverlap(xfA, 0, shapeA, xfB, 0, shapeB);
}

void b3ConvexContact::Collide() 
{
	b3Transform xfA = m_fixtureA->GetBody()->GetTransform();
	b3Transform xfB = m_fixtureB->GetBody()->GetTransform();

	B3_ASSERT(m_manifoldCount == 0);
	Evaluate(m_manifold, xfA, xfB);
	m_manifoldCount = 1;
}