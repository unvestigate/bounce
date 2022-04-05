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

#ifndef B3_HULL_TRIANGLE_CONTACT_H
#define B3_HULL_TRIANGLE_CONTACT_H

#include <bounce/dynamics/contacts/convex_contact.h>

class b3HullAndTriangleContact : public b3ConvexContact
{
public:
	static b3Contact* Create(b3Fixture* fixtureA, b3Fixture* fixtureB, b3BlockAllocator* allocator);
	static void Destroy(b3Contact* contact, b3BlockAllocator* allocator);

	b3HullAndTriangleContact(b3Fixture* fixtureA, b3Fixture* fixtureB);
	~b3HullAndTriangleContact() { }

	void Evaluate(b3Manifold& manifold, const b3Transform& xfA, const b3Transform& xfB) override;
};

#endif
