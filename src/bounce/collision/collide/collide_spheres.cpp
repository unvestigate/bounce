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

#include <bounce/collision/collision.h>
#include <bounce/collision/shapes/sphere_shape.h>

void b3CollideSpheres(b3Manifold& manifold, 
	const b3Transform& xf1, const b3SphereShape* sphere1,
	const b3Transform& xf2, const b3SphereShape* sphere2)
{
	b3Vec3 c1 = xf1 * sphere1->m_center;
	scalar r1 = sphere1->m_radius;

	b3Vec3 c2 = xf2 * sphere2->m_center;
	scalar r2 = sphere2->m_radius;
	
	b3Vec3 d = c2 - c1;
	scalar dd = b3Dot(d, d);
	scalar totalRadius = r1 + r2;
	if (dd > totalRadius * totalRadius)
	{
		return;
	}

	scalar distance = b3Length(d);
	b3Vec3 normal(scalar(0), scalar(1), scalar(0));
	if (distance > B3_EPSILON)
	{
		normal = d / distance;
	}
	
	manifold.pointCount = 1;
	manifold.points[0].localNormal1 = b3MulC(xf1.rotation, normal);
	manifold.points[0].localPoint1 = sphere1->m_center;
	manifold.points[0].localPoint2 = sphere2->m_center;
	manifold.points[0].id = b3MakeID(0, 0);
}