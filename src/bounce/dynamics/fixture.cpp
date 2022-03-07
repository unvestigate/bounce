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

#include <bounce/dynamics/fixture.h>
#include <bounce/dynamics/body.h>
#include <bounce/dynamics/world.h>
#include <bounce/dynamics/contacts/contact.h>
#include <bounce/collision/broad_phase.h>
#include <bounce/collision/shapes/sphere_shape.h>
#include <bounce/collision/shapes/capsule_shape.h>
#include <bounce/collision/shapes/triangle_shape.h>
#include <bounce/collision/shapes/hull_shape.h>
#include <bounce/collision/shapes/mesh_shape.h>
#include <bounce/collision/geometry/hull.h>
#include <bounce/collision/geometry/mesh.h>
#include <bounce/common/memory/block_allocator.h>
#include <bounce/common/draw.h>

b3Fixture::b3Fixture()
{
	m_body = nullptr;
	m_prev = nullptr;
	m_next = nullptr;
	m_shape = nullptr;
	m_density = scalar(0);
}

void b3Fixture::Create(b3BlockAllocator* allocator, b3Body* body, const b3FixtureDef* def)
{
	m_userData = def->userData;
	m_friction = def->friction;
	m_restitution = def->restitution;
	
	m_body = body;
	
	m_isSensor = def->isSensor;
	
	m_shape = def->shape->Clone(allocator);
	
	m_density = def->density;
}

void b3Fixture::Destroy(b3BlockAllocator* allocator)
{
	// Free the child shape.
	switch (m_shape->m_type)
	{
	case b3Shape::e_sphere:
	{
		b3SphereShape* s = (b3SphereShape*)m_shape;
		s->~b3SphereShape();
		allocator->Free(s, sizeof(b3SphereShape));
		break;
	}
	case b3Shape::e_capsule:
	{
		b3CapsuleShape* s = (b3CapsuleShape*)m_shape;
		s->~b3CapsuleShape();
		allocator->Free(s, sizeof(b3CapsuleShape));
		break;
	}
	case b3Shape::e_triangle:
	{
		b3TriangleShape* s = (b3TriangleShape*)m_shape;
		s->~b3TriangleShape();
		allocator->Free(s, sizeof(b3TriangleShape));
		break;
	}
	case b3Shape::e_hull:
	{
		b3HullShape* s = (b3HullShape*)m_shape;
		s->~b3HullShape();
		allocator->Free(s, sizeof(b3HullShape));
		break;
	}
	case b3Shape::e_mesh:
	{
		b3MeshShape* s = (b3MeshShape*)m_shape;
		s->~b3MeshShape();
		allocator->Free(s, sizeof(b3MeshShape));
		break;
	}
	default:
	{
		B3_ASSERT(false);
		break;
	}
	}
	
	m_shape = nullptr;
}

void b3Fixture::SetSensor(bool flag)
{
	if (flag != m_isSensor)
	{
		if (m_body)
		{
			m_body->SetAwake(true);
		}
		m_isSensor = flag;
	}
}

void b3Fixture::DestroyContacts()
{
	b3World* world = m_body->GetWorld();
	b3ContactEdge* ce = m_contactList.m_head;
	while (ce)
	{
		b3ContactEdge* tmp = ce;
		ce = ce->m_next;
		world->m_contactManager.Destroy(tmp->m_contact);
	}
}

void b3Fixture::Synchronize(b3BroadPhase* broadPhase, const b3Transform& transform1, const b3Transform& transform2)
{
	// Compute an AABB that covers the swept shape (may miss some rotation effect).
	b3AABB aabb1, aabb2;
	m_shape->ComputeAABB(&aabb1, transform1);
	m_shape->ComputeAABB(&aabb2, transform2);

	b3AABB aabb = b3Combine(aabb1, aabb2);

	b3Vec3 displacement = aabb2.GetCenter() - aabb1.GetCenter();

	broadPhase->MoveProxy(m_broadPhaseID, aabb, displacement);
}

const b3AABB& b3Fixture::GetFatAABB() const
{
	return m_body->m_world->m_contactManager.m_broadPhase.GetFatAABB(m_broadPhaseID);
}

void b3Fixture::Dump(u32 bodyIndex) const
{
	switch (GetType())
	{
	case b3Shape::e_sphere:
	{
		b3SphereShape* s = (b3SphereShape*) m_shape;
		b3Log("		b3SphereShape shape;\n");
		b3Log("		shape.m_center.Set(%f, %f, %f);\n", s->m_center.x, s->m_center.y, s->m_center.z);
		b3Log("		shape.m_radius = %f;\n", s->m_radius);
		break;
	}
	case b3Shape::e_capsule:
	{
		b3CapsuleShape* s = (b3CapsuleShape*)m_shape;
		b3Log("		b3CapsuleShape shape;\n");
		b3Log("		shape.m_centers[0].Set(%f, %f, %f);\n", s->m_vertex1.x, s->m_vertex1.y, s->m_vertex1.z);
		b3Log("		shape.m_centers[1].Set(%f, %f, %f);\n", s->m_vertex2.x, s->m_vertex2.y, s->m_vertex2.z);
		b3Log("		shape.m_radius = %f;\n", s->m_radius);
		break;
	}
	case b3Shape::e_triangle:
	{
		b3TriangleShape* s = (b3TriangleShape*)m_shape;
		b3Log("		b3TriangleShape shape;\n");
		b3Log("		shape.m_vertex1.Set(%f, %f, %f);\n", s->m_vertex1.x, s->m_vertex1.y, s->m_vertex1.z);
		b3Log("		shape.m_vertex2.Set(%f, %f, %f);\n", s->m_vertex2.x, s->m_vertex2.y, s->m_vertex2.z);
		b3Log("		shape.m_vertex3.Set(%f, %f, %f);\n", s->m_vertex3.x, s->m_vertex3.y, s->m_vertex3.z);
		b3Log("		shape.m_radius = %f;\n", s->m_radius);
		break;
	}
	case b3Shape::e_hull:
	{
		b3HullShape* s = (b3HullShape*)m_shape;
		const b3Hull* h = s->m_hull;
		
		b3Log("		u8* marker = (u8*) b3Alloc(%d);\n", h->GetSize());
		b3Log("		\n");
		b3Log("		b3Hull* h = (b3Hull*)marker;\n");
		b3Log("		marker += 1 * sizeof(b3Hull);\n");
		b3Log("		h->vertices = (b3Vec3*)marker;\n");
		b3Log("		marker += %d * sizeof(b3Vec3);\n", h->vertexCount);
		b3Log("		h->edges = (b3HalfEdge*)marker;\n");
		b3Log("		marker += %d * sizeof(b3HalfEdge);\n", h->edgeCount);
		b3Log("		h->faces = (b3Face*)marker;\n");
		b3Log("		marker += %d * sizeof(b3Face);\n", h->faceCount);
		b3Log("		h->planes = (b3Plane*)marker;\n");
		b3Log("		marker += %d * sizeof(b3Plane);\n", h->faceCount);
		b3Log("		\n");
		b3Log("		h->centroid.Set(%f, %f, %f);\n", h->centroid.x, h->centroid.y, h->centroid.z);
		b3Log("		\n");
		b3Log("		h->vertexCount = %d;\n", h->vertexCount);
		for (u32 i = 0; i < h->vertexCount; ++i)
		{
			const b3Vec3* v = h->vertices + i;
			b3Log("		h->vertices[%d].Set(%f, %f, %f);\n", i, v->x, v->y, v->z);
		}
		b3Log("		\n");
		b3Log("		h->edgeCount = %d;\n", h->edgeCount);
		for (u32 i = 0; i < h->edgeCount; ++i)
		{
			const b3HalfEdge* e = h->edges + i;
			b3Log("		h->edges[%d].origin = %d;\n", i, e->origin);
			b3Log("		h->edges[%d].twin = %d;\n", i, e->twin);
			b3Log("		h->edges[%d].face = %d;\n", i, e->face);
			b3Log("		h->edges[%d].prev = %d;\n", i, e->prev);
			b3Log("		h->edges[%d].next = %d;\n", i, e->next);
		}
		b3Log("		\n");
		b3Log("		h->faceCount = %d;\n", h->faceCount);
		for (u32 i = 0; i < h->faceCount; ++i)
		{
			const b3Face* f = h->faces + i;
			b3Log("		h->faces[%d].edge = %d;\n", i, f->edge);
		}
		b3Log("		\n");
		for (u32 i = 0; i < h->faceCount; ++i)
		{
			const b3Plane* p = h->planes + i;
			b3Log("		h->planes[%d].normal.Set(%f, %f, %f);\n", i, p->normal.x, p->normal.y, p->normal.z);
			b3Log("		h->planes[%d].offset = %f;\n", i, p->offset);
		}
		b3Log("		\n");
		b3Log("		h->Validate();\n");
		b3Log("		\n");
		b3Log("		b3HullShape shape;\n");
		b3Log("		shape.m_hull = h;\n");
		b3Log("		shape.m_radius = %f;\n", s->m_radius);
		break;
	}
	case b3Shape::e_mesh:
	{
		b3MeshShape* s = (b3MeshShape*)m_shape;
		const b3Mesh* m = s->m_mesh;
		
		b3Log("		u8* marker = (u8*) b3Alloc(%d);\n", m->GetSize());
		b3Log("		\n");
		b3Log("		b3Mesh* m = (b3Hull*)marker;\n");
		b3Log("		marker += 1 * sizeof(b3Mesh);\n");
		b3Log("		m->vertices = (b3Vec3*)marker;\n");
		b3Log("		marker += %d * sizeof(b3Vec3);\n", m->vertexCount);
		b3Log("		m->triangles = (b3MeshTriangle*)marker;\n");
		b3Log("		marker += %d * sizeof(b3MeshTriangle);\n", m->triangleCount);
		b3Log("		m->planes = (b3Plane*)marker;\n");
		b3Log("		marker += %d * sizeof(b3Plane);\n", 2 * m->triangleCount);
		b3Log("		\n");
		for (u32 i = 0; i < m->vertexCount; ++i)
		{
			const b3Vec3* v = m->vertices + i;
			b3Log("		m->vertices[%d].Set(%f, %f, %f);\n", i, v->x, v->y, v->z);
		}
		b3Log("		\n");
		for (u32 i = 0; i < m->triangleCount; ++i)
		{
			const b3MeshTriangle* t = m->triangles + i;
			b3Log("		m->triangles[%d].v1 = %d;\n", i, t->v1);
			b3Log("		m->triangles[%d].v2 = %d;\n", i, t->v2);
			b3Log("		m->triangles[%d].v3 = %d;\n", i, t->v3);
		}
		b3Log("		\n");
		b3Log("		\n");
		b3Log("		m->BuildTree();\n");		
		b3Log("		m->BuildAdjacency();\n");
		b3Log("		\n");
		b3Log("		b3MeshShape shape;\n");
		b3Log("		shape.m_mesh = m;\n");
		b3Log("		shape.m_radius = %f;\n", s->m_radius);
		break;
	}
	default:
	{
		B3_ASSERT(false);
		break;
	}
	};

	b3Log("		\n");
	b3Log("		b3FixtureDef fd;\n");
	b3Log("		fd.shape = &shape;\n");
	b3Log("		fd.density = %f;\n", m_density);
	b3Log("		fd.restitution = %f;\n", m_restitution);
	b3Log("		fd.friction = %f;\n", m_friction);
	b3Log("		fd.sensor = %d;\n", m_isSensor);
	b3Log("		\n");
	b3Log("		bodies[%d]->CreateFixture(fd);\n", bodyIndex);
}