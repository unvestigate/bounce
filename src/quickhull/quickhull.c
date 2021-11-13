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

#include <quickhull/quickhull.h>

#if !defined(qh_alloc_and_free)

#include <stdlib.h> // NULL, malloc, free

#define qh_alloc(size) malloc(size)
#define qh_free(p) free(p)

#define qh_alloc_and_free // #if !defined(qh_alloc_and_free)

#endif // #if !defined(qh_alloc_and_free)

#include <assert.h>
#include <float.h>
#include <math.h>

float qh_abs_float(float x)
{
	return x < 0.0f ? -x : x;
}

float qh_sqrt_float(float x)
{
	return sqrtf(x);
}

qh_vec3_t qh_vec3_make(float x, float y, float z)
{
	qh_vec3_t v;
	v.x = x;
	v.y = y;
	v.z = z;
	return v;
}

qh_vec3_t qh_vec3_zero()
{
	return qh_vec3_make(0.0f, 0.0f, 0.0f);
}

qh_vec3_t qh_vec3_add(qh_vec3_t a, qh_vec3_t b)
{
	return qh_vec3_make(a.x + b.x, a.y + b.y, a.z + b.z);
}

qh_vec3_t qh_vec3_sub(qh_vec3_t a, qh_vec3_t b)
{
	return qh_vec3_make(a.x - b.x, a.y - b.y, a.z - b.z);
}

qh_vec3_t qh_vec3_mul(float a, qh_vec3_t b)
{
	return qh_vec3_make(a * b.x, a * b.y, a * b.z);
}

qh_vec3_t qh_vec3_div(qh_vec3_t a, float b)
{
	return qh_vec3_make(a.x / b, a.y / b, a.z / b);
}

qh_vec3_t qh_vec3_negate(qh_vec3_t a)
{
	return qh_vec3_make(-a.x, -a.y, -a.z);
}

float qh_vec3_dot(qh_vec3_t a, qh_vec3_t b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

float qh_vec3_length(qh_vec3_t a)
{
	return qh_sqrt_float(a.x * a.x + a.y * a.y + a.z * a.z);
}

qh_vec3_t qh_vec3_normalize(qh_vec3_t a)
{
	float len = qh_vec3_length(a);
	assert(len > 0.0f);
	return qh_vec3_div(a, len);
}

float qh_vec3_length_squared(qh_vec3_t a)
{
	return a.x * a.x + a.y * a.y + a.z * a.z;
}

float qh_vec3_distance_squared(qh_vec3_t a, qh_vec3_t b)
{
	return qh_vec3_length_squared(qh_vec3_sub(a, b));
}

qh_vec3_t qh_vec3_cross(qh_vec3_t a, qh_vec3_t b)
{
	return qh_vec3_make(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

float qh_distance(qh_vec3_t p, qh_plane_t plane)
{
	return qh_vec3_dot(plane.n, p) - plane.d;
}

float qh_abs_distance(qh_vec3_t p, qh_plane_t plane)
{
	return qh_abs_float(qh_distance(p, plane));
}

typedef struct qh_vertex_t qh_vertex_t;
typedef struct qh_half_edge_t qh_half_edge_t;
typedef struct qh_face_t qh_face_t;

struct qh_vertex_t
{
	qh_vertex_t* prev;
	qh_vertex_t* next;

	qh_vec3_t position;

	qh_face_t* conflict_face;

	qh_vertex_t* free_next;
	int is_active;
};

struct qh_half_edge_t
{
	qh_vertex_t* tail;

	qh_half_edge_t* prev;
	qh_half_edge_t* next;
	qh_half_edge_t* twin;

	qh_face_t* face;

	qh_half_edge_t* free_next;
	int is_active;
};

struct qh_face_t
{
	qh_face_t* prev;
	qh_face_t* next;

	qh_half_edge_t* edge;

	qh_vertex_t* conflict_vertex_head;
	int conflict_vertex_count;

	qh_vec3_t center;
	qh_plane_t plane;
	float area;

	int is_visible;

	qh_face_t* free_next;
	int is_active;
};

int qh_face_vertex_count(const qh_face_t* face)
{
	int n = 0;
	qh_half_edge_t* e = face->edge;
	do
	{
		++n;
		e = e->next;
	} while (e != face->edge);
	return n;
}

void qh_face_insert_conflict_vertex(qh_face_t* face, qh_vertex_t* vertex)
{
	vertex->prev = NULL;
	vertex->next = face->conflict_vertex_head;
	if (face->conflict_vertex_head)
	{
		face->conflict_vertex_head->prev = vertex;
	}
	face->conflict_vertex_head = vertex;
	++face->conflict_vertex_count;
}

void qh_face_remove_conflict_vertex(qh_face_t* face, qh_vertex_t* vertex)
{
	if (vertex->prev)
	{
		vertex->prev->next = vertex->next;
	}

	if (vertex->next)
	{
		vertex->next->prev = vertex->prev;
	}

	if (vertex == face->conflict_vertex_head)
	{
		face->conflict_vertex_head = vertex->next;
	}

	--face->conflict_vertex_count;
}

typedef struct qh_hull_t
{
	void* memory;

	int free_vertex_capacity;
	int free_vertex_count;
	qh_vertex_t* free_vertices;

	int free_edge_capacity;
	int free_edge_count;
	qh_half_edge_t* free_edges;

	int free_face_capacity;
	int free_face_count;
	qh_face_t* free_faces;

	int horizon_edge_count;
	qh_half_edge_t** horizon_edge_buffer;
	qh_vertex_t** horizon_vertex_buffer;

	int conflict_vertex_count;
	qh_vertex_t** conflict_vertex_buffer;

	int new_face_count;
	qh_face_t** new_face_buffer;

	float coplanarity_tolerance;

	qh_vertex_t* vertex_head;
	int vertex_count;

	qh_face_t* face_head;
	int face_count;

	int iterations;
} qh_hull_t;

qh_vertex_t* qh_hull_allocate_vertex(qh_hull_t* hull)
{
	assert(hull->free_vertex_count < hull->free_vertex_capacity);
	++hull->free_vertex_count;

	qh_vertex_t* vertex = hull->free_vertices;
	assert(vertex->is_active == 0);
	vertex->is_active = 1;
	hull->free_vertices = vertex->free_next;
	return vertex;
}

void qh_hull_free_vertex(qh_hull_t* hull, qh_vertex_t* vertex)
{
	assert(hull->free_vertex_count > 0);
	--hull->free_vertex_count;

	assert(vertex->is_active == 1);
	vertex->is_active = 0;
	vertex->free_next = hull->free_vertices;
	hull->free_vertices = vertex;
}

qh_half_edge_t* qh_hull_allocate_edge(qh_hull_t* hull)
{
	assert(hull->free_edge_count < hull->free_edge_capacity);
	++hull->free_edge_count;

	qh_half_edge_t* edge = hull->free_edges;
	assert(edge->is_active == 0);
	edge->is_active = 1;
	hull->free_edges = edge->free_next;
	return edge;
}

void qh_hull_free_edge(qh_hull_t* hull, qh_half_edge_t* edge)
{
	assert(hull->free_edge_count > 0);
	--hull->free_edge_count;

	assert(edge->is_active == 1);
	edge->is_active = 0;
	edge->free_next = hull->free_edges;
	hull->free_edges = edge;
}

qh_face_t* qh_hull_allocate_face(qh_hull_t* hull)
{
	assert(hull->free_face_count < hull->free_face_capacity);
	++hull->free_face_count;

	qh_face_t* face = hull->free_faces;
	assert(face->is_active == 0);
	face->is_active = 1;
	hull->free_faces = face->free_next;
	return face;
}

void qh_hull_free_face(qh_hull_t* hull, qh_face_t* face)
{
	assert(hull->free_face_count > 0);
	--hull->free_face_count;

	assert(face->is_active == 1);
	face->is_active = 0;
	face->free_next = hull->free_faces;
	hull->free_faces = face;
}

void qh_half_edge_validate(const qh_half_edge_t* edge)
{
	assert(edge->is_active == 1);

	const qh_half_edge_t* twin = edge->twin;
	assert(twin->is_active == 1);
	assert(twin->twin == edge);

	assert(edge->tail->is_active == 1);
	qh_vec3_t A = edge->tail->position;

	assert(twin->tail->is_active == 1);
	qh_vec3_t B = twin->tail->position;

	assert(qh_vec3_distance_squared(A, B) > FLT_EPSILON * FLT_EPSILON);

	const qh_half_edge_t* next = edge->next;
	assert(next->is_active == 1);
	assert(twin->tail == next->tail);

	{
		// CCW
		int found = 0;
		const qh_face_t* face = edge->face;
		const qh_half_edge_t* e = face->edge;
		do
		{
			if (e == edge)
			{
				found = 1;
				break;
			}
			e = e->next;
		} while (e != face->edge);

		assert(found == 1);
	}

	{
		// CW
		int found = 0;
		const qh_face_t* face = edge->face;
		const qh_half_edge_t* e = face->edge;
		do
		{
			if (e == edge)
			{
				found = 1;
				break;
			}
			e = e->prev;
		} while (e != face->edge);

		assert(found == 1);
	}
}

void qh_face_validate(const qh_face_t* face)
{
	assert(face->is_active == 1);

	// Conflict
	int conflict_vertex_count = 0;
	qh_vertex_t* conflict_vertex = face->conflict_vertex_head;
	while (conflict_vertex != NULL)
	{
		assert(conflict_vertex->is_active == 1);
		assert(conflict_vertex->conflict_face == face);
		++conflict_vertex_count;
		conflict_vertex = conflict_vertex->next;
	}
	assert(conflict_vertex_count == face->conflict_vertex_count);

	// CCW
	{
		const qh_half_edge_t* edge = face->edge;
		do
		{
			assert(edge->is_active == 1);
			assert(edge->face == face);

			assert(edge->twin != NULL);
			assert(edge->twin->is_active == 1);

			if (edge->twin->face != NULL)
			{
				assert(edge->twin->face->is_active == 1);
				assert(edge->twin->face != face);
			}

			edge = edge->next;
		} while (edge != face->edge);
	}

	// CW
	{
		const qh_half_edge_t* edge = face->edge;
		do
		{
			assert(edge->is_active == 1);
			assert(edge->face == face);

			assert(edge->twin != NULL);
			assert(edge->twin->is_active == 1);

			if (edge->twin->face != NULL)
			{
				assert(edge->twin->face->is_active == 1);
				assert(edge->twin->face != face);
			}

			edge = edge->prev;
		} while (edge != face->edge);
	}

	{
		const qh_half_edge_t* edge = face->edge;
		do
		{
			qh_half_edge_validate(edge);

			edge = edge->next;
		} while (edge != face->edge);
	}
}

int qh_hull_has_vertex(const qh_hull_t* hull, qh_vertex_t* vertex)
{
	for (qh_vertex_t* hull_vertex = hull->vertex_head; hull_vertex != NULL; hull_vertex = hull_vertex->next)
	{
		assert(hull_vertex->is_active == 1);
		if (hull_vertex == vertex)
		{
			return 1;
		}
	}
	return 0;
}

void qh_hull_validate(const qh_hull_t* hull)
{
	for (qh_vertex_t* vertex = hull->vertex_head; vertex != NULL; vertex = vertex->next)
	{
		assert(vertex->is_active == 1);
	}

	for (qh_face_t* face = hull->face_head; face != NULL; face = face->next)
	{
		assert(face->is_active == 1);

		for (qh_vertex_t* vertex = face->conflict_vertex_head; vertex != NULL; vertex = vertex->next)
		{
			assert(vertex->is_active == 1);
		}

		qh_half_edge_t* ein = face->edge;
		do
		{
			assert(ein->is_active == 1);

			// Tail vertex must be contained in the hull.
			assert(ein->tail->is_active == 1);
			int has_tail_vertex = qh_hull_has_vertex(hull, ein->tail);
			assert(has_tail_vertex == 1);

			qh_half_edge_t* eout = ein->next;

			assert(ein->twin->is_active == 1);
			assert(eout->is_active == 1);
			assert(eout->twin->is_active == 1);

			// Each vertex has at least three neighbor faces
			assert(ein->twin->face != eout->twin->face);

			ein = eout;
		} while (ein != face->edge);

		qh_face_validate(face);
	}
}

void qh_hull_insert_vertex(qh_hull_t* hull, qh_vertex_t* vertex)
{
	vertex->prev = NULL;
	vertex->next = hull->vertex_head;
	if (hull->vertex_head)
	{
		hull->vertex_head->prev = vertex;
	}
	hull->vertex_head = vertex;
	++hull->vertex_count;
}

void qh_hull_remove_vertex(qh_hull_t* hull, qh_vertex_t* vertex)
{
	if (vertex->prev)
	{
		vertex->prev->next = vertex->next;
	}
	if (vertex->next)
	{
		vertex->next->prev = vertex->prev;
	}
	if (vertex == hull->vertex_head)
	{
		hull->vertex_head = vertex->next;
	}
	--hull->vertex_count;
}

qh_vertex_t* qh_hull_create_vertex(qh_hull_t* hull, qh_vec3_t position)
{
	qh_vertex_t* vertex = qh_hull_allocate_vertex(hull);
	vertex->position = position;
	vertex->conflict_face = NULL;

	qh_hull_insert_vertex(hull, vertex);

	return vertex;
}

qh_half_edge_t* qh_hull_find_half_edge(const qh_hull_t* hull, const qh_vertex_t* v1, const qh_vertex_t* v2)
{
	for (qh_face_t* face = hull->face_head; face != NULL; face = face->next)
	{
		qh_half_edge_t* e = face->edge;
		do
		{
			assert(e->is_active == 1);
			assert(e->twin != NULL);
			assert(e->twin->is_active == 1);

			if (e->tail == v1 && e->twin->tail == v2)
			{
				return e;
			}

			if (e->tail == v2 && e->twin->tail == v1)
			{
				return e->twin;
			}

			e = e->next;
		} while (e != face->edge);
	}
	return NULL;
}

qh_vec3_t qh_vec3_newell(qh_vec3_t a, qh_vec3_t b)
{
	return qh_vec3_make((a.y - b.y) * (a.z + b.z), (a.z - b.z) * (a.x + b.x), (a.x - b.x) * (a.y + b.y));
}

void qh_face_reset_face_data(qh_face_t* face)
{
	// Compute face centroid, normal, and area

	// Compute polygon centroid
	qh_vec3_t c = qh_vec3_zero();

	int count = 0;
	qh_half_edge_t* e = face->edge;
	do
	{
		qh_vec3_t v = e->tail->position;
		c = qh_vec3_add(c, v);
		++count;
		e = e->next;
	} while (e != face->edge);

	assert(count >= 3);
	c = qh_vec3_div(c, (float)count);

	// Compute normal  
	qh_vec3_t n = qh_vec3_zero();

	e = face->edge;
	do
	{
		qh_vec3_t v1 = e->tail->position;
		qh_vec3_t v2 = e->next->tail->position;

		// Shift the polygon origin to the center
		v1 = qh_vec3_sub(v1, c);
		v2 = qh_vec3_sub(v2, c);

		// Apply Newell's method
		n = qh_vec3_add(n, qh_vec3_newell(v1, v2));

		e = e->next;
	} while (e != face->edge);

	// Center
	face->center = c;

	float len = qh_vec3_length(n);
	assert(len > 0.0f);
	n = qh_vec3_div(n, len);

	// Area
	face->area = 0.5f * len;

	// Normal
	face->plane.n = n;
	face->plane.d = qh_vec3_dot(n, c);
}

void qh_hull_insert_face(qh_hull_t* hull, qh_face_t* face)
{
	face->prev = NULL;
	face->next = hull->face_head;
	if (hull->face_head)
	{
		hull->face_head->prev = face;
	}
	hull->face_head = face;
	++hull->face_count;
}

void qh_hull_remove_face(qh_hull_t* hull, qh_face_t* face)
{
	if (face->prev)
	{
		face->prev->next = face->next;
	}
	if (face->next)
	{
		face->next->prev = face->prev;
	}
	if (face == hull->face_head)
	{
		hull->face_head = face->next;
	}
	--hull->face_count;
}

qh_face_t* qh_hull_create_face(qh_hull_t* hull, qh_vertex_t* v1, qh_vertex_t* v2, qh_vertex_t* v3)
{
	qh_face_t* face = qh_hull_allocate_face(hull);

	qh_half_edge_t* e1 = qh_hull_find_half_edge(hull, v1, v2);
	if (e1 == NULL)
	{
		e1 = qh_hull_allocate_edge(hull);
		e1->tail = v1;
		e1->face = face;
		e1->prev = NULL;
		e1->next = NULL;

		e1->twin = qh_hull_allocate_edge(hull);
		e1->twin->tail = v2;
		e1->twin->face = NULL;
		e1->twin->prev = NULL;
		e1->twin->next = NULL;
		e1->twin->twin = e1;
	}
	else
	{
		// Edge must be a boundary edge.
		assert(e1->face == NULL);
		e1->face = face;

		assert(e1->tail == v1);
		assert(e1->twin != NULL);
		assert(e1->twin->is_active == 1);
		assert(e1->twin->tail == v2);
	}

	qh_half_edge_t* e2 = qh_hull_find_half_edge(hull, v2, v3);
	if (e2 == NULL)
	{
		e2 = qh_hull_allocate_edge(hull);
		e2->tail = v2;
		e2->face = face;
		e2->prev = NULL;
		e2->next = NULL;

		e2->twin = qh_hull_allocate_edge(hull);
		e2->twin->tail = v3;
		e2->twin->face = NULL;
		e2->twin->prev = NULL;
		e2->twin->next = NULL;
		e2->twin->twin = e2;
	}
	else
	{
		assert(e2->face == NULL);
		e2->face = face;

		assert(e2->tail == v2);
		assert(e2->twin != NULL);
		assert(e2->twin->is_active == 1);
		assert(e2->twin->tail == v3);
	}

	qh_half_edge_t* e3 = qh_hull_find_half_edge(hull, v3, v1);
	if (e3 == NULL)
	{
		e3 = qh_hull_allocate_edge(hull);
		e3->tail = v3;
		e3->face = face;
		e3->prev = NULL;
		e3->next = NULL;

		e3->twin = qh_hull_allocate_edge(hull);
		e3->twin->tail = v1;
		e3->twin->face = NULL;
		e3->twin->prev = NULL;
		e3->twin->next = NULL;
		e3->twin->twin = e3;
	}
	else
	{
		assert(e3->face == NULL);
		e3->face = face;

		assert(e3->tail == v3);
		assert(e3->twin != NULL);
		assert(e3->twin->is_active == 1);
		assert(e3->twin->tail == v1);
	}

	assert(e1->prev == NULL);
	e1->prev = e3;
	assert(e1->next == NULL);
	e1->next = e2;

	assert(e2->prev == NULL);
	e2->prev = e1;
	assert(e2->next == NULL);
	e2->next = e3;

	assert(e3->prev == NULL);
	e3->prev = e2;
	assert(e3->next == NULL);
	e3->next = e1;

	// Link edge to face
	face->edge = e1;

	// Compute face data 
	qh_face_reset_face_data(face);

	// Initilize conflict list
	face->conflict_vertex_head = NULL;
	face->conflict_vertex_count = 0;

	// Validate face
	qh_face_validate(face);

	// Insert into hull list of faces
	qh_hull_insert_face(hull, face);

	return face;
}

void qh_hull_destroy_face(qh_hull_t* hull, qh_face_t* face)
{
	// This code assumes the conflict vertices have been removed
	assert(face->conflict_vertex_count == 0);

	// Destroy half-edges 
	qh_half_edge_t* e = face->edge;
	do
	{
		qh_half_edge_t* e0 = e;
		e = e->next;

		// Is the edge a boundary?
		if (e0->twin->face == NULL)
		{
			// Edge is non-shared.
			qh_hull_free_edge(hull, e0->twin);
			qh_hull_free_edge(hull, e0);
		}
		else
		{
			// Edge is shared. 
			// Mark the twin edge as a boundary edge.
			assert(e0->twin != NULL);
			assert(e0->twin->twin == e0);
			e0->face = NULL;
			e0->prev = NULL;
			e0->next = NULL;
		}

	} while (e != face->edge);

	// Remove face from hull list
	qh_hull_remove_face(hull, face);

	// Release memory
	qh_hull_free_face(hull, face);
}

qh_vec3_t qh_vec3_array_make(int vertex_stride, const void* vertex_base, int vtx)
{
	float* p = (float*)((char*)vertex_base + vertex_stride * vtx);
	return qh_vec3_make(p[0], p[1], p[2]);
}

int qh_build_initial_hull(qh_hull_t* hull, int vertex_stride, const void* vertex_base, int vertex_count)
{
	if (vertex_count < 4)
	{
		assert(0);
		return 0;
	}

	int i1 = 0, i2 = 0;

	{
		// Find the points that maximizes the distance along the canonical axes.
		// Also store a tolerance for coplanarity checks.
		int aabb_min[3];
		float min_vertex[3];
		min_vertex[0] = FLT_MAX;
		min_vertex[1] = FLT_MAX;
		min_vertex[2] = FLT_MAX;

		int aabb_max[3];
		float max_vertex[3];
		max_vertex[0] = -FLT_MAX;
		max_vertex[1] = -FLT_MAX;
		max_vertex[2] = -FLT_MAX;

		for (int i = 0; i < vertex_count; ++i)
		{
			qh_vec3_t v = qh_vec3_array_make(vertex_stride, vertex_base, i);

			float vertex[3];
			vertex[0] = v.x;
			vertex[1] = v.y;
			vertex[2] = v.z;

			for (int j = 0; j < 3; ++j)
			{
				if (vertex[j] < min_vertex[j])
				{
					min_vertex[j] = vertex[j];
					aabb_min[j] = i;
				}

				if (vertex[j] > max_vertex[j])
				{
					max_vertex[j] = vertex[j];
					aabb_max[j] = i;
				}
			}
		}

		// Store coplanarity tolerance.
		hull->coplanarity_tolerance = 3.0f * (qh_abs_float(max_vertex[0]) + qh_abs_float(max_vertex[1]) + qh_abs_float(max_vertex[2])) * FLT_EPSILON;

		// Find the longest segment.
		float d0 = 0.0f;

		for (int i = 0; i < 3; ++i)
		{
			int v1 = aabb_min[i];
			int v2 = aabb_max[i];

			qh_vec3_t A = qh_vec3_array_make(vertex_stride, vertex_base, v1);
			qh_vec3_t B = qh_vec3_array_make(vertex_stride, vertex_base, v2);

			float d = qh_vec3_distance_squared(A, B);

			if (d > d0)
			{
				d0 = d;
				i1 = aabb_min[i];
				i2 = aabb_max[i];
			}
		}

		// Coincidence check
		const float coincidence_tol = FLT_EPSILON;
		if (d0 <= coincidence_tol * coincidence_tol)
		{
			assert(0);
			return 0;
		}
	}

	assert(i1 != i2);

	qh_vec3_t A = qh_vec3_array_make(vertex_stride, vertex_base, i1);
	qh_vec3_t B = qh_vec3_array_make(vertex_stride, vertex_base, i2);

	int i3 = 0;

	{
		// Find the triangle which has the largest area.
		float a0 = 0.0f;

		for (int i = 0; i < vertex_count; ++i)
		{
			if (i == i1 || i == i2)
			{
				continue;
			}

			qh_vec3_t C = qh_vec3_array_make(vertex_stride, vertex_base, i);

			qh_vec3_t N = qh_vec3_cross(qh_vec3_sub(B, A), qh_vec3_sub(C, A));

			float a = qh_vec3_length_squared(N);

			if (a > a0)
			{
				a0 = a;
				i3 = i;
			}
		}

		// Colinear check.
		const float colinear_tol = 4.0f * FLT_EPSILON * FLT_EPSILON;
		if (a0 <= colinear_tol)
		{
			assert(0);
			return 0;
		}
	}

	assert(i3 != i1 && i3 != i2);

	qh_vec3_t C = qh_vec3_array_make(vertex_stride, vertex_base, i3);

	qh_plane_t plane;
	plane.n = qh_vec3_cross(qh_vec3_sub(B, A), qh_vec3_sub(C, A));
	plane.n = qh_vec3_normalize(plane.n);
	plane.d = qh_vec3_dot(A, plane.n);

	int i4 = 0;

	{
		// Find the furthest point from the triangle plane.
		float d0 = 0.0f;

		for (int i = 0; i < vertex_count; ++i)
		{
			if (i == i1 || i == i2 || i == i3)
			{
				continue;
			}

			qh_vec3_t D = qh_vec3_array_make(vertex_stride, vertex_base, i);

			float d = qh_abs_distance(D, plane);

			if (d > d0)
			{
				d0 = d;
				i4 = i;
			}
		}

		// Coplanar check.
		if (d0 <= hull->coplanarity_tolerance)
		{
			assert(0);
			return 0;
		}
	}

	assert(i4 != i1 && i4 != i2 && i4 != i3);

	// Add okay simplex to the hull.
	qh_vec3_t D = qh_vec3_array_make(vertex_stride, vertex_base, i4);

	qh_vertex_t* v1 = qh_hull_create_vertex(hull, A);
	qh_vertex_t* v2 = qh_hull_create_vertex(hull, B);
	qh_vertex_t* v3 = qh_hull_create_vertex(hull, C);
	qh_vertex_t* v4 = qh_hull_create_vertex(hull, D);

	if (qh_distance(D, plane) < 0.0f)
	{
		qh_hull_create_face(hull, v1, v2, v3);
		qh_hull_create_face(hull, v4, v2, v1);
		qh_hull_create_face(hull, v4, v3, v2);
		qh_hull_create_face(hull, v4, v1, v3);
	}
	else
	{
		// Ensure CCW order.
		qh_hull_create_face(hull, v1, v3, v2);
		qh_hull_create_face(hull, v4, v1, v2);
		qh_hull_create_face(hull, v4, v2, v3);
		qh_hull_create_face(hull, v4, v3, v1);
	}

	// Connectivity check.
	qh_hull_validate(hull);

	// Add remaining points to the conflict lists on each face.
	for (int i = 0; i < vertex_count; ++i)
	{
		// Skip hull vertices.
		if (i == i1 || i == i2 || i == i3 || i == i4)
		{
			continue;
		}

		qh_vec3_t p = qh_vec3_array_make(vertex_stride, vertex_base, i);

		// Ignore internal points since they can't be in the hull.
		float d0 = hull->coplanarity_tolerance;
		qh_face_t* f0 = NULL;

		for (qh_face_t* f = hull->face_head; f != NULL; f = f->next)
		{
			float d = qh_distance(p, f->plane);
			if (d > d0)
			{
				d0 = d;
				f0 = f;
			}
		}

		if (f0)
		{
			// Add conflict vertex to face
			qh_vertex_t* v = qh_hull_allocate_vertex(hull);
			v->position = p;
			v->conflict_face = f0;
			qh_face_insert_conflict_vertex(f0, v);
		}
	}

	return 1;
}

void qh_hull_find_horizon(qh_hull_t* hull, qh_vertex_t* eye)
{
	// Mark all faces as visible or invisible to the eye
	for (qh_face_t* face = hull->face_head; face != NULL; face = face->next)
	{
		float d = qh_distance(eye->position, face->plane);
		if (d > hull->coplanarity_tolerance)
		{
			face->is_visible = 1;
		}
		else
		{
			face->is_visible = 0;
		}
	}

	// Find the horizon 
	hull->horizon_edge_count = 0;
	for (qh_face_t* face = hull->face_head; face != NULL; face = face->next)
	{
		if (face->is_visible == 0)
		{
			continue;
		}

		qh_half_edge_t* begin = face->edge;
		qh_half_edge_t* edge = begin;
		do
		{
			qh_half_edge_t* twin = edge->twin;
			qh_face_t* other = twin->face;

			if (other->is_visible == 0)
			{
				hull->horizon_edge_buffer[hull->horizon_edge_count++] = edge;
			}

			edge = edge->next;
		} while (edge != begin);
	}

	// Sort the horizon in CCW order to avoid recursion.
	assert(hull->horizon_edge_count > 0);
	for (int i = 0; i < hull->horizon_edge_count - 1; ++i)
	{
		qh_half_edge_t* e1 = hull->horizon_edge_buffer[i]->twin;
		qh_vertex_t* v1 = e1->tail;

		for (int j = i + 1; j < hull->horizon_edge_count; ++j)
		{
			// Ensure unique edges
			assert(hull->horizon_edge_buffer[i] != hull->horizon_edge_buffer[j]);

			qh_half_edge_t* e2 = hull->horizon_edge_buffer[j];
			qh_vertex_t* v2 = e2->tail;

			if (v1 == v2)
			{
				// Swap
				qh_half_edge_t* tmp = hull->horizon_edge_buffer[i + 1];
				hull->horizon_edge_buffer[i + 1] = hull->horizon_edge_buffer[j];
				hull->horizon_edge_buffer[j] = tmp;
				break;
			}
		}
	}
}

void qh_hull_add_new_faces(qh_hull_t* hull, qh_vertex_t* eye)
{
	// Ensure CCW horizon order. 
	// Usually it can fail hit if face merging is disable.
	assert(hull->horizon_edge_count > 0);
	for (int i = 0; i < hull->horizon_edge_count; ++i)
	{
		qh_half_edge_t* e1 = hull->horizon_edge_buffer[i]->twin;

		int j = i + 1 < hull->horizon_edge_count ? i + 1 : 0;
		qh_half_edge_t* e2 = hull->horizon_edge_buffer[j];

		assert(e1->tail == e2->tail);
	}

	// Save horizon vertices
	for (int i = 0; i < hull->horizon_edge_count; ++i)
	{
		qh_half_edge_t* edge = hull->horizon_edge_buffer[i];

		hull->horizon_vertex_buffer[i] = edge->tail;
	}

	qh_vec3_t eye_position = eye->position;

	// Remove the eye vertex from the face conflict list
	qh_face_remove_conflict_vertex(eye->conflict_face, eye);

	// Release the eye vertex
	qh_hull_free_vertex(hull, eye);

	// Add the eye vertex to the hull
	qh_vertex_t* v1 = qh_hull_create_vertex(hull, eye_position);

	// Collect conflict vertices of future deleted faces.
	hull->conflict_vertex_count = 0;

	// Destroy visible faces
	qh_face_t* f = hull->face_head;
	while (f)
	{
		qh_face_t* f0 = f;
		f = f->next;

		// Skip invisible faces.
		if (f0->is_visible == 0)
		{
			continue;
		}

		// Extract face conflict vertices from face
		qh_vertex_t* v = f0->conflict_vertex_head;
		while (v)
		{
			qh_vertex_t* v0 = v;
			v = v->next;

			// Save vertex
			hull->conflict_vertex_buffer[hull->conflict_vertex_count++] = v0;

			qh_face_remove_conflict_vertex(f0, v0);
		}

		qh_hull_destroy_face(hull, f0);
	}

	// Add new faces to the hull
	hull->new_face_count = 0;
	for (int i = 0; i < hull->horizon_edge_count; ++i)
	{
		int j = i + 1 < hull->horizon_edge_count ? i + 1 : 0;

		qh_vertex_t* v2 = hull->horizon_vertex_buffer[i];
		qh_vertex_t* v3 = hull->horizon_vertex_buffer[j];

		hull->new_face_buffer[hull->new_face_count++] = qh_hull_create_face(hull, v1, v2, v3);
	}
}

void qh_hull_resolve_orphans(qh_hull_t* hull)
{
	// Move the orphaned conflict vertices into the new faces
	// Remove internal conflict vertices
	for (int i = 0; i < hull->conflict_vertex_count; ++i)
	{
		qh_vertex_t* v = hull->conflict_vertex_buffer[i];

		qh_vec3_t p = v->position;

		float d0 = hull->coplanarity_tolerance;
		qh_face_t* f0 = NULL;

		for (int j = 0; j < hull->new_face_count; ++j)
		{
			qh_face_t* nf = hull->new_face_buffer[j];

			// Was the face deleted due to merging?
			if (nf->is_active == 0)
			{
				continue;
			}

			float d = qh_distance(p, nf->plane);
			if (d > d0)
			{
				d0 = d;
				f0 = nf;
			}
		}

		if (f0)
		{
			// Add conflict vertex to the new face
			qh_face_insert_conflict_vertex(f0, v);
			v->conflict_face = f0;
		}
		else
		{
			// Free conflict vertex. 
			// It has already been removed from its face.
			qh_hull_free_vertex(hull, v);
		}
	}
}

qh_half_edge_t* qh_hull_fix_merge(qh_hull_t* hull, qh_face_t* face1, qh_half_edge_t* ein)
{
	qh_half_edge_t* eout = ein->next;

	assert(ein->twin->face == eout->twin->face);

	qh_face_t* face3 = ein->twin->face;

	int count = qh_face_vertex_count(face3);

	// Is the face 3 a triangle?
	if (count == 3)
	{
		qh_half_edge_t* nextEdge = eout->next;

		// Unlink incoming edge from face 1
		assert(ein->prev->next == ein);
		ein->prev->next = ein->twin->next;

		// Unlink incoming edge twin from face 3
		assert(ein->twin->next->prev == ein->twin);
		ein->twin->next->prev = ein->prev;

		assert(ein->face == face1);
		if (face1->edge == ein)
		{
			face1->edge = ein->prev;
		}

		// Set incoming edge twin face reference the face 1
		assert(ein->twin->next->face == face3);
		ein->twin->next->face = face1;

		// Unlink outgoing edge from face 1
		assert(eout->next->prev == eout);
		eout->next->prev = eout->twin->prev;
		assert(eout->twin->prev->next == eout->twin);
		eout->twin->prev->next = eout->next;

		assert(eout->face == face1);
		if (face1->edge == eout)
		{
			face1->edge = eout->next;
		}

		// Reset face 1 data
		qh_face_reset_face_data(face1);

		// Validate face 1
		qh_face_validate(face1);

		// Remove outgoing vertex
		qh_hull_remove_vertex(hull, eout->tail);
		qh_hull_free_vertex(hull, eout->tail);

		// Remove incoming edge
		qh_hull_free_edge(hull, ein->twin);
		qh_hull_free_edge(hull, ein);

		// Remove outgoing edge
		qh_hull_free_edge(hull, eout->twin);
		qh_hull_free_edge(hull, eout);

		// Move face 3 conflict vertices into face 1
		qh_vertex_t* v = face3->conflict_vertex_head;
		while (v)
		{
			qh_vertex_t* v0 = v;
			v = v->next;

			qh_face_insert_conflict_vertex(face1, v0);
			v0->conflict_face = face1;
		}

		// Remove face 3
		qh_hull_remove_face(hull, face3);
		qh_hull_free_face(hull, face3);

		// Return the next edge
		return nextEdge;
	}

	// Face not a triangle.
	qh_half_edge_t* nextEdge = eout->next;

	// Extend the incoming edge to the next vertex
	assert(ein->twin->tail == eout->tail);
	ein->twin->tail = eout->twin->tail;

	// Remove outgoing vertex
	qh_hull_remove_vertex(hull, eout->tail);
	qh_hull_free_vertex(hull, eout->tail);

	// Unlink outgoing edge from face 1
	assert(eout->prev->next == eout);
	eout->prev->next = eout->next;
	assert(eout->next->prev == eout);
	eout->next->prev = eout->prev;

	assert(eout->face == face1);
	if (face1->edge == eout)
	{
		face1->edge = eout->next;
	}

	// Reset face 1 data
	qh_face_reset_face_data(face1);

	// Validate face 1
	qh_face_validate(face1);

	// Unlink outgoing edge twin from face 3
	assert(eout->twin->prev->next == eout->twin);
	eout->twin->prev->next = eout->twin->next;
	assert(eout->twin->next->prev == eout->twin);
	eout->twin->next->prev = eout->twin->prev;

	assert(eout->twin->face == face3);
	if (face3->edge == eout->twin)
	{
		face3->edge = eout->twin->next;
	}

	// Remove outgoing edge
	qh_hull_free_edge(hull, eout->twin);
	qh_hull_free_edge(hull, eout);

	// Return the next edge
	return nextEdge;
}

int qh_hull_fix_face(qh_hull_t* hull, qh_face_t* face)
{
	// Maintained invariants:
	// - Each vertex must have at least three neighbor faces  
	// - Face 1 must be convex

	// Search a incoming (and outgoing edge) in the face 1
	// which have the same neighbour face.
	qh_half_edge_t* edge = NULL;

	qh_half_edge_t* ein = face->edge;
	do
	{
		qh_half_edge_t* eout = ein->next;

		// Has the outgoing vertex become redundant?
		if (ein->twin->face == eout->twin->face)
		{
			edge = ein;
			break;
		}

		ein = eout;
	} while (ein != face->edge);

	if (edge)
	{
		// Remove the outgoing vertex. 
		qh_hull_fix_merge(hull, face, edge);
		return 1;
	}

	// Topological error not found.
	return 0;
}

qh_face_t* qh_hull_remove_edge(qh_hull_t* hull, qh_half_edge_t* edge)
{
	assert(edge->is_active == 1);

	qh_face_t* face1 = edge->face;
	assert(face1->is_active == 1);
	assert(edge->twin->is_active == 1);
	qh_face_t* face2 = edge->twin->face;

	// Edge must be shared.
	assert(face2 != NULL);
	assert(face2->is_active == 1);
	assert(face2 != face1);

	// Merge face 2 into face 1

	// Set face 2 edges owner face to face 1,
	// except the twin edge which will be deleted
	for (qh_half_edge_t* e2 = edge->twin->next; e2 != edge->twin; e2 = e2->next)
	{
		assert(e2->face == face2);
		e2->face = face1;
	}

	// Set the face 1 to reference a non-deleted edge
	assert(edge->face == face1);
	if (face1->edge == edge)
	{
		face1->edge = edge->next;
	}

	// Unlink edge from face 1
	assert(edge->prev->next == edge);
	edge->prev->next = edge->twin->next;
	assert(edge->next->prev == edge);
	edge->next->prev = edge->twin->prev;

	assert(edge->twin->prev->next == edge->twin);
	edge->twin->prev->next = edge->next;
	assert(edge->twin->next->prev == edge->twin);
	edge->twin->next->prev = edge->prev;

	// Reset face 1 data
	qh_face_reset_face_data(face1);

	// Validate face 1
	qh_face_validate(face1);

	// Move face 2 conflict vertices into face 1
	qh_vertex_t* v = face2->conflict_vertex_head;
	while (v)
	{
		qh_vertex_t* v0 = v;
		v = v->next;

		qh_face_insert_conflict_vertex(face1, v0);
		v0->conflict_face = face1;
	}

	// Remove face 2
	qh_hull_remove_face(hull, face2);
	qh_hull_free_face(hull, face2);

	// Remove edge
	qh_hull_free_edge(hull, edge->twin);
	qh_hull_free_edge(hull, edge);

	// Repair topological errors in the face 
	while (qh_hull_fix_face(hull, face1));

	// Return face 1
	return face1;
}

int qh_hull_merge_face(qh_hull_t* hull, qh_face_t* face1)
{
	float tolerance = hull->coplanarity_tolerance;

	// Non-convex edge
	qh_half_edge_t* edge = NULL;

	qh_half_edge_t* e = face1->edge;

	do
	{
		qh_half_edge_t* twin = e->twin;
		qh_face_t* face2 = twin->face;

		assert(face2 != NULL);
		assert(face2 != face1);

		float d1 = qh_distance(face2->center, face1->plane);
		float d2 = qh_distance(face1->center, face2->plane);

		if (d1 < -tolerance && d2 < -tolerance)
		{
			// Edge is convex
			e = e->next;
			continue;
		}

		// Edge is concave or coplanar
		edge = e;
		break;

	} while (e != face1->edge);

	if (edge)
	{
		qh_hull_remove_edge(hull, edge);
		return 1;
	}

	return 0;
}

int qh_hull_merge_large_face(qh_hull_t* hull, qh_face_t* face1)
{
	float tolerance = hull->coplanarity_tolerance;

	qh_half_edge_t* edge = NULL;

	qh_half_edge_t* e = face1->edge;

	assert(e->face == face1);

	// Find a non-convex edge
	do
	{
		qh_half_edge_t* twin = e->twin;
		qh_face_t* face2 = twin->face;

		assert(face2 != NULL);
		assert(face2 != face1);

		if (face1->area > face2->area)
		{
			// Face 1 merge
			float d = qh_distance(face2->center, face1->plane);
			if (d < -tolerance)
			{
				// Edge is convex wrt to the face 1
				e = e->next;
				continue;
			}

			// Edge is concave or coplanar wrt to the face 1
			edge = e;
			break;
		}

		// Face 2 merge
		float d = qh_distance(face1->center, face2->plane);
		if (d < -tolerance)
		{
			// Edge is convex wrt to the face 2
			e = e->next;
			continue;
		}

		// Edge is concave or coplanar wrt to the face 2
		edge = e;
		break;

	} while (e != face1->edge);

	if (edge)
	{
		qh_hull_remove_edge(hull, edge);
		return 1;
	}

	return 0;
}

void qh_hull_merge_new_faces(qh_hull_t* hull)
{
	// Merge with respect to the largest face.
	for (int i = 0; i < hull->new_face_count; ++i)
	{
		qh_face_t* face = hull->new_face_buffer[i];

		// Was the face deleted due to merging?
		if (face->is_active == 0)
		{
			continue;
		}

		while (qh_hull_merge_large_face(hull, face));
	}

	// Merge with respect to the both faces.
	for (int i = 0; i < hull->new_face_count; ++i)
	{
		qh_face_t* face = hull->new_face_buffer[i];

		// Was the face deleted due to merging?
		if (face->is_active == 0)
		{
			continue;
		}

		while (qh_hull_merge_face(hull, face));
	}
}

void qh_hull_translate(qh_hull_t* hull, qh_vec3_t translation)
{
	// Shift vertices
	for (qh_vertex_t* v = hull->vertex_head; v != NULL; v = v->next)
	{
		v->position = qh_vec3_add(v->position, translation);
	}

	// Reset face data
	for (qh_face_t* f = hull->face_head; f != NULL; f = f->next)
	{
		qh_face_reset_face_data(f);
	}
}

void qh_hull_validate_convexity(const qh_hull_t* hull)
{
	float tolerance = hull->coplanarity_tolerance;

	for (qh_face_t* face = hull->face_head; face != NULL; face = face->next)
	{
		assert(face->is_active == 1);

		const qh_half_edge_t* edge = face->edge;
		do
		{
			assert(edge->is_active == 1);
			assert(edge->face == face);

			assert(edge->twin != NULL);
			assert(edge->twin->is_active == 1);
			qh_face_t* other = edge->twin->face;

			// Ensure closed volume
			assert(other != NULL);
			assert(other->is_active == 1);

			// Ensure topological health
			assert(face != other);

			// Ensure edge convexity
			float d1 = qh_distance(other->center, face->plane);
			assert(d1 < -tolerance);

			float d2 = qh_distance(face->center, other->plane);
			assert(d2 < -tolerance);

			// Ensure polygon convexity
			qh_vec3_t P = edge->tail->position;
			qh_vec3_t Q = edge->twin->tail->position;

			qh_vec3_t E = qh_vec3_sub(Q, P);
			qh_vec3_t D = qh_vec3_cross(E, face->plane.n);

			// Edge side plane
			qh_plane_t plane;
			plane.n = qh_vec3_normalize(D);
			plane.d = qh_vec3_dot(plane.n, P);

			// All the other vertices must be behind the edge side plane
			const qh_half_edge_t* eother = edge->prev;
			do
			{
				float d = qh_distance(eother->tail->position, plane);
				assert(d <= 0.0f);

				eother = eother->prev;
			} while (eother != edge->next);

			edge = edge->next;
		} while (edge != face->edge);
	}
}

qh_vertex_t* qh_hull_find_eye_vertex(qh_hull_t* hull)
{
	// Find the furthest conflict point.
	float d0 = hull->coplanarity_tolerance;
	qh_vertex_t* v0 = NULL;

	for (qh_face_t* f = hull->face_head; f != NULL; f = f->next)
	{
		for (qh_vertex_t* v = f->conflict_vertex_head; v != NULL; v = v->next)
		{
			float d = qh_distance(v->position, f->plane);
			if (d > d0)
			{
				d0 = d;
				v0 = v;
			}
		}
	}
	return v0;
}

void qh_hull_add_eye_vertex(qh_hull_t* hull, qh_vertex_t* eye)
{
	qh_hull_find_horizon(hull, eye);
	qh_hull_add_new_faces(hull, eye);
	qh_hull_merge_new_faces(hull);
	qh_hull_resolve_orphans(hull);
}

void qh_hull_destroy(qh_hull_t* hull)
{
	qh_vertex_t* v = hull->vertex_head;
	while (v)
	{
		qh_vertex_t* v0 = v;
		v = v->next;

		qh_hull_free_vertex(hull, v0);
	}

	assert(hull->free_vertex_count == 0);

	qh_face_t* f = hull->face_head;
	while (f)
	{
		qh_face_t* f0 = f;
		f = f->next;

		qh_half_edge_t* e = f0->edge;
		do
		{
			qh_half_edge_t* e0 = e;
			e = e->next;

			qh_hull_free_edge(hull, e0);
		} while (e != f0->edge);

		qh_hull_free_face(hull, f0);
	}

	assert(hull->free_edge_count == 0);
	assert(hull->free_face_count == 0);
	qh_free(hull->memory);
	qh_free(hull);
}

qh_hull_t* qh_hull_create(int vertex_stride, const void* vertex_base, int vertex_count)
{
	assert(vertex_stride >= sizeof(qh_vec3_t));
	assert(vertex_count >= 4);

	qh_hull_t* hull = (qh_hull_t*)qh_alloc(sizeof(qh_hull_t));

	// Euler's formula
	int V = vertex_count;
	int E = 3 * V - 6;
	int HE = 2 * E;
	int F = 2 * V - 4;

	// Compute memory buffer size for the worst case.
	int size = 0;

	size += V * sizeof(qh_vertex_t);
	size += HE * sizeof(qh_half_edge_t);
	size += F * sizeof(qh_face_t);

	// Horizon 
	size += HE * sizeof(qh_half_edge_t*);

	// Saved horizon vertices
	// One vertex per horizon edge
	size += HE * sizeof(qh_vertex_t*);

	// Saved conflict vertices
	size += V * sizeof(qh_vertex_t*);

	// New Faces
	// One face per horizon edge
	size += HE * sizeof(qh_face_t*);

	// Allocate memory buffer 
	hull->memory = qh_alloc(size);

	// Initialize free lists
	hull->free_vertex_capacity = V;
	hull->free_vertex_count = hull->free_vertex_capacity;
	hull->free_vertices = NULL;
	qh_vertex_t* vertices = (qh_vertex_t*)hull->memory;
	for (int i = 0; i < V; ++i)
	{
		vertices[i].is_active = 1;
		qh_hull_free_vertex(hull, vertices + i);
	}
	assert(hull->free_vertex_count == 0);

	hull->free_edge_capacity = HE;
	hull->free_edge_count = hull->free_edge_capacity;
	hull->free_edges = NULL;
	qh_half_edge_t* edges = (qh_half_edge_t*)((char*)vertices + V * sizeof(qh_vertex_t));
	for (int i = 0; i < HE; ++i)
	{
		edges[i].is_active = 1;
		qh_hull_free_edge(hull, edges + i);
	}
	assert(hull->free_edge_count == 0);

	hull->free_face_capacity = F;
	hull->free_face_count = hull->free_face_capacity;
	hull->free_faces = NULL;
	qh_face_t* faces = (qh_face_t*)((char*)edges + HE * sizeof(qh_half_edge_t));
	for (int i = 0; i < F; ++i)
	{
		qh_face_t* f = faces + i;
		f->conflict_vertex_head = NULL;
		f->conflict_vertex_count = 0;
		f->is_active = 1;
		qh_hull_free_face(hull, f);
	}
	assert(hull->free_face_count == 0);

	hull->horizon_edge_buffer = (qh_half_edge_t**)((char*)faces + F * sizeof(qh_face_t));
	hull->horizon_edge_count = 0;

	hull->horizon_vertex_buffer = (qh_vertex_t**)((char*)hull->horizon_edge_buffer + HE * sizeof(qh_half_edge_t*));

	hull->conflict_vertex_buffer = (qh_vertex_t**)((char*)hull->horizon_vertex_buffer + HE * sizeof(qh_vertex_t*));
	hull->conflict_vertex_count = 0;

	hull->new_face_buffer = (qh_face_t**)((char*)hull->conflict_vertex_buffer + V * sizeof(qh_vertex_t*));
	hull->new_face_count = 0;

	hull->vertex_head = NULL;
	hull->vertex_count = 0;

	hull->face_head = NULL;
	hull->face_count = 0;

	// Build initial tetrahedron
	if (!qh_build_initial_hull(hull, vertex_stride, vertex_base, vertex_count))
	{
		qh_hull_destroy(hull);
		return NULL;
	}

	int iterations = 0;

	// Run Quickhull
	qh_vertex_t* eye = qh_hull_find_eye_vertex(hull);
	while (eye)
	{
		qh_hull_validate(hull);
		qh_hull_validate_convexity(hull);

		qh_hull_add_eye_vertex(hull, eye);

		eye = qh_hull_find_eye_vertex(hull);

		++iterations;
	}

	qh_hull_validate(hull);
	qh_hull_validate_convexity(hull);

	hull->iterations = iterations;

	return hull;
}

typedef struct qh_pointer_array
{
	int capacity;
	int count;
	void** pointers;
} qh_pointer_array;

int qh_pointer_array_insert_pointer(qh_pointer_array* a, void* p)
{
	for (int i = 0; i < a->count; ++i)
	{
		if (a->pointers[i] == p)
		{
			return i;
		}
	}
	assert(a->count < a->capacity);
	a->pointers[a->count] = p;
	return a->count++;
}

int qh_pointer_array_get_pointer(qh_pointer_array* a, void* p)
{
	for (int i = 0; i < a->count; ++i)
	{
		if (a->pointers[i] == p)
		{
			return i;
		}
	}
	assert(0);
	return -1;
}

qh_vec3_t qh_compute_centroid(const qh_output_t* hull)
{
	// M. Kallay - "Computing the Moment of Inertia of a Solid Defined by a Triangle Mesh"
	assert(hull->vertex_count >= 4);

	float volume = 0.0f;
	qh_vec3_t centroid = qh_vec3_zero();

	// Put the reference point inside the hull
	qh_vec3_t center = qh_vec3_zero();
	for (int i = 0; i < hull->vertex_count; ++i)
	{
		center = qh_vec3_add(center, hull->vertices[i]);
	}
	center = qh_vec3_div(center, (float)hull->vertex_count);

	for (int i = 0; i < hull->face_count; ++i)
	{
		const qh_output_face_t* face = hull->faces + i;
		const qh_output_half_edge_t* begin = hull->edges + face->edge;

		const qh_output_half_edge_t* edge = hull->edges + begin->next;
		do
		{
			const qh_output_half_edge_t* next = hull->edges + edge->next;

			int i1 = begin->origin;
			int i2 = edge->origin;
			int i3 = next->origin;

			qh_vec3_t v1 = qh_vec3_sub(hull->vertices[i1], center);
			qh_vec3_t v2 = qh_vec3_sub(hull->vertices[i2], center);
			qh_vec3_t v3 = qh_vec3_sub(hull->vertices[i3], center);

			// Signed tetrahedron volume
			float D = qh_vec3_dot(v1, qh_vec3_cross(v2, v3));

			// Contribution to the mass
			volume += D;

			// Contribution to the centroid
			qh_vec3_t v4 = qh_vec3_add(qh_vec3_add(v1, v2), v3);

			centroid = qh_vec3_add(centroid, qh_vec3_mul(D, v4));

			edge = next;
		} while ((hull->edges + edge->next) != begin);
	}

	// Centroid
	assert(volume > 0.0f);
	centroid = qh_vec3_div(centroid, 4.0f * volume);
	centroid = qh_vec3_add(centroid, center);
	return centroid;
}

qh_output_t* qh_output_create(const qh_hull_t* hull)
{
	int vertex_count = hull->vertex_count;
	qh_vec3_t* vertices = (qh_vec3_t*)qh_alloc(vertex_count * sizeof(qh_vec3_t));

	qh_pointer_array vertex_pointers;
	vertex_pointers.capacity = hull->vertex_count;
	vertex_pointers.pointers = (void**)qh_alloc(vertex_pointers.capacity * sizeof(void*));
	vertex_pointers.count = 0;
	for (qh_vertex_t* vertex = hull->vertex_head; vertex != NULL; vertex = vertex->next)
	{
		vertices[vertex_pointers.count] = vertex->position;
		vertex_pointers.pointers[vertex_pointers.count] = vertex;
		++vertex_pointers.count;
	}

	qh_pointer_array edge_pointers;
	edge_pointers.capacity = 0;
	for (qh_face_t* face = hull->face_head; face != NULL; face = face->next)
	{
		qh_half_edge_t* edge = face->edge;
		do
		{
			++edge_pointers.capacity;
			edge = edge->next;
		} while (edge != face->edge);
	}
	edge_pointers.pointers = (void**)qh_alloc(edge_pointers.capacity * sizeof(void*));
	edge_pointers.count = 0;

	for (qh_face_t* face = hull->face_head; face != NULL; face = face->next)
	{
		qh_half_edge_t* edge = face->edge;
		do
		{
			qh_pointer_array_insert_pointer(&edge_pointers, edge);

			// Twin edge just after its twin
			qh_pointer_array_insert_pointer(&edge_pointers, edge->twin);

			edge = edge->next;
		} while (edge != face->edge);
	}

	int edge_count = edge_pointers.count;
	qh_output_half_edge_t* edges = (qh_output_half_edge_t*)qh_alloc(edge_count * sizeof(qh_output_half_edge_t));
	int face_count = hull->face_count;
	qh_output_face_t* faces = (qh_output_face_t*)qh_alloc(face_count * sizeof(qh_output_face_t));
	qh_plane_t* planes = (qh_plane_t*)qh_alloc(face_count * sizeof(qh_plane_t));

	int iface = 0;
	for (qh_face_t* face = hull->face_head; face != NULL; face = face->next)
	{
		planes[iface] = face->plane;

		qh_half_edge_t* begin = face->edge;

		qh_output_face_t* hface = faces + iface;
		hface->edge = qh_pointer_array_get_pointer(&edge_pointers, begin);

		qh_half_edge_t* edge = begin;
		do
		{
			qh_vertex_t* origin = edge->tail;
			int iorigin = qh_pointer_array_get_pointer(&vertex_pointers, origin);

			int iedge = qh_pointer_array_get_pointer(&edge_pointers, edge);
			qh_output_half_edge_t* hedge = edges + iedge;
			hedge->face = iface;
			hedge->origin = iorigin;

			qh_half_edge_t* twin = edge->twin;
			int itwin = qh_pointer_array_get_pointer(&edge_pointers, twin);
			qh_output_half_edge_t* htwin = edges + itwin;
			htwin->twin = iedge;

			hedge->twin = itwin;

			qh_half_edge_t* next = edge->next;
			int inext = qh_pointer_array_get_pointer(&edge_pointers, next);

			qh_half_edge_t* prev = edge->prev;
			int iprev = qh_pointer_array_get_pointer(&edge_pointers, prev);

			edges[iedge].prev = iprev;
			edges[iedge].next = inext;

			edge = next;
		} while (edge != begin);

		++iface;
	}

	qh_free(vertex_pointers.pointers);
	qh_free(edge_pointers.pointers);

	qh_output_t* output = (qh_output_t*)qh_alloc(sizeof(qh_output_t));
	output->vertex_count = vertex_count;
	output->vertices = vertices;
	output->edge_count = edge_count;
	output->edges = edges;
	output->face_count = face_count;
	output->faces = faces;
	output->planes = planes;
	output->centroid = qh_compute_centroid(output);
	return output;
}

qh_output_t* qh_create_hull(int vertex_stride, const void* vertex_base, int vertex_count, float coincidence_tol)
{
	// Copy vertices into local buffer, perform welding.
	int weld_vertex_count = 0;
	qh_vec3_t* weld_vertices = (qh_vec3_t*)qh_alloc(vertex_count * sizeof(qh_vec3_t));
	for (int i = 0; i < vertex_count; ++i)
	{
		qh_vec3_t v = qh_vec3_array_make(vertex_stride, vertex_base, i);

		int unique = 1;
		for (int j = 0; j < weld_vertex_count; ++j)
		{
			if (qh_vec3_distance_squared(v, weld_vertices[j]) <= coincidence_tol * coincidence_tol)
			{
				unique = 0;
				break;
			}
		}

		if (unique)
		{
			weld_vertices[weld_vertex_count++] = v;
		}
	}

	if (weld_vertex_count < 4)
	{
		// Polyhedron is degenerate.
		qh_free(weld_vertices);
		return NULL;
	}

	// Construct the convex hull.
	qh_hull_t* hull = qh_hull_create(sizeof(qh_vec3_t), weld_vertices, weld_vertex_count);

	qh_free(weld_vertices);

	if (!hull)
	{
		return NULL;
	}

	qh_output_t* output = qh_output_create(hull);

	qh_hull_destroy(hull);

	return output;
}

void qh_destroy_hull(qh_output_t* hull)
{
	qh_free(hull->vertices);
	qh_free(hull->edges);
	qh_free(hull->faces);
	qh_free(hull->planes);
	qh_free(hull);
}

qh_output_t* qh_create_simplified_hull(int vertex_stride, const void* vertex_base, int vertex_count, float coincidence_tol, float angle_tol)
{
	// Copy vertices into local buffer, perform welding.
	int weld_count = 0;
	qh_vec3_t* weld_vertices = (qh_vec3_t*)qh_alloc(vertex_count * sizeof(qh_vec3_t));
	for (int i = 0; i < vertex_count; ++i)
	{
		qh_vec3_t v = qh_vec3_array_make(vertex_stride, vertex_base, i);

		int unique = 1;
		for (int j = 0; j < weld_count; ++j)
		{
			if (qh_vec3_distance_squared(v, weld_vertices[j]) <= coincidence_tol * coincidence_tol)
			{
				unique = 0;
				break;
			}
		}

		if (unique)
		{
			weld_vertices[weld_count++] = v;
		}
	}

	if (weld_count < 4)
	{
		// Polyhedron is degenerate.
		qh_free(weld_vertices);
		return NULL;
	}

	// Construct the hull in primary space.
	qh_hull_t* primary = qh_hull_create(sizeof(qh_vec3_t), weld_vertices, weld_count);

	qh_free(weld_vertices);

	if (!primary)
	{
		return NULL;
	}

	// Simplify the constructed hull.
	qh_vec3_t primary_center = qh_vec3_zero();
	for (qh_vertex_t* v = primary->vertex_head; v; v = v->next)
	{
		primary_center = qh_vec3_add(primary_center, v->position);
	}
	primary_center = qh_vec3_div(primary_center, (float)primary->vertex_count);

	// Put the origin inside the hull.
	qh_hull_translate(primary, qh_vec3_negate(primary_center));

	int dual_vertex_count = 0;
	qh_vec3_t* dual_vertices = (qh_vec3_t*)qh_alloc(primary->face_count * sizeof(qh_vec3_t));
	qh_face_t** dual_faces = (qh_face_t**)qh_alloc(primary->face_count * sizeof(qh_face_t*));

	for (qh_face_t* face = primary->face_head; face != NULL; face = face->next)
	{
		qh_plane_t plane = face->plane;
		assert(plane.d > 0.0f);
		qh_vec3_t dual_vertex = qh_vec3_div(plane.n, plane.d);

		int unique = 1;
		for (int j = 0; j < dual_vertex_count; ++j)
		{
			qh_face_t* existing_face = dual_faces[j];

			qh_vec3_t v1 = plane.n;
			qh_vec3_t v2 = existing_face->plane.n;

			float x = qh_vec3_dot(v1, v2);

			qh_vec3_t e = qh_vec3_cross(v1, v2);
			float y = sinf(qh_vec3_length(e));

			float angle = atan2f(y, x);

			if (angle < angle_tol)
			{
				if (face->area > existing_face->area)
				{
					// Keep largest face.
					dual_faces[j] = face;
					dual_vertices[j] = dual_vertex;
				}

				unique = 0;
				break;
			}
		}

		if (unique)
		{
			dual_faces[dual_vertex_count] = face;
			dual_vertices[dual_vertex_count] = dual_vertex;
			++dual_vertex_count;
		}
	}

	qh_free(dual_faces);

	qh_hull_destroy(primary);

	if (dual_vertex_count < 4)
	{
		qh_free(dual_vertices);
		return NULL;
	}

	// Construct the hull in dual space.
	qh_hull_t* dual = qh_hull_create(sizeof(qh_vec3_t), dual_vertices, dual_vertex_count);

	qh_free(dual_vertices);

	if (!dual)
	{
		return NULL;
	}

	// Recover the simplified hull in primary space. 
	qh_vec3_t* primary_vertices = (qh_vec3_t*)qh_alloc(dual->face_count * sizeof(qh_vec3_t));
	int primary_vertex_count = 0;
	for (qh_face_t* face = dual->face_head; face != NULL; face = face->next)
	{
		qh_plane_t plane = face->plane;
		assert(plane.d > 0.0f);
		qh_vec3_t primary_vertex = qh_vec3_div(plane.n, plane.d);

		int unique = 1;
		for (int j = 0; j < primary_vertex_count; ++j)
		{
			if (qh_vec3_distance_squared(primary_vertex, primary_vertices[j]) <= coincidence_tol * coincidence_tol)
			{
				unique = 0;
				break;
			}
		}

		if (unique)
		{
			primary_vertices[primary_vertex_count++] = primary_vertex;
		}
	}

	qh_hull_destroy(dual);

	// Simplified primary hull relative to center.
	qh_hull_t* recovered_primary = qh_hull_create(sizeof(qh_vec3_t), primary_vertices, primary_vertex_count);

	qh_free(primary_vertices);

	if (!recovered_primary)
	{
		return NULL;
	}

	// Translate the primary hull back to the origin
	qh_hull_translate(recovered_primary, primary_center);

	// Prepare output.
	qh_output_t* output = qh_output_create(recovered_primary);

	qh_hull_destroy(recovered_primary);

	// Done.
	return output;
}
