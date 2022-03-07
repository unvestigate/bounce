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

#include "test.h"

extern "C"
{
#include <quickhull/quickhull.h>
}

extern u32 b3_allocCalls, b3_maxAllocCalls;
extern u32 b3_convexCalls, b3_convexCacheHits;
extern u32 b3_gjkCalls, b3_gjkIters, b3_gjkMaxIters;
extern bool b3_convexCache;

float RandomFloat(float a, float b)
{
	float r = float(rand()) / float(RAND_MAX);
	float d = b - a;
	return a + r * d;
}

b3Hull* CreateHull(const b3Vec3* vertices, int vertexCount)
{
	qh_vec3_t* float_vertices = (qh_vec3_t*)malloc(vertexCount * sizeof(qh_vec3_t));
	for (int i = 0; i < vertexCount; ++i)
	{
		float_vertices[i].x = (float)vertices[i].x;
		float_vertices[i].y = (float)vertices[i].y;
		float_vertices[i].z = (float)vertices[i].z;
	}

	qh_output_t* output = qh_create_simplified_hull(sizeof(qh_vec3_t), float_vertices, vertexCount, B3_LINEAR_SLOP, 0.25f * B3_PI);
	if (output == NULL)
	{
		return NULL;
	}

	free(float_vertices);

	b3Hull* hull = (b3Hull*)malloc(sizeof(b3Hull));
	hull->vertexCount = output->vertex_count;
	hull->vertices = (b3Vec3*)malloc(output->vertex_count * sizeof(b3Vec3));
	for (int i = 0; i < output->vertex_count; ++i)
	{
		hull->vertices[i].x = output->vertices[i].x;
		hull->vertices[i].y = output->vertices[i].y;
		hull->vertices[i].z = output->vertices[i].z;
	}
	hull->edgeCount = output->edge_count;
	hull->edges = (b3HalfEdge*)malloc(output->edge_count * sizeof(b3HalfEdge));
	for (int i = 0; i < output->edge_count; ++i)
	{
		hull->edges[i].origin = output->edges[i].origin;
		hull->edges[i].twin = output->edges[i].twin;
		hull->edges[i].face = output->edges[i].face;
		hull->edges[i].prev = output->edges[i].prev;
		hull->edges[i].next = output->edges[i].next;
	}
	hull->faceCount = output->face_count;
	hull->faces = (b3Face*)malloc(output->face_count * sizeof(b3Face));
	hull->planes = (b3Plane*)malloc(output->face_count * sizeof(b3Plane));
	for (int i = 0; i < output->face_count; ++i)
	{
		hull->faces[i].edge = output->faces[i].edge;
		hull->planes[i].normal.x = output->planes[i].n.x;
		hull->planes[i].normal.y = output->planes[i].n.y;
		hull->planes[i].normal.z = output->planes[i].n.z;
		hull->planes[i].offset = output->planes[i].d;
	}

	hull->centroid.x = output->centroid.x;
	hull->centroid.y = output->centroid.y;
	hull->centroid.z = output->centroid.z;

	qh_destroy_hull(output);

	hull->Validate();

	return hull;
}

void DestroyHull(b3Hull* hull)
{
	free(hull->vertices);
	free(hull->edges);
	free(hull->faces);
	free(hull->planes);
	free(hull);
}

Test::Test() : 
	m_bodyDragger(&m_ray, &m_world)
{
	m_draw.m_debugDrawData = g_debugDrawData;
	
	m_world.SetContactListener(this);
	m_world.SetProfiler(g_profiler);
	m_world.SetDebugDraw(&m_draw);

	m_ray.origin.SetZero();
	m_ray.direction.Set(0.0f, 0.0f, -1.0f);
	m_ray.fraction = 1000.0f;

	m_groundHull.SetExtents(50.0f, 1.0f, 50.0f);
	
	m_groundMesh.BuildTree();
	m_groundMesh.BuildAdjacency();

	b3_convexCache = g_testSettings->convexCache;
}

void Test::Step()
{
	b3_convexCache = g_testSettings->convexCache;

	// Step
	m_world.SetSleeping(g_testSettings->sleep);
	m_world.SetWarmStarting(g_testSettings->warmStart);
	m_world.Step(g_testSettings->inv_hertz, g_testSettings->velocityIterations, g_testSettings->positionIterations);

	// Draw
	u32 drawFlags = 0;
	drawFlags += g_testSettings->drawBounds * b3World::e_aabbsFlag;
	drawFlags += g_testSettings->drawShapes * b3World::e_shapesFlag;
	drawFlags += g_testSettings->drawCenterOfMasses * b3World::e_centerOfMassesFlag;
	drawFlags += g_testSettings->drawJoints * b3World::e_jointsFlag;
	drawFlags += g_testSettings->drawContactPoints * b3World::e_contactPointsFlag;
	drawFlags += g_testSettings->drawContactNormals * b3World::e_contactNormalsFlag;
	drawFlags += g_testSettings->drawContactTangents * b3World::e_contactTangentsFlag;
	drawFlags += g_testSettings->drawContactPolygons * b3World::e_contactPolygonsFlag;

	m_world.SetDrawFlags(drawFlags);
	m_world.Draw();
	m_world.DrawSolid();

	if (g_testSettings->pause)
	{
		DrawString(b3Color_white, "*PAUSED*");
	}
	else
	{
		DrawString(b3Color_white, "*PLAYING*");
	}

	if (g_settings->drawStats)
	{
		DrawString(b3Color_white, "Bodies %d", m_world.GetBodyList().m_count);
		DrawString(b3Color_white, "Joints %d", m_world.GetJointList().m_count);
		DrawString(b3Color_white, "Contacts %d", m_world.GetContactList().m_count);

		scalar avgGjkIters = 0.0f;
		if (b3_gjkCalls > 0)
		{
			avgGjkIters = scalar(b3_gjkIters) / scalar(b3_gjkCalls);
		}

		DrawString(b3Color_white, "GJK Calls %d", b3_gjkCalls);
		DrawString(b3Color_white, "GJK Iterations %d (%d) (%f)", b3_gjkIters, b3_gjkMaxIters, avgGjkIters);

		scalar convexCacheHitRatio = 0.0f;
		if (b3_convexCalls > 0)
		{
			convexCacheHitRatio = scalar(b3_convexCacheHits) / scalar(b3_convexCalls);
		}

		DrawString(b3Color_white, "Convex Calls %d", b3_convexCalls);
		DrawString(b3Color_white, "Convex Cache Hits %d (%f)", b3_convexCacheHits, convexCacheHitRatio);
		DrawString(b3Color_white, "Frame Allocations %d (%d)", b3_allocCalls, b3_maxAllocCalls);
	}
}

void Test::MouseMove(const b3Ray& pw)
{
	m_ray = pw;

	if (m_bodyDragger.IsDragging() == true)
	{
		m_bodyDragger.Drag();
	}
}

void Test::MouseLeftDown(const b3Ray& pw)
{
	if (m_bodyDragger.IsDragging() == false)
	{
		if (m_bodyDragger.StartDragging() == true)
		{
			BeginDragging();
		}
	}
}

void Test::MouseLeftUp(const b3Ray& pw)
{
	if (m_bodyDragger.IsDragging() == true)
	{
		m_bodyDragger.StopDragging();

		EndDragging();
	}
}
