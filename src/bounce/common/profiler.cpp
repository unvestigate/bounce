/*
* Copyright (c) 2016-2019 Irlan Robson https://irlanrobson.github.io
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

#include <bounce/common/profiler.h>
#include <bounce/common/math/math.h>

b3ProfilerNode* b3ProfilerNode::FindChildNode(const char* name) 
{
	for (b3ProfilerNode* c = m_childList; c != nullptr; c = c->m_childNext)
	{
		if (c->m_name == name)
		{
			return c;
		}
	}
	return nullptr;
}

b3Profiler::b3Profiler() : 
	m_nodePool(sizeof(b3ProfilerNode)),
	m_statsPool(sizeof(b3ProfilerStat))
{
	m_root = nullptr;
	m_top = nullptr;
	m_statList = nullptr;
	m_statCount = 0;
}

void b3Profiler::Clear()
{
	B3_ASSERT(m_top == nullptr);
	if (m_root)
	{
		DestroyNodeRecursively(m_root);
		m_root = nullptr;
	}
}

void b3Profiler::OpenScope(const char* name)
{
	if (m_top)
	{
		b3ProfilerNode* topChild = m_top->FindChildNode(name);
		if (topChild)
		{
			// Top child node becomes top node
			m_top = topChild;

			++topChild->m_callCount;

			if (topChild->m_recursionCount == 0)
			{
				m_time.Update();
				topChild->m_t1 = m_time.GetCurrentMilis();
			}

			++topChild->m_recursionCount;

			return;
		}
	}

	m_time.Update();
	
	// Create a new node
	b3ProfilerNode* newNode = (b3ProfilerNode*)m_nodePool.Allocate();
	newNode->m_name = name;
	newNode->m_t1 = m_time.GetCurrentMilis();
	newNode->m_elapsed = 0.0f;
	newNode->m_callCount = 1;
	newNode->m_recursionCount = 1;
	newNode->m_stat = nullptr;
	newNode->m_parent = m_top;
	newNode->m_childList = nullptr;
	newNode->m_childNext = nullptr;
	newNode->m_childCount = 0;

	if (m_root == nullptr)
	{
		// Insert into tree
		B3_ASSERT(m_top == nullptr);
		m_root = newNode;
		m_top = newNode;
		return;
	}

	if (m_top)
	{
		// Top node gets a new kid
		newNode->m_childNext = m_top->m_childList;
		m_top->m_childList = newNode;
		++m_top->m_childCount;
	}

	// New node becomes top node
	m_top = newNode;
}

void b3Profiler::CloseScope()
{
	B3_ASSERT(m_top != nullptr);

	--m_top->m_recursionCount;
	if (m_top->m_recursionCount > 0)
	{
		return;
	}

	// Update top elapsed time
	m_time.Update();
	m_top->m_t2 = m_time.GetCurrentMilis();

	double elapsed = m_top->m_t2 - m_top->m_t1;

	m_top->m_elapsed += elapsed;

	b3ProfilerStat* topStat = FindStat(m_top->m_name);
	if (topStat == nullptr)
	{
		// Create a new stats
		topStat = (b3ProfilerStat*)m_statsPool.Allocate();
		topStat->m_name = m_top->m_name;
		topStat->m_minElapsed = elapsed;
		topStat->m_maxElapsed = elapsed;

		// Push stat to profiler list of stats
		topStat->m_next = m_statList;
		m_statList = topStat;
		++m_statCount;
	}
	else
	{
		// Update top stats
		topStat->m_minElapsed = b3Min(topStat->m_minElapsed, elapsed);
		topStat->m_maxElapsed = b3Max(topStat->m_maxElapsed, elapsed);
	}

	// Set top stat
	m_top->m_stat = topStat;
	
	// Top parent becomes top node
	m_top = m_top->m_parent;
}

b3ProfilerStat* b3Profiler::FindStat(const char* name)
{
	for (b3ProfilerStat* s = m_statList; s != nullptr; s = s->m_next)
	{
		if (s->m_name == name)
		{
			return s;
		}
	}
	return nullptr;
}

void b3Profiler::DestroyNodeRecursively(b3ProfilerNode* node)
{
	b3ProfilerNode* c = node->m_childList;
	while (c)
	{
		b3ProfilerNode* boom = c;
		c = c->m_childNext;
		DestroyNodeRecursively(boom);
	}
	node->~b3ProfilerNode();
	m_nodePool.Free(node);
}
