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

#include <bounce/collision/trees/static_tree.h>
#include <bounce/common/draw.h>
#include <algorithm>

b3StaticTree::b3StaticTree()
{
	m_root = B3_NULL_STATIC_NODE;
	m_nodes = nullptr;
	m_nodeCount = 0;
}

b3StaticTree::~b3StaticTree()
{
	b3Free(m_nodes);
}

struct b3SortPredicate
{
	bool operator()(uint32 a, uint32 b)
	{
		b3Vec3 cA = aabbs[a].GetCenter();
		b3Vec3 cB = aabbs[b].GetCenter();
		
		return cA[axis] < cB[axis];
	}

	uint32 axis;
	const b3AABB* aabbs;
};

static uint32 b3Partition(const b3AABB& setAABB, uint32* indices, uint32 count, const b3AABB* aabbs)
{
	// Choose a partitioning axis.
	uint32 splitAxis = setAABB.GetLongestAxisIndex();

	// Choose a split point.
	scalar splitPos = setAABB.GetCenter()[splitAxis];

	// Sort along the split axis.
	b3SortPredicate predicate;
	predicate.aabbs = aabbs;
	predicate.axis = splitAxis;

	std::sort(indices, indices + count, predicate);

	// Find the AABB that splits the set in two subsets.
	uint32 left = 0;
	uint32 right = count - 1;
	uint32 middle = left;
	while (middle < right)
	{
		b3Vec3 center = aabbs[indices[middle]].GetCenter();
		if (center[splitAxis] > splitPos)
		{
			// Found median.
			break;
		}
		++middle;
	}

	// Ensure nonempty subsets.
	uint32 count1 = middle;
	uint32 count2 = count - middle;
	if (count1 == 0 || count2 == 0)
	{
		// Choose median.
		middle = (left + right) / 2;
	}

	return middle;
}

void b3StaticTree::BuildRecursively(b3StaticNode* node, uint32* indices, uint32 count, const b3AABB* aabbs, uint32 nodeCapacity, uint32& leafCount, uint32& internalCount)
{
	B3_ASSERT(count > 0);
	
	// Enclose set
	b3AABB setAABB = aabbs[indices[0]];
	for (uint32 i = 1; i < count; ++i)
	{
		setAABB = b3Combine(setAABB, aabbs[indices[i]]);
	}

	node->aabb = setAABB;

	if (count <= 1)
	{
		++leafCount;
		node->child1 = B3_NULL_STATIC_NODE;
		node->index = indices[0];
	}
	else
	{
		++internalCount;

		// Partition current set
		uint32 middle = b3Partition(setAABB, indices, count, aabbs);

		// Allocate left subtree
		B3_ASSERT(m_nodeCount < nodeCapacity);
		node->child1 = m_nodeCount;
		++m_nodeCount;

		// Allocate right subtree
		B3_ASSERT(m_nodeCount < nodeCapacity);
		node->child2 = m_nodeCount;
		++m_nodeCount;

		// Build left and right subtrees
		BuildRecursively(m_nodes + node->child1, indices, middle, aabbs, nodeCapacity, leafCount, internalCount);
		BuildRecursively(m_nodes + node->child2, indices + middle, count - middle, aabbs, nodeCapacity, leafCount, internalCount);
	}
}

void b3StaticTree::Build(const b3AABB* aabbs, uint32 count)
{
	B3_ASSERT(m_nodes == nullptr && m_nodeCount == 0);
	B3_ASSERT(count > 0);

	// Leafs = n, Internals = n - 1, Total = 2n - 1, if we assume
	// each leaf node contains exactly 1 object.
	uint32 internalCapacity = count - 1;
	uint32 leafCapacity = count;
	uint32 nodeCapacity = 2 * count - 1;
	
	m_root = 0;
	m_nodes = (b3StaticNode*)b3Alloc(nodeCapacity * sizeof(b3StaticNode));
	m_nodeCount = 1;

	uint32* indices = (uint32*)b3Alloc(count * sizeof(uint32));
	for (uint32 i = 0; i < count; ++i)
	{
		indices[i] = i;
	}

	uint32 leafCount = 0;
	uint32 internalCount = 0;
	BuildRecursively(m_nodes, indices, count, aabbs, nodeCapacity, leafCount, internalCount);

	b3Free(indices);

	B3_ASSERT(leafCount == leafCapacity);
	B3_ASSERT(internalCount == internalCapacity);
	B3_ASSERT(m_nodeCount == nodeCapacity);
}

void b3StaticTree::Draw(b3Draw* draw) const
{
	if (m_nodeCount == 0)
	{
		return;
	}

	b3Stack<uint32, 256> stack;
	stack.Push(m_root);

	while (!stack.IsEmpty())
	{
		uint32 nodeIndex = stack.Top();

		stack.Pop();

		const b3StaticNode* node = m_nodes + nodeIndex;
		if (node->IsLeaf())
		{
			draw->DrawAABB(node->aabb, b3Color_red);
		}
		else
		{
			draw->DrawAABB(node->aabb, b3Color_green);
			
			stack.Push(node->child1);
			stack.Push(node->child2);
		}
	}
}
