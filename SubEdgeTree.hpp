#pragma once
#include "Types.hpp"
#include <cstdint>
#include<cassert>
#include <vector>
#include <algorithm>
#include <utility>

// The subEdge tree data structure, memory layout of a 'node' is |int32 - parent|float - left end|he - left sub|he - right sub| So each node takes 4x4 = 16 bytes

// Lookups can be done in a tree like fashion. SubEdges are stored from low coordinate -> high coordinate
class SubEdgeTree
{
public:
	uint32_t findSubEdge(halfEdge start, halfEdge opposite, float splitpoint) const;
	uint32_t getsubEdges(halfEdge start, std::vector<halfEdge>& subEdges) const;
	inline halfEdge splitHalfEdge(halfEdge twin, halfEdge Edge, halfEdge original, halfEdge left_child, halfEdge right_child, float splitpoint);
	inline void removeNode(halfEdge hf, std::vector<halfEdge>& E2e);
	inline void updateParent(halfEdge tree_head, halfEdge new_parent);
	inline std::pair<halfEdge, halfEdge> subdivideTree(halfEdge tree_head, float splitpoint, std::vector<halfEdge>& E2e);
	// Iterator like functions
	inline uint32_t next(uint32_t index) const;
	inline uint32_t begin(uint32_t head) const;
	static inline uint32_t end() { return -1; };
	static inline uint32_t toIndex(const halfEdge key) { assert(key.isSubdivided()); return (key.id & 0x7fffffff); }
	halfEdge& operator[](const halfEdge index) { return _storage[toIndex(index)]; };
	halfEdge& operator[](const uint32_t index) { return _storage[index]; };
	const halfEdge& operator[](const halfEdge index) const { return _storage[toIndex(index)]; };
	const halfEdge& operator[](const uint32_t index) const { return _storage[index]; };

private:

	inline halfEdge toParent(const uint32_t index) const { return _storage[(index/4)*4]; }
	std::vector<halfEdge> _storage;
};


// Find the index of a given subEdge with opposite as twin halfEdge
uint32_t SubEdgeTree::findSubEdge(halfEdge start, const halfEdge opposite, const float splitpoint) const
{
	halfEdge split_he = _storage[toIndex(start) + 1];
	float split = reinterpret_cast<float&>(split_he);
	halfEdge child = start;
	bool left = false;
	while (child.isSubdivided()) {
		// left child
		if (splitpoint < split + eps) {
			child = _storage[toIndex(child) + 2];
			left = true;
			if (!child.isSubdivided()) break;
		}
		// right child
		else {
			child = _storage[toIndex(child) + 3];
			left = false;
			if (!child.isSubdivided()) break;
		}
		start = child;
		split_he = _storage[toIndex(start) + 1];
		split = reinterpret_cast<float&>(split_he);
	}
	assert(child == opposite);
	return toIndex(start) + (left ? 2 : 3);
}

inline uint32_t SubEdgeTree::getsubEdges(const halfEdge start, std::vector<halfEdge>& subEdges) const
{
	// left child
	const halfEdge left_child = _storage[toIndex(start) + 2];
	if (left_child.isSubdivided()) {
		getsubEdges(left_child, subEdges);
	}
	else { subEdges.push_back(left_child); }
	// right child
	const halfEdge right_child = _storage[toIndex(start) + 3];
	if (right_child.isSubdivided()) {
		getsubEdges(right_child, subEdges);
	}
	else { subEdges.push_back(right_child); }
	return 0;
}

// Split a (sub)halfEdge in two with a left and right child, returns a reference to the top of the tree
inline halfEdge SubEdgeTree::splitHalfEdge(const halfEdge twin, const halfEdge Edge, const halfEdge original, const halfEdge left_child, const halfEdge right_child, float splitpoint)
{
	assert(!twin.isBorder());
	if (Edge.isBorder()) { return -1;}
	uint32_t subface_ref = _storage.size();
	if (twin.isSubdivided()) {
		// Already subdivided, find the subedge that corresponds to the split edge
		uint32_t index = findSubEdge(twin, original, splitpoint);
		_storage[index].id = (1 << 31) + subface_ref;
		_storage.push_back((1 << 31) + index); // parent
		_storage.push_back(reinterpret_cast<halfEdge &>(splitpoint)); // split
		_storage.push_back(left_child);
		_storage.push_back(right_child);
		return twin;
	}
	_storage.push_back(Edge);  // parent
	_storage.push_back(reinterpret_cast<halfEdge &>(splitpoint)); // split
	_storage.push_back(left_child);
	_storage.push_back(right_child);
	return halfEdge((1 << 31) + subface_ref);
}

inline void SubEdgeTree::removeNode(const halfEdge hf, std::vector<halfEdge>& E2e)
{
	// If the container only contains one node, just delete it
	if (_storage.size() == 4) {
		_storage.pop_back();
		_storage.pop_back();
		_storage.pop_back();
		_storage.pop_back();
		return;
	}
	// Else perform the swap of nodes
	auto parent = _storage.end() - 4;
	if ((*parent).isSubdivided()) {
		_storage[toIndex(*parent)] = hf;
	}
	else {
		E2e[(*parent).id] = hf;
	}
	std::iter_swap(_storage.begin() + toIndex(hf), _storage.end() - 4);
	std::iter_swap(_storage.begin() + toIndex(hf) + 1, _storage.end() - 3);
	std::iter_swap(_storage.begin() + toIndex(hf) + 2, _storage.end() - 2);
	std::iter_swap(_storage.begin() + toIndex(hf) + 3, _storage.end() - 1);
	_storage.pop_back();
	_storage.pop_back();
	_storage.pop_back();
	_storage.pop_back();
}

inline void SubEdgeTree::updateParent(halfEdge tree_head, halfEdge new_parent)
{
	if (!tree_head.isSubdivided()) return;
	(*this)[(toIndex(tree_head) / 4) * 4] = new_parent;
}

static inline bool isRight(uint32_t index) { return (index % 4) == 3; }

inline uint32_t SubEdgeTree::next(uint32_t index) const
{
	auto parent = toParent(index);
	// is right child
	if (isRight(index)) {
		// move up the chain
		if (!parent.isSubdivided()) { return -1; }
		while (isRight(toIndex(parent))) {
			parent = toParent(toIndex(parent));
			if (!parent.isSubdivided()) { return -1; }
		}
		index = toIndex(parent);
	}
	// Left child, so go to right
	index += 1;
	// take left turns
	while (_storage[index].isSubdivided()) {
		index = toIndex(_storage[index]) + 2;
	}
	return index;
}

inline uint32_t SubEdgeTree::begin(uint32_t head) const
{
	while (_storage[head + 2].isSubdivided()) {
		head = toIndex(_storage[head + 2]);
	}
	return head + 2;
}

inline std::pair<halfEdge, halfEdge> SubEdgeTree::subdivideTree(const halfEdge tree_head, const float splitpoint, std::vector<halfEdge>& E2e)
{
	// Runs recursively trough all the points assign parents appropriately
	std::pair<halfEdge, halfEdge> ret = { border_id, border_id };
	halfEdge split_he = _storage[toIndex(tree_head) + 1];
	float split = reinterpret_cast<float&>(split_he);
	halfEdge& left = _storage[toIndex(tree_head) + 2];
	halfEdge& right = _storage[toIndex(tree_head) + 3];
	if (std::abs(split - splitpoint) < eps) {
		// Split happens exactly in the middle
		ret = { left, right };
		// Remove the current node as it is no longer necassary
		removeNode(tree_head, E2e);
		return ret;
	}
	if (splitpoint < split + eps) {
		// Split happens left
		if (left.isSubdivided()) {
			ret = subdivideTree(left, splitpoint, E2e);
			// set left of this element to the right head
			if (ret.second != left) updateParent(ret.second, left);
			left = ret.second;
			// make right hand the owner
			ret.second = tree_head;
		}
		else {
			ret = { left,tree_head };
		}
	}
	else {
		// Split happens right
		if (right.isSubdivided()) {
			ret = subdivideTree(right, splitpoint, E2e);
			// set right of this to left head
			if(ret.first != right) updateParent(ret.first, right);
			right = ret.first;
			// make left the owner
			ret.first = tree_head;
		}
		else {
			ret = { tree_head, right };
		}
	}
	return ret;
}
