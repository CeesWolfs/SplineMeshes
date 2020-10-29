#pragma once
#include "Types.hpp"
#include "Mesh.hpp"
#include <cstdint>
#include <vector>
#include <algorithm>
#include <utility>

// Datastructure used for the subtrees, implement in a tree like fashion, provides fast insertion O(m), fast deletion O(m), and reasonble lookup O(m) where m is depth (approx log n) and very limited in our case
// Storage is in ints layout is as follows |parent||left child||right child|
class SubFaceTree
{
public:
	uint32_t findSubFacet(const halfFacet start, const halfFacet opposite) const;
	uint32_t getsubFacets(const halfFacet start, std::vector<halfFacet>& subFacets) const;
	inline halfFacet splitHalfFacet(const halfFacet facet, const halfFacet left_child, const halfFacet right_child);
	inline const std::pair<halfFacet, halfFacet> subdivideTree(const halfFacet tree_head, const float split_point, const class Mesh2D& mesh);

private:
	inline const uint32_t toIndex(const halfFacet key) const { return (key.id & 0x7fffffff); }
	halfFacet indexToParent(const uint32_t index) const {
		return _storage[index / 3];
	}
	const halfFacet operator[](const halfFacet index) { return _storage[toIndex(index)]; };
	std::vector<halfFacet> _storage;
};

// Find the index of a given subFacet with opposite as twin halfFacet
inline uint32_t SubFaceTree::findSubFacet(const halfFacet start, const halfFacet opposite) const
{
	uint32_t res = 0;
	// left child
	const halfFacet left_child = _storage[toIndex(start)];
	if (left_child == opposite) res = toIndex(start);
	if (left_child.isSubdivided()) {
		uint32_t leftRes = findSubFacet(left_child, opposite);
		if (leftRes != 0) return leftRes;
	}
	// right child
	const halfFacet right_child = _storage[toIndex(start) + 1];
	if (right_child == opposite) res = toIndex(start) + 1;
	if (right_child.isSubdivided()) {
		uint32_t rightRes = findSubFacet(right_child, opposite);
		if (rightRes != 0) return rightRes;
	}
	return res;
}

inline uint32_t SubFaceTree::getsubFacets(const halfFacet start, std::vector<halfFacet>& subFacets) const
{
	// left child
	const halfFacet left_child = _storage[toIndex(start)];
	if (left_child.isSubdivided()) {
		getsubFacets(left_child, subFacets);
	}
	else { subFacets.push_back(left_child); }
	// right child
	const halfFacet right_child = _storage[toIndex(start) + 1];
	if (right_child.isSubdivided()) {
		getsubFacets(right_child, subFacets);
	}
	else { subFacets.push_back(right_child); }
	return 0;
}

// Split a (sub)halfFacet in two with a left and right child, returns a reference to the top of the tree
inline halfFacet SubFaceTree::splitHalfFacet(const halfFacet facet, const halfFacet left_child, const halfFacet right_child)
{
	if (facet.isBorder()) return -1;
	uint32_t subface_ref = _storage.size() + 1;
	if (facet.isSubdivided()) {
		// Already subdivided, find the subedge that corresponds to the split edge
		uint32_t index = findSubFacet(facet, left_child);
		_storage[index].id = (1 << 31) + subface_ref;
		_storage.push_back(_storage[index]); // parent
		_storage.push_back(left_child);
		_storage.push_back(right_child);
		return facet;
	}
	else {
		_storage.push_back(facet);  // parent, todo parent refers to the wrong element
		_storage.push_back(left_child);
		_storage.push_back(right_child);
		return halfFacet((1 << 31) + subface_ref);
	}
}

// Returns -1 if the point is entirely outside, 1 if the point is entirely inside, 0 if the point is on the border
static inline int pointinhalfFacet(const halfFacet he, const float splitpoint, const class Mesh2D& mesh)
{
	auto[a, b] = mesh.getHalfEdgeBounds(he);
	constexpr float eps = 1e-7;
	if ((b - splitpoint) < eps) return 0;
	if (a > b) std::swap(a, b);
	if ((splitpoint > a) && (splitpoint < b)) {
		return 1;
	}
	return -1;
}

inline const std::pair<halfFacet, halfFacet> SubFaceTree::subdivideTree(const halfFacet tree_head, const float split_point, const class Mesh2D& mesh)
{
	// Runs recursively trough all the points, might be a bottleneck, will optimise if this is the case
	std::pair<halfFacet, halfFacet> ret = { border_id, border_id };
	halfFacet left = _storage[toIndex(tree_head)];
	halfFacet right = _storage[toIndex(tree_head) + 1];
	if(left.isSubdivided()) {
		return subdivideTree(left, split_point, mesh);
	}
	else {
		const int in = pointinhalfFacet(left, split_point, mesh);
		if (in == -1) { ret.first = left; } // splitpoint not in the interval
		if (in == 0) { ret.first = left; ret.second = right; }
		if (in == 1) { ret.first = left; ret.second = left; }
	}
	if (right.isSubdivided()) {
		return subdivideTree(right, split_point, mesh);
	}
	else {
		const int in = pointinhalfFacet(right, split_point, mesh);
		if (in == -1) { ret.first = left; } // splitpoint not in the interval
		if (in == 0) { ret.first = left; ret.second = right; }
		if (in == 1) { ret.first = left; ret.second = left; }
	}
	return std::pair<halfFacet, halfFacet>(tree_head,tree_head);
}
