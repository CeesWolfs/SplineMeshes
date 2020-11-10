#pragma once

#include <vector>
#include <stdint.h>
#include <cstdio>
#include "Types.hpp"
#include "SubEdgeTree.hpp"


class Mesh2D
{
public:
	Mesh2D();
	int SplitElement(const uint32_t element, const enum axis split_axis, const float splitpoint);
	std::vector<halfEdge> Adjancency(uint32_t element); // TODO optimize this with a small vector like datatype
	std::pair<float, float> getHalfEdgeBounds(const halfEdge hf) const;
	uint32_t size() const { return EC.size(); };
	void Visualize();
private:
	void SplitAndSubdivide(halfEdge left, halfEdge right, float splitpoint,uint32_t& vertex);
	void updateTwin(halfEdge& twin, const halfEdge original, const halfEdge new_edge);
	uint32_t getLowestVertex(const halfEdge he);
	std::vector<Vertex> _vertices;
	std::vector<halfEdge> E2e;
	std::vector<Element> EC;
	std::vector<halfEdge> V2e;
	SubEdgeTree _subedges;
	halfEdge& Twin(halfEdge facet) {
		return E2e[facet.id];
	}
};

Mesh2D::Mesh2D()
{
	_vertices.push_back({ 0.0,0.0 });
	_vertices.push_back({ 1.0,0.0 });
	_vertices.push_back({ 1.0,1.0 });
	_vertices.push_back({ 0.0,1.0 });
	EC.push_back({ 0,1,2,3 });
	V2e.push_back({ 0,0 });
	V2e.push_back({ 0,1 });
	V2e.push_back({ 0,2 });
	V2e.push_back({ 0,3 });
	E2e.push_back({ halfEdge(border_id) });
	E2e.push_back({ halfEdge(border_id) });
	E2e.push_back({ halfEdge(border_id) });
	E2e.push_back({ halfEdge(border_id) });
}

// returns the id of the newly created element, -1 of no element could be created
int Mesh2D::SplitElement(uint32_t element, enum axis split_axis, float splitpoint)
{
	uint32_t new_elem_ref = EC.size();
	// x as split axis
	//	4--------------6------------4
	//	|   <elem,2>   |   <new,2>	|
	//	|              |			|
	//	|			   |	        |
	//	|	  <elem,0> |   <new,0>	|
	//	1--------------5------------3
	if (split_axis == x) {
		if ((splitpoint <= this->_vertices[this->EC[element].v1].x) || splitpoint >= this->_vertices[this->EC[element].v2].x) {
			return -1; // Splitpoint not in element
		}
		// Split the element, by first creating the two new vertices
		const Vertex v1_new = { splitpoint, this->_vertices[this->EC[element].v1].y };
		const Vertex v2_new = { splitpoint, this->_vertices[this->EC[element].v4].y };
		uint32_t v1_ref = this->_vertices.size();
		uint32_t v2_ref = v1_ref + 1;

		// Push back all half edges for the new element
		E2e.push_back(E2e[(element << 2)]);
		E2e.push_back(E2e[(element << 2) + 1]);
		E2e.push_back(E2e[(element << 2) + 2]);
		E2e.push_back(halfEdge(element, 1));
		// Finally update the half edges, for the left element we only need to update the second half edge
		E2e[(element << 2) + 1] = halfEdge(new_elem_ref, 3);

		if (!Twin(halfEdge(element, 0)).isBorder()) {
			SplitAndSubdivide(halfEdge(element, 0), halfEdge(new_elem_ref, 0), splitpoint, v1_ref);
		}
		if (!Twin(halfEdge(element, 2)).isBorder()) {
			SplitAndSubdivide(halfEdge(element, 2), halfEdge(new_elem_ref, 2), splitpoint, v2_ref);
		}

		// Push back the new vertices
		_vertices.push_back(v1_new);
		_vertices.push_back(v2_new);
		// Copy the orginal element
		Element Elem_copy = EC[element];
		EC.push_back(Elem_copy);
		// Make the original element the one with lower x values, we need to update vertex 2 and 3
		EC[element].v2 = v1_ref;
		EC[element].v3 = v2_ref;
		// Make the new element the one with higher x values, update vertex 1 and 4
		EC[new_elem_ref].v1 = v1_ref;
		EC[new_elem_ref].v4 = v2_ref;
		// Update v2e for the original vertices
		if (V2e[Elem_copy.v2].getElement() == element) {
			V2e[Elem_copy.v2] = halfEdge(new_elem_ref, 1);
		}
		if (V2e[Elem_copy.v3].getElement() == element) {
			V2e[Elem_copy.v3] = halfEdge(new_elem_ref, 2);
		}
		// Update v2e for the new vertices
		V2e.push_back(halfEdge(new_elem_ref, 0));
		V2e.push_back(halfEdge(element, 3));
	}
	if (split_axis == y) {
		if ((splitpoint <= this->_vertices[this->EC[element].v1].y) || splitpoint >= this->_vertices[this->EC[element].v4].y) {
			return -1; // Splitpoint not in element
		}
		// Splitting an element, multiple steps need to happen, firstly the new element should be created, 
		// either creating new vertices, or merging with existing subedge vertices. Also the subedges of the split edge should be divided amongst two
		// 

		// Split the element, by first creating the two new vertices
		const Vertex v1_new = { this->_vertices[this->EC[element].v1].x, splitpoint };
		const Vertex v2_new = { this->_vertices[this->EC[element].v2].x, splitpoint };
		uint32_t v1_ref = this->_vertices.size();
		uint32_t v2_ref = v1_ref + 1;
		// Push back the new vertices, todo only push backs none merged vertices
		_vertices.push_back(v1_new);
		_vertices.push_back(v2_new);
		// Copy the orginal element
		Element Elem_copy = EC[element];
		EC.push_back(Elem_copy);
		// Push back half edges for the new element, and divide the half edges that need to be split
		E2e.push_back(halfEdge(element, 2));
		E2e.push_back(E2e[(element << 2) + 1]);
		E2e.push_back(E2e[(element << 2) + 2]);
		E2e.push_back(E2e[(element << 2) + 3]);
		if (!Twin(halfEdge(element, 1)).isBorder())
			SplitAndSubdivide(halfEdge(element, 1), halfEdge(new_elem_ref, 1), splitpoint, v1_ref);
		if (!Twin(halfEdge(element, 3)).isBorder()) {
			SplitAndSubdivide(halfEdge(element, 3), halfEdge(new_elem_ref, 3), splitpoint, v2_ref);
		}
		// Finally update the half edges, for the bottomn element we only need to update the third first half edge
		E2e[(element << 2) + 2] = halfEdge(new_elem_ref, 0);
		// Make the original element the one with lower x values, we need to update vertex 2 and 3
		EC[element].v4 = v1_ref;
		EC[element].v3 = v2_ref;
		// Make the new element the one with higher x values, update vertex 1 and 4
		EC[new_elem_ref].v1 = v1_ref;
		EC[new_elem_ref].v2 = v2_ref;
		// Update v2e for the original vertices
		if (V2e[Elem_copy.v4].getElement() == element) {
			V2e[Elem_copy.v4] = halfEdge(new_elem_ref, 3);
		}
		if (V2e[Elem_copy.v3].getElement() == element) {
			V2e[Elem_copy.v3] = halfEdge(new_elem_ref, 2);
		}
		// Update v2e for the new vertices
		V2e.push_back(halfEdge(new_elem_ref, 0));
		V2e.push_back(halfEdge(new_elem_ref, 1));
	}
	return new_elem_ref;
}

inline std::vector<halfEdge> Mesh2D::Adjancency(uint32_t element)
{
	std::vector<halfEdge> adjanceny;
	for (size_t i = element * 4; i < (element + 1) * 4; i++)
	{
		//if (E2e[i].isBorder()) continue;
		if (E2e[i].isSubdivided() && !E2e[i].isBorder()) {
			_subedges.getsubEdges(E2e[i], adjanceny);
		}
		else {
			adjanceny.push_back(E2e[i]);
		}
	}
	return adjanceny;

}

inline std::pair<float, float> Mesh2D::getHalfEdgeBounds(const halfEdge hf) const
{
	const auto& elem = EC[hf.getElement()];
	switch (hf.getLocalId())
	{
	case 0: return std::pair<float, float>(_vertices[elem.v1].x, _vertices[elem.v2].x);
	case 1: return std::pair<float, float>(_vertices[elem.v2].y, _vertices[elem.v3].y);
	case 2: return std::pair<float, float>(_vertices[elem.v4].x, _vertices[elem.v3].x);
	case 3: return std::pair<float, float>(_vertices[elem.v1].y, _vertices[elem.v4].y);
	}
}

// Mesh to a python representation to be used with the visualize mesh python script
inline void Mesh2D::Visualize()
{
	printf("nodes = np.array([");
	for (const auto vert : _vertices) {
		printf("[%f, %f],", vert.x, vert.y);
	}
	printf("])\n");
	printf("elements = np.array([");
	for (const auto el : EC) {
		printf("[%d,%d,%d,%d],", el.v1, el.v2, el.v3, el.v4);
	}
	printf("])\n");
}

inline void Mesh2D::updateTwin(halfEdge& twin, const halfEdge original, const halfEdge new_edge)
{
	if (twin.isSubdivided()) {
		auto index = _subedges.findSubEdge(twin, original, getHalfEdgeBounds(original).second);
		_subedges[index] = new_edge;
	}
	else {
		twin = new_edge;
	}
}

// Get the vertex with the lowest coordinate on the half edge axis
inline uint32_t Mesh2D::getLowestVertex(const halfEdge he)
{
	assert(!he.isSubdivided());
	auto elem = he.getElement();
	switch (he.getLocalId())
	{
	case 0:
		return EC[elem].v1;
	case 1:
		return EC[elem].v2;
	case 2:
		return EC[elem].v4;
	case 3:
		return EC[elem].v1;
	}
}

// Complicated function. Splits half edges in two, by subdividing subedges, and splitting twins, updates the necassary twins
inline void Mesh2D::SplitAndSubdivide(halfEdge left, halfEdge right, float splitpoint, uint32_t& vertex)
{
	auto left_twin = Twin(left);
	if (left_twin.isSubdivided())
	{
		auto [left_head, right_head] = _subedges.subdivideTree(left_twin, splitpoint);
		// Reference to the first subedge now belonging to the right half edges
		auto node = _subedges.begin(_subedges.toIndex(right_head));
		// if the split is (almost) exactly at the border of a halfEdge, we need to merge vertices
		if (abs(splitpoint - getHalfEdgeBounds(_subedges[node]).first) < eps) {
			vertex = getLowestVertex(_subedges[node]);
			updateTwin(E2e[_subedges[node].id], left, right);
		}
		// Else split the twin element
		else {
			E2e[_subedges[node].id] = _subedges.splitHalfEdge(E2e[_subedges[node].id], left, right, splitpoint);

		}
		// todo check if we need nexts
		while (_subedges.next(node) != _subedges.end()) {
			node = _subedges.next(node);
			updateTwin(E2e[_subedges[node].id], left, right);	
		}
		Twin(left) = left_head;
		Twin(right) = right_head;
	}
	else {
		E2e[left_twin.id] = _subedges.splitHalfEdge(Twin(left_twin), left, right, splitpoint);
	}

}
