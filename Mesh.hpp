#pragma once

#include <vector>
#include <stdint.h>
#include <cstdio>
#include "Types.hpp"
#include "SubFaceTree.hpp"


class Mesh2D
{
public:
	Mesh2D();
	int SplitElement(const uint32_t element, const enum axis split_axis, const float splitpoint);
	std::vector<halfFacet> Adjancency(uint32_t element); // TODO optimize this with a small vector like datatype
	std::pair<float, float> getHalfEdgeBounds(const halfFacet he) const;
	void Visualize();
private:
	std::vector<Vertex> _vertices;
	std::vector<halfFacet> E2e;
	std::vector<Element> EC;
	std::vector<halfFacet> V2e;
	SubFaceTree _subedges;
	halfFacet Twin(halfFacet facet) {
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
	E2e.push_back({ halfFacet(border_id) });
	E2e.push_back({ halfFacet(border_id) });
	E2e.push_back({ halfFacet(border_id) });
	E2e.push_back({ halfFacet(border_id) });
}

// returns the id of the newly created element, -1 of no element could be created
int Mesh2D::SplitElement(uint32_t element, enum axis split_axis, float splitpoint)
{
	uint32_t new_elem_ref = EC.size();
	// x as split axis
	//	4--------------6------------4
	//	|   <elem,2>   |   <new,2>	|
	//	|              |			|
	//	|			   |			|
	//	|	<elem,0>   |   <new,0>	|
	//	1--------------5------------3
	if (split_axis == x) {
		if ((splitpoint <= this->_vertices[this->EC[element].v1].x) || splitpoint >= this->_vertices[this->EC[element].v2].x) {
			return -1; // Splitpoint not in element
		}
		// Split the element, by first creating the two new vertices
		const Vertex v1_new = { splitpoint, this->_vertices[this->EC[element].v1].y };
		const Vertex v2_new = { splitpoint, this->_vertices[this->EC[element].v4].y };
		const uint32_t v1_ref = this->_vertices.size();
		// Push back the new vertices
		_vertices.push_back(v1_new);
		_vertices.push_back(v2_new);
		// Copy the orginal element
		Element Elem_copy = EC[element];
		EC.push_back(Elem_copy);
		// Make the original element the one with lower x values, we need to update vertex 2 and 3
		EC[element].v2 = v1_ref;
		EC[element].v3 = v1_ref + 1;
		// Make the new element the one with higher x values, update vertex 1 and 4
		EC[new_elem_ref].v1 = v1_ref;
		EC[new_elem_ref].v4 = v1_ref + 1;
		// Push back all half edges for the new element
		E2e.push_back(E2e[(element << 2)]);
		E2e.push_back(E2e[(element << 2) + 1]);
		E2e.push_back(E2e[(element << 2) + 2]);
		E2e.push_back(halfFacet(element, 1));
		// Finally update the half edges, for the left element we only need to update the second half edge
		E2e[(element << 2) + 1] = halfFacet(new_elem_ref, 3);
		// Update v2e for the original vertices
		if (V2e[Elem_copy.v2].getElement() == element) {
			V2e[Elem_copy.v2] = halfFacet(new_elem_ref, 1);
		}
		if (V2e[Elem_copy.v3].getElement() == element) {
			V2e[Elem_copy.v3] = halfFacet(new_elem_ref, 2);
		}
		// Update v2e for the new vertices
		V2e.push_back(halfFacet(new_elem_ref, 0));
		V2e.push_back(halfFacet(element, 3));
		// Finally we need to update all twin half_facets
		if(!Twin(halfFacet(element, 2)).isBorder())
			E2e[Twin(halfFacet(element, 2)).id] = _subedges.splitHalfFacet(E2e[Twin(halfFacet(element, 2)).id], halfFacet(element, 2), halfFacet(new_elem_ref, 2));
		if (!Twin(halfFacet(element, 0)).isBorder())
			E2e[Twin(halfFacet(element, 0)).id] = _subedges.splitHalfFacet(E2e[Twin(halfFacet(element, 1)).id], halfFacet(element, 0), halfFacet(new_elem_ref, 0));
	}
	if (split_axis == y) {
		if ((splitpoint <= this->_vertices[this->EC[element].v1].y) || splitpoint >= this->_vertices[this->EC[element].v4].y) {
			return -1; // Splitpoint not in element
		}
		// Split the element, by first creating the two new vertices
		const Vertex v1_new = { this->_vertices[this->EC[element].v1].x, splitpoint };
		const Vertex v2_new = { this->_vertices[this->EC[element].v2].x, splitpoint };
		const uint32_t v1_ref = this->_vertices.size();
		// Push back the new vertices
		_vertices.push_back(v1_new);
		_vertices.push_back(v2_new);
		// Copy the orginal element
		Element Elem_copy = EC[element];
		EC.push_back(Elem_copy);
		// Make the original element the one with lower x values, we need to update vertex 2 and 3
		EC[element].v4 = v1_ref;
		EC[element].v3 = v1_ref + 1;
		// Make the new element the one with higher x values, update vertex 1 and 4
		EC[new_elem_ref].v1 = v1_ref;
		EC[new_elem_ref].v2 = v1_ref + 1;
		// Push back all half edges for the new element TODO if the opposite is divided implement checking which half edges still overlap
		E2e.push_back(halfFacet(element, 2));
		E2e.push_back(E2e[(element << 2) + 1]);
		E2e.push_back(E2e[(element << 2) + 2]);
		E2e.push_back(E2e[(element << 2) + 3]);
		// Finally update the half edges, for the bottomn element we only need to update the third first half edge
		E2e[(element << 2) + 2] = halfFacet(new_elem_ref, 0);
		// Update v2e for the original vertices
		if (V2e[Elem_copy.v4].getElement() == element) {
			V2e[Elem_copy.v4] = halfFacet(new_elem_ref, 3);
		}
		if (V2e[Elem_copy.v3].getElement() == element) {
			V2e[Elem_copy.v3] = halfFacet(new_elem_ref, 2);
		}
		// Update v2e for the new vertices
		V2e.push_back(halfFacet(new_elem_ref, 0));
		V2e.push_back(halfFacet(new_elem_ref, 1));
		// Finally we need to update all twin half_facets
		if (!Twin(halfFacet(element, 1)).isBorder())
			E2e[Twin(halfFacet(element, 1)).id] = _subedges.splitHalfFacet(E2e[Twin(halfFacet(element, 1)).id], halfFacet(element, 1), halfFacet(new_elem_ref, 1));
		if (!Twin(halfFacet(element, 3)).isBorder()) {
			E2e[Twin(halfFacet(element, 3)).id] = _subedges.splitHalfFacet(E2e[Twin(halfFacet(element, 3)).id], halfFacet(element, 3), halfFacet(new_elem_ref, 3));
		}
	}
	return new_elem_ref;
}

inline std::vector<halfFacet> Mesh2D::Adjancency(uint32_t element)
{
	std::vector<halfFacet> adjanceny;
	for (size_t i = element*4; i < (element+1)*4; i++)
	{
		//if (E2e[i].isBorder()) continue;
		if (E2e[i].isSubdivided() && !E2e[i].isBorder()) {
			_subedges.getsubFacets(E2e[i], adjanceny);
		}
		else {
			adjanceny.push_back(E2e[i]);
		}
	}
	return adjanceny;

}

inline std::pair<float, float> Mesh2D::getHalfEdgeBounds(const halfFacet he) const
{
	uint32_t v_ori, v_dest;
	const auto & elem = EC[hf.getElement];
	switch (hf.getLocalId)
	{
	case 0: return std::pair<float, float>(_vertices[elem.v1].x, _vertices[elem.v2].x);
	case 1: return std::pair<float, float>(_vertices[elem.v2].y, _vertices[elem.v3].y);
	case 2: return std::pair<float, float>(_vertices[elem.v3].x, _vertices[elem.v4].x);
	case 3: return std::pair<float, float>(_vertices[elem.v4].y, _vertices[elem.v1].y);
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
		printf("[%d,%d,%d,%d],", el.v1, el.v2,el.v3,el.v4);
	}
	printf("])\n");
}