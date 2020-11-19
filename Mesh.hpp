#pragma once

#define TINYPLY_IMPLEMENTATION

#include <vector>
#include <numeric>
#include <stdint.h>
#include <string>
#include <fstream>
#include "tinyply.hpp" // For saving the mesh
#include "Types.hpp"
#include "SubEdgeTree.hpp"


class Mesh2D
{
public:
	Mesh2D(); // Default construct the Mesh starting as a 1 by 1 square
	// Split an element along an axis at a certain point, returns the id of the newly created element
	int SplitElement(uint32_t element, enum axis split_axis, float splitpoint);
	// Split an element in four pieces, along two axis, around a single point
	int SplitElementInFourths(uint32_t element, Vertex splitcentre);
	// Get the adjancenent half edges of an element
	std::vector<halfEdge> Adjacency(uint32_t element) const; // TODO optimize this with a small vector like datatype
	// Check if two elements are adjacent
	bool Adjacent(uint32_t elem1, uint32_t elem2) const;
	// Get the bounds, start and end points, of a half edge
	std::pair<float, float> getHalfEdgeBounds(halfEdge hf) const;
	// Returns the number of elements in the mesh
	uint32_t size() const { return EC.size(); };
	// Returns the number of vertices in the mesh
	uint32_t vertices_size() const {return _vertices.size(); };
	// Output to a format that a python script can draw
	void Visualize() const;
	// Save to a .ply file
	void Save(const std::string& filename);
private:
	// Function called by SplitElement, Splits and subdivides a halfedge of the element that is being splitted
	// Returns wheter the vertex is merged with an existing vertex
	bool SplitHalfEdge(halfEdge original, halfEdge new_left, halfEdge new_right, float splitpoint,uint32_t& vertex);
	// Function called by SplitElement, updates a twin half edge to point to the new element
	void updateTwin(halfEdge& twin, halfEdge original, halfEdge new_edge);
	// Function called by SplitElement, switches a half edge to a new element
	void updateHalfEdge(halfEdge original, halfEdge new_edge);
	// Gets the vertex with the lowest coordinate along of the halfedge, needed for lookups
	uint32_t getLowestVertex(halfEdge he);
	// Check if a vertex is contained in a element
	inline bool inElement(uint32_t element, const Vertex v) {
		if (_vertices[EC[element].v1].x > v.x || v.x > _vertices[EC[element].v2].x) return false;
		if (_vertices[EC[element].v1].y > v.y || v.y > _vertices[EC[element].v4].y) return false;
		return true;
	}
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
	// Splitting an element, multiple steps need to happen, firstly the new element should be created,
	// either creating new vertices, or merging with existing subedge vertices.
	// Also the subedges of the split edge should be divided amongst two
	bool v5_merged = false, v6_merged = false;
	uint32_t new_elem_ref = EC.size();
	// x as split axis
	//	4--------------6------------4
	//	|   <elem,2>   |   <new,2>	|
	//	|              |			      |
	//	|			   			 |	          |
	//	|	  <elem,0> 	 |   <new,0>	|
	//	1--------------5------------3
	if (split_axis == x) {
		if ((splitpoint <= this->_vertices[this->EC[element].v1].x) || splitpoint >= this->_vertices[this->EC[element].v2].x) {
			return -1; // Splitpoint not in element
		}
		// Split the element, by first creating the two new vertices
		const Vertex v5_new = { splitpoint, this->_vertices[this->EC[element].v1].y };
		const Vertex v6_new = { splitpoint, this->_vertices[this->EC[element].v4].y };
		uint32_t v5_ref = this->_vertices.size();
		uint32_t v6_ref = v5_ref + 1;
		// Push back all half edges for the new element
		E2e.push_back(Twin(halfEdge(element, 0)));
		E2e.push_back(Twin(halfEdge(element, 1)));
		E2e.push_back(Twin(halfEdge(element, 2)));
		E2e.push_back(halfEdge(element, 1));
		// Finally update the half edges, for the left element we point the second half edge to the new element
		updateHalfEdge(halfEdge(element, 1), halfEdge(new_elem_ref, 1));
		Twin(halfEdge(element, 1)) = halfEdge(new_elem_ref, 3);
		// Update the first and third half edges, those are now split in two
		v5_merged = SplitHalfEdge(halfEdge(element, 0), halfEdge(element, 0), halfEdge(new_elem_ref, 0), splitpoint, v5_ref);
		if (v5_merged) v6_ref--;
		v6_merged = SplitHalfEdge(halfEdge(element, 2), halfEdge(element, 2), halfEdge(new_elem_ref, 2), splitpoint, v6_ref);
		// Push back the new vertices if not merged
		if (!v5_merged) {
			V2e.push_back(halfEdge(new_elem_ref, 0));
			_vertices.push_back(v5_new);
		}
		if(!v6_merged) {
			V2e.push_back(halfEdge(new_elem_ref, 3));
			_vertices.push_back(v6_new);
		}
		// Copy the orginal element
		Element Elem_copy = EC[element];
		EC.push_back(Elem_copy);
		// Make the original element the one with lower x values, we need to update vertex 2 and 3
		EC[element].v2 = v5_ref;
		EC[element].v3 = v6_ref;
		// Make the new element the one with higher x values, update vertex 1 and 4
		EC[new_elem_ref].v1 = v5_ref;
		EC[new_elem_ref].v4 = v6_ref;
		// Update v2e for the original vertices
		if (V2e[Elem_copy.v2].getElement() == element) {
			V2e[Elem_copy.v2] = halfEdge(new_elem_ref, 1);
		}
		if (V2e[Elem_copy.v3].getElement() == element) {
			V2e[Elem_copy.v3] = halfEdge(new_elem_ref, 2);
		}
	}
	// y as split axis
	//	4---------------------------3
	//	|	         <new,2> 	    |
	//	|	         <new,0> 	 	|
	//	6---------------------------5
	//  |	         <elem,2> 	 	|
	//	|	         <elem,0> 	 	|
	//	1---------------------------2
	if (split_axis == y) {
		if ((splitpoint <= this->_vertices[this->EC[element].v1].y) || splitpoint >= this->_vertices[this->EC[element].v4].y) {
			return -1; // Splitpoint not in element
		}
		// Split the element, by first creating the two new vertices
		const Vertex v5_new = { this->_vertices[this->EC[element].v2].x, splitpoint };
		const Vertex v6_new = { this->_vertices[this->EC[element].v1].x, splitpoint };
		uint32_t v5_ref = this->_vertices.size();
		uint32_t v6_ref = v5_ref + 1;
		// Push back the half edges for the new element
		E2e.push_back(halfEdge(element, 2));
		E2e.push_back(Twin(halfEdge(element, 1)));
		E2e.push_back(Twin(halfEdge(element, 2)));
		E2e.push_back(Twin(halfEdge(element, 3)));
		// Finally update the half edges, for the left element we point the second half edge to the new element
		updateHalfEdge(halfEdge(element,2), halfEdge(new_elem_ref, 2));
		Twin(halfEdge(element, 2)) = halfEdge(new_elem_ref, 0);
		// Update the first and third half edges, those are now split in two
		v5_merged = SplitHalfEdge(halfEdge(element, 1), halfEdge(element, 1), halfEdge(new_elem_ref, 1), splitpoint, v5_ref);
		if (v5_merged) v6_ref--;
		v6_merged = SplitHalfEdge(halfEdge(element, 3), halfEdge(element, 3), halfEdge(new_elem_ref, 3), splitpoint, v6_ref);
		// Push back the new vertices if not merged
		if (!v5_merged) {
			V2e.push_back(halfEdge(new_elem_ref, 2));
			_vertices.push_back(v5_new);
		}
		if(!v6_merged) {
			V2e.push_back(halfEdge(new_elem_ref, 3));
			_vertices.push_back(v6_new);
		}
		// Copy the orginal element
		Element Elem_copy = EC[element];
		EC.push_back(Elem_copy);
		// Make the original element the one with lower y values, we need to update vertex 3 and 4
		EC[element].v3 = v5_ref;
		EC[element].v4 = v6_ref;
		// Make the new element the one with higher y values, update vertex 1 and 2
		EC[new_elem_ref].v1 = v6_ref;
		EC[new_elem_ref].v2 = v5_ref;
		// Update v2e for the original vertices
		if (V2e[Elem_copy.v3].getElement() == element) {
			V2e[Elem_copy.v3] = halfEdge(new_elem_ref, 1);
		}
		if (V2e[Elem_copy.v4].getElement() == element) {
			V2e[Elem_copy.v4] = halfEdge(new_elem_ref, 2);
		}
	}
	return new_elem_ref;
}

int Mesh2D::SplitElementInFourths(uint32_t element, const Vertex splitcentre)
{
//	4------------7--------------3
//	|	         |	 		  	|
//	|	new_3    |	new_2       |
//	8------------9--------------6
//  |	         |              |
//	|    elem    |  new_1       |
//	1------------5--------------2

	// TODO check if vertex in element
	assert(inElement(element, splitcentre));
	// Define all the new vertices

	// Split the element, by first creating the two new vertices
	const Vertex v5_new = { splitcentre.x, this->_vertices[this->EC[element].v1].y };
	const Vertex v6_new = { this->_vertices[this->EC[element].v2].x, splitcentre.y };
	const Vertex v7_new = { splitcentre.x, this->_vertices[this->EC[element].v3].y };
	const Vertex v8_new = { this->_vertices[this->EC[element].v1].x, splitcentre.y };
	const Vertex v9_new = splitcentre;
	// Store references to all the vertices in the _vertices array
	std::array<bool, 4> vertex_merged{ false };
	std::array<uint32_t, 5> vertex_references;
	std::iota(vertex_references.begin(), vertex_references.end(), _vertices.size());
	// Reference to the first new element
	uint32_t new_elem = EC.size();

	// Push back all half edges for the first new element
	E2e.push_back(Twin(halfEdge(element, 0)));
	E2e.push_back(Twin(halfEdge(element, 1)));
	E2e.push_back(halfEdge(new_elem + 1, 0));
	E2e.push_back(halfEdge(element, 1));
	// Push back all half edges for the second new element
	E2e.push_back(halfEdge(new_elem, 2));
	E2e.push_back(Twin(halfEdge(element, 1)));
	E2e.push_back(Twin(halfEdge(element, 2)));
	E2e.push_back(halfEdge(new_elem + 2, 1));
	// Push back all half edges for the third new element
	E2e.push_back(halfEdge(element, 2));
	E2e.push_back(halfEdge(new_elem + 1, 3));
	E2e.push_back(Twin(halfEdge(element, 2)));
	E2e.push_back(Twin(halfEdge(element, 3)));

	// Split all the original half edges for the original element
	vertex_merged[0] = SplitHalfEdge(halfEdge(element, 0), halfEdge(element, 0), halfEdge(new_elem, 0), splitcentre.x, vertex_references[0]);
	vertex_merged[1] = SplitHalfEdge(halfEdge(element, 1), halfEdge(new_elem, 1), halfEdge(new_elem + 1, 1), splitcentre.y, vertex_references[1]);
	vertex_merged[2] = SplitHalfEdge(halfEdge(element, 2), halfEdge(new_elem + 2, 2), halfEdge(new_elem + 1, 2), splitcentre.x, vertex_references[2]);
	vertex_merged[3] = SplitHalfEdge(halfEdge(element, 3), halfEdge(element, 3), halfEdge(new_elem + 2, 3), splitcentre.y, vertex_references[3]);

	// Update the original half edges
	Twin(halfEdge(element, 1)) = halfEdge(new_elem, 3);
	Twin(halfEdge(element, 2)) = halfEdge(new_elem + 2, 0);

	// Update all the vertex refernces by subtracting merged vertices
	int i = 0;
	if (!vertex_merged[1]) { vertex_references[1] -= std::count(vertex_merged.begin(), vertex_merged.begin() + 1, true); }
	if (!vertex_merged[2]) { vertex_references[2] -= std::count(vertex_merged.begin(), vertex_merged.begin() + 2, true); }
	if (!vertex_merged[3]) { vertex_references[3] -= std::count(vertex_merged.begin(), vertex_merged.begin() + 3, true); }
	vertex_references[4] -= std::count(vertex_merged.begin(), vertex_merged.end(), true);
	// Create all the new elements
	// First copy the old vertices
	Element Elem = EC[element];
	EC.push_back({ vertex_references[0], Elem.v2, vertex_references[1], vertex_references[4] });
	EC.push_back({ vertex_references[4], vertex_references[1], Elem.v3, vertex_references[2] });
	EC.push_back({ vertex_references[3], vertex_references[4], vertex_references[2], Elem.v4 });
	EC[element] = { Elem.v1, vertex_references[0], vertex_references[4], vertex_references[3] };
	// Create the new vertices and Update V2e for the new vertices
	if (!vertex_merged[0]) { V2e.push_back(halfEdge(new_elem, 0)); _vertices.push_back(v5_new); }
	if (!vertex_merged[1]) { V2e.push_back(halfEdge(new_elem + 1, 1)); _vertices.push_back(v6_new); }
	if (!vertex_merged[2]) { V2e.push_back(halfEdge(new_elem + 2, 2)); _vertices.push_back(v7_new); }
	if (!vertex_merged[3]) { V2e.push_back(halfEdge(element, 3)); _vertices.push_back(v8_new);}
	V2e.push_back(halfEdge(new_elem + 2, 0));
	_vertices.push_back(v9_new);
	// Update V2e for the already exisiting vertices
	i = 1;
	std::for_each(Elem.vertices.begin() + 1, Elem.vertices.end(), [&](uint32_t& vertex) {if (V2e[vertex].getElement() == element) { V2e[vertex] = halfEdge(new_elem + i, i); i++; } });
	return element;
}

inline std::vector<halfEdge> Mesh2D::Adjacency(uint32_t element) const
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

bool Mesh2D::Adjacent(uint32_t elem1, uint32_t elem2) const
{
	for (size_t i = elem1 * 4; i < (elem1 + 1) * 4; i++)
	{
		auto twin = E2e[i];
		if (twin.isBorder()) continue;
		if (twin.isSubdivided()) {
			for (auto node = _subedges.begin(_subedges.toIndex(twin)); node != _subedges.end(); node = _subedges.next(node))
			{
				if (_subedges[node].getElement() == elem2) return true;
			}
		}
		else {
			if (E2e[i].getElement() == elem2) return true;
		}
	}
	return false;
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
	// Note this should never happen, but makes the compiler happy
	default: return std::pair<float, float>(_vertices[elem.v1].x, _vertices[elem.v2].x);
	}
}

// Mesh to a python representation to be used with the visualize mesh python script
inline void Mesh2D::Visualize() const
{
	printf("nodes = np.array([");
	for (const auto vert : _vertices) {
		printf("[%f, %f],", vert.x, vert.y);
	}
	printf("])\n");
	printf("elements = np.array([");
	for (const auto& el : EC) {
		printf("[%d,%d,%d,%d],", el.v1, el.v2, el.v3, el.v4);
	}
	printf("])\n");
}

inline void Mesh2D::Save(const std::string& filename)
{
	std::filebuf fb_binary;
	fb_binary.open(filename + ".ply", std::ios::out | std::ios::binary);
	std::ostream outstream_binary(&fb_binary);
	if (outstream_binary.fail()) throw std::runtime_error("failed to open " + filename);
	tinyply::PlyFile file;
	file.add_properties_to_element("vertex", { "x", "y", "z"},
		tinyply::Type::FLOAT32, _vertices.size(), reinterpret_cast<uint8_t*>(_vertices.data()), tinyply::Type::INVALID, 0);
	file.add_properties_to_element("face", { "vertex_indices" },
		tinyply::Type::UINT32, EC.size(), reinterpret_cast<uint8_t*>(EC.data()), tinyply::Type::UINT8, 4);
	// Write a binary file
	file.write(outstream_binary, true);
}

inline void Mesh2D::updateTwin(halfEdge& twin, const halfEdge original, const halfEdge new_edge)
{
	assert(!twin.isBorder());
	if (twin.isSubdivided()) {
		auto index = _subedges.findSubEdge(twin, original, getHalfEdgeBounds(original).second);
		_subedges[index] = new_edge;
	}
	else {
		twin = new_edge;
	}
}

inline void Mesh2D::updateHalfEdge(halfEdge original, halfEdge new_edge)
{
	auto twin = E2e[original.id];
	if (twin.isBorder()) return;
	// Update all twins
	if (twin.isSubdivided()) {
		auto node = _subedges.begin(_subedges.toIndex(twin));
		updateTwin(E2e[_subedges[node].id], original, new_edge);
		while (_subedges.next(node) != _subedges.end()) {
			node = _subedges.next(node);
			updateTwin(E2e[_subedges[node].id], original, new_edge);
		}
	}
	else {
		updateTwin(Twin(twin), original, new_edge);
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
	// Note this should never happen, but makes the compiler happy
	default:
		return EC[elem].v1;
	}
}

// Splits half edges in two, by subdividing subedges, and splitting twins, updates the necassary twins
// TODO REWRITE
bool Mesh2D::SplitHalfEdge(halfEdge original, halfEdge new_left, halfEdge new_right, float splitpoint, uint32_t& vertex)
{
	bool merged = false;
	auto left_twin = Twin(original);
	// Do not split borders
	if (left_twin.isBorder()) return false;
	if (left_twin.isSubdivided())
	{
		auto [left_head, right_head] = _subedges.subdivideTree(left_twin, splitpoint, E2e);
		if (left_head.isSubdivided()) _subedges.updateParent(left_head, new_left);
		if (right_head.isSubdivided()) _subedges.updateParent(left_head, new_right);
		if(right_head.isSubdivided()) {
			// Reference to the first subedge now belonging to the right half edges
			auto node = _subedges.begin(_subedges.toIndex(right_head));
			// if the split is (almost) exactly at the border of a halfEdge, we need to merge vertices
			if (std::abs(splitpoint - getHalfEdgeBounds(_subedges[node]).first) < eps) {
				vertex = getLowestVertex(_subedges[node]);
				merged = true;
				updateTwin(E2e[_subedges[node].id], original, new_right);
			}
			// Else split the twin element
			else {
				E2e[_subedges[node].id] = _subedges.splitHalfEdge(E2e[_subedges[node].id], _subedges[node], original, new_left, new_right, splitpoint);
			}
			while (_subedges.next(node) != _subedges.end()) {
				node = _subedges.next(node);
				updateTwin(E2e[_subedges[node].id], original, new_right);
			}
		}
		else {
			// if the split is (almost) exactly at the border of a halfEdge, we need to merge vertices
			if (std::abs(splitpoint - getHalfEdgeBounds(right_head).first) < eps) {
				vertex = getLowestVertex(right_head);
				merged = true;
				updateTwin(E2e[right_head.id], original, new_right);
			}
			// Else split the twin element
			else {
				Twin(right_head) = _subedges.splitHalfEdge(Twin(right_head), right_head, original, new_left, new_right, splitpoint);
			}
		}
		if (new_left != original) {
			if (left_head.isSubdivided()) {
				auto node = _subedges.begin(_subedges.toIndex(left_head));
				for (; _subedges.next(node) != _subedges.end(); node = _subedges.next(node))
				{
					// Error if update already happened!
					updateTwin(E2e[_subedges[node].id], original, new_left);
				}
				// Only update the last left twin if merged
				if (merged) updateTwin(E2e[_subedges[node].id], original, new_left);
			}
			else {
				// Only necassary if merged
				if(merged) updateTwin(E2e[left_head.id], original, new_left);
			}
		}

		Twin(new_left) = left_head;
		Twin(new_right) = right_head;
	}
	else {
		Twin(left_twin) = _subedges.splitHalfEdge(Twin(left_twin), left_twin, original, new_left, new_right, splitpoint);
	}
	return merged;
}
