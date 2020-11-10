#include "Mesh.hpp"

int main() {
	Mesh2D mesh;
	mesh.SplitElement(0, axis::x, 0.4);
	uint32_t new_elem = mesh.SplitElement(1, axis::y, 0.7);
	mesh.SplitElement(2, axis::x, 0.8);
	new_elem = mesh.SplitElement(1, axis::y, 0.2);
	mesh.SplitElement(0, axis::y, 0.4);
	mesh.Visualize();
	const auto adj = mesh.Adjancency(0);
	char buf[20];
	for (const auto facet : adj) {
		printf(facet.toStr(buf));
	}
}