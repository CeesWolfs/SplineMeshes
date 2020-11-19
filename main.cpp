#include <random>
#include "Mesh.hpp"
#include "Spline.hpp"

int main() {
	Mesh2D mesh;
	mesh.SplitElement(0, axis::x, 0.5);
	mesh.SplitElement(1, axis::y, 0.5);
	mesh.SplitElement(0, axis::y, 0.5);
	auto new_elem = mesh.SplitElement(0, axis::x, 0.25);
	mesh.SplitElement(0, axis::y, 0.25);
	mesh.SplitElement(new_elem, axis::y, 0.25);
	mesh.Save("mesh");
	mesh.Visualize();
	const auto adj = mesh.Adjacency(new_elem);
	char buf[20];
	for (const auto facet : adj) {
		printf("%s", facet.toStr(buf));
	}
}
