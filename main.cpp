#include "Mesh.hpp"

int main() {
	Mesh2D mesh;
	mesh.SplitElement(0, axis::x, 0.5);
	mesh.SplitElement(1, axis::y, 0.5);
	mesh.SplitElement(0, axis::y, 0.5);
	mesh.Visualize();
	const auto adj = mesh.Adjancency(0);
	char buf[20];
	for (const auto facet : adj) {
		printf("%s", facet.toStr(buf));
	}
	printf("\n");
}
