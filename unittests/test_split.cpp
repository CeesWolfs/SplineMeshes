#include <catch2/catch.hpp>
#include "../Mesh.hpp"
TEST_CASE("Split outside elements fails", "[Mesh]") {
	Mesh2D mesh;
	CHECK(mesh.SplitElement(0, axis::x, 1.4) == -1);
}

TEST_CASE("Split inside elements works", "[Mesh]") {
	Mesh2D mesh;
	CHECK(mesh.SplitElement(0, axis::x, 0.5) == 1);
	CHECK(mesh.size() == 2);
}

TEST_CASE("Number of edges is correct for simple non conformal object", "[Mesh]") {
	Mesh2D mesh;
	mesh.SplitElement(0, axis::x, 0.4);
	uint32_t new_elem = mesh.SplitElement(1, axis::y, 0.7);
	mesh.SplitElement(2, axis::x, 0.8);
	mesh.SplitElement(1, axis::y, 0.2);
	const auto adj = mesh.Adjancency(4);
	CHECK(adj.size() == 5);
}

TEST_CASE("Merging edges works", "[Mesh]") {
	Mesh2D mesh;
	mesh.SplitElement(0, axis::x, 0.5);
	mesh.SplitElement(1, axis::y, 0.5);
  mesh.SplitElement(0, axis::y, 0.5);
	CHECK(mesh.vertices_size() == 9);
}
