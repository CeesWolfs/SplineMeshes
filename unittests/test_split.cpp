#include <catch2/catch.hpp>
#include <random>
#include <algorithm>
#include <functional> // bind
#include "../Mesh.hpp"

namespace helpers {
	Mesh2D* random_mesh(int N) {
		Mesh2D* mesh = new Mesh2D;

		// Start with a uniform grid
		mesh->SplitElementInFourths(0, {0.5, 0.5});
		mesh->SplitElementInFourths(2, {0.75, 0.75});
		mesh->SplitElementInFourths(3, {0.25, 0.75});
		mesh->SplitElementInFourths(1, {0.75, 0.25});
		mesh->SplitElementInFourths(0, {0.25, 0.25});
		// Select random elements to split
		std::random_device random_device;
		std::mt19937 random_engine(random_device());
		std::uniform_real_distribution<float> fl_distr(0.2, 0.8);

		while(mesh->size() <= N) {
			std::uniform_int_distribution<int> distribution(0, mesh->size()-1);
			std::vector<uint32_t> to_split(10);
			std::generate(to_split.begin(),to_split.end(), std::bind(distribution, random_engine));
			for(auto elem : to_split) {
				if(fl_distr(random_engine) < 0.5) {
					auto bounds = mesh->getHalfEdgeBounds(halfEdge(elem, 0));
					float split = bounds.first + fl_distr(random_engine)*(bounds.second - bounds.first);
					mesh->SplitElement(elem, axis::x, split);
				}
				else {
					auto bounds = mesh->getHalfEdgeBounds(halfEdge(elem, 1));
					float split = bounds.first + fl_distr(random_engine)*(bounds.second - bounds.first);
					mesh->SplitElement(elem, axis::y, split);
				}

			}
		}
		return mesh;
	}
	Mesh2D* random_mesh_quadsplits(int N) {
		Mesh2D* mesh = new Mesh2D;

		// Start with a uniform grid
		mesh->SplitElementInFourths(0, {0.5, 0.5});
		mesh->SplitElementInFourths(2, {0.75, 0.75});
		mesh->SplitElementInFourths(3, {0.25, 0.75});
		mesh->SplitElementInFourths(1, {0.75, 0.25});
		mesh->SplitElementInFourths(0, {0.25, 0.25});
		// Select random elements to split
		std::random_device random_device;
		std::mt19937 random_engine(random_device());
		std::uniform_real_distribution<float> fl_distr(0.2, 0.8);

		while(mesh->size() <= N) {
			std::uniform_int_distribution<int> distribution(0, mesh->size()-1);
			std::vector<uint32_t> to_split(10);
			std::generate(to_split.begin(),to_split.end(), std::bind(distribution, random_engine));
			for(auto elem : to_split) {
				auto bounds = mesh->getHalfEdgeBounds(halfEdge(elem, 0));
				auto bounds1 = mesh->getHalfEdgeBounds(halfEdge(elem, 1));
				float split = bounds.first + fl_distr(random_engine)*(bounds.second - bounds.first);
				float split2 = bounds1.first + fl_distr(random_engine)*(bounds1.second - bounds1.first);
				mesh->SplitElementInFourths(elem, {split, split2});
			}
		}
		return mesh;
	}
} // helpers

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
	const auto adj = mesh.Adjacency(4);
	CHECK(adj.size() == 5);
}

TEST_CASE("Merging edges works", "[Mesh]") {
	Mesh2D mesh;
	mesh.SplitElement(0, axis::x, 0.5);
	mesh.SplitElement(1, axis::y, 0.5);
  	mesh.SplitElement(0, axis::y, 0.5);
	CHECK(mesh.vertices_size() == 9);
}

TEST_CASE("Tree tests", "[SubEdgeTree]") {
	Mesh2D mesh;
	mesh.SplitElement(0, axis::x, 0.5);
	uint32_t top = mesh.SplitElement(0, axis::y, 0.4);
	mesh.SplitElement(0, axis::y, 0.3);
	mesh.SplitElement(0, axis::y, 0.2);
	mesh.SplitElement(0, axis::y, 0.1);
	top = mesh.SplitElement(top, axis::y, 0.6);
	top = mesh.SplitElement(top, axis::y, 0.8);
	mesh.SplitElement(top - 1, axis::x, 0.3);
	mesh.SplitElement(5, axis::x, 0.3);
	mesh.SplitElement(1, axis::y, 0.5);
	mesh.SplitElement(1, axis::y, 0.2);
	for (int elem = 0; elem < mesh.size(); ++elem)
	{
		const auto adj = mesh.Adjacency(elem);
		for (auto& hf : adj) {
			if (hf.isBorder()) continue;
			REQUIRE(mesh.Adjacent(hf.getElement(), elem));
		}
	}
}

TEST_CASE("Sanity tests 1, All elements border eachother", "[Mesh]") {
	Mesh2D mesh;
	mesh.SplitElementInFourths(0, {0.5, 0.5});
	mesh.SplitElementInFourths(1, {0.75, 0.25});
	mesh.SplitElementInFourths(2, {0.75, 0.75});
	mesh.SplitElementInFourths(3, {0.25, 0.75});
	// Get the adjacency for each element
	for (int elem = 0; elem < mesh.size(); ++elem)
	{
		const auto adj = mesh.Adjacency(elem);
		for(auto& hf : adj) {
			if(hf.isBorder()) continue;
			REQUIRE(mesh.Adjacent(hf.getElement(), elem));
		}
	}

}

TEST_CASE("Sanity tests 1, All elements border eachother for a bunch of random meshes", "[Mesh]") {
	Mesh2D* mesh = helpers::random_mesh(30);
	mesh->Save("random_test1");
	for (int elem = 0; elem < mesh->size(); ++elem)
	{
		const auto adj = mesh->Adjacency(elem);
		for(auto& hf : adj) {
			if(hf.isBorder()) continue;
			REQUIRE(mesh->Adjacent(hf.getElement(), elem));
		}
	}
	mesh = helpers::random_mesh_quadsplits(40);
	mesh->Save("random_test2");
	for (int elem = 0; elem < mesh->size(); ++elem)
	{
		const auto adj = mesh->Adjacency(elem);
		for(auto& hf : adj) {
			if(hf.isBorder()) continue;
			REQUIRE(mesh->Adjacent(hf.getElement(), elem));
		}
	}
	mesh = helpers::random_mesh(50);
	mesh->Save("random_test3");
	for (int elem = 0; elem < mesh->size(); ++elem)
	{
		const auto adj = mesh->Adjacency(elem);
		for(auto& hf : adj) {
			if(hf.isBorder()) continue;
			if(!mesh->Adjacent(hf.getElement(), elem)) std::cout << elem;
			REQUIRE(mesh->Adjacent(hf.getElement(), elem));
		}
	}
}
