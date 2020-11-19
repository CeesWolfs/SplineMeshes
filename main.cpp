#include <random>
#include "Mesh.hpp"
#include "Spline.hpp"

int main() {
	Mesh2D* mesh = new Mesh2D;

	// Start with a uniform grid
	mesh->SplitElementInFourths(0, { 0.5, 0.5 });
	mesh->SplitElementInFourths(2, { 0.75, 0.75 });
	mesh->SplitElementInFourths(3, { 0.25, 0.75 });
	mesh->SplitElementInFourths(1, { 0.75, 0.25 });
	mesh->SplitElementInFourths(0, { 0.25, 0.25 });
	// Select random elements to split
	std::random_device random_device;
	std::mt19937 random_engine(random_device());
	std::uniform_real_distribution<float> fl_distr(0.2, 0.8);

	while (mesh->size() <= 400) {
		std::uniform_int_distribution<int> distribution(0, mesh->size() - 1);
		std::vector<uint32_t> to_split(10);
		std::generate(to_split.begin(), to_split.end(), std::bind(distribution, random_engine));
		for (auto elem : to_split) {
			auto bounds = mesh->getHalfEdgeBounds(halfEdge(elem, 0));
			auto bounds1 = mesh->getHalfEdgeBounds(halfEdge(elem, 1));
			float split = bounds.first + fl_distr(random_engine) * (bounds.second - bounds.first);
			float split2 = bounds1.first + fl_distr(random_engine) * (bounds1.second - bounds1.first);
			mesh->SplitElementInFourths(elem, { split, split2 });
		}
	}
	mesh->Visualize();
}
