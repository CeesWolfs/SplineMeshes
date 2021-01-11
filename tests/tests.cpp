#include <catch2/catch.hpp>
#include "../SubFaceTree.hpp"
#include "../Mesh.hpp"
#include "../Types.hpp"
#include "../QuantitiesOfInterest.hpp"
#include "../SplineMesh.hpp"

namespace SanityChecks {
	/*
	* Check if element A touches B, that B touches A 
	*/
	bool AllAdjacent(const Mesh& mesh) {
		for (size_t cub = 0; cub < mesh.getCuboids().size(); ++cub)
		{
			for (size_t i = 0; i < 6; i++)
			{
				auto twin = mesh.Twin(halfFace(cub, i));
				if (twin.isBorder()) continue;
				if (twin.isSubdivided()) {
					for (auto it = mesh.getSft().cbegin(twin); it != mesh.getSft().cend(); ++it)
					{
						if (!mesh.Adjacent((*it).getCuboid(), cub)) {
							std::cout << "Failed at " << '<' << cub << ',' << i << '>' << '-' << '<' << (*it).getCuboid() << ',' << (int)((*it).getLocalId()) << '>';
							return false;
						}
					}
				}
				else {
					if (!mesh.Adjacent(twin.getCuboid(), cub)) {
						return false;
					}
				}
			}
		}
		return true;
	}
}

namespace helpers {
	Mesh* random_mesh(const int N) {
		// Start with uniform mesh
		Mesh* mesh = new Mesh(10,10,10);
		// Select random elements to split
		std::random_device random_device;
		std::mt19937 random_engine(random_device()); 
		std::uniform_real_distribution<float> fl_distr(0.2, 0.8);

		while(mesh->getCuboids().size() < N) {
			std::uniform_int_distribution<int> distribution(0, mesh->getCuboids().size()-1);
			std::vector<uint32_t> to_split(20);
			std::generate(to_split.begin(),to_split.end(), std::bind(distribution, random_engine));
			for(auto elem : to_split) {
				auto rand = fl_distr(random_engine);
				auto bl_corner = mesh->getVertices()[mesh->getCuboids()[elem].v1];
				auto tr_corner = mesh->getVertices()[mesh->getCuboids()[elem].v7];
				if(rand < 0.4) {
					mesh->SplitAlongXY(elem, fl_distr(random_engine)*(tr_corner.z-bl_corner.z) + bl_corner.z);
				}
				else if(rand < 0.6) {
					mesh->SplitAlongXZ(elem, fl_distr(random_engine)*(tr_corner.y-bl_corner.y) + bl_corner.y);
				}
				else {
					mesh->SplitAlongYZ(elem, fl_distr(random_engine)*(tr_corner.x-bl_corner.x) + bl_corner.x);
				}
			}
		}
		return mesh;
	}
} // helpers


TEST_CASE("Test for basic quantities of interest constructor") {
	Mesh mesh;
	mesh.SplitAlongXY(0, 0.5);
	QuantitiesOfInterest q(mesh);
	const bool checkVertices = q.getMesh().getVertices().size() == mesh.getVertices().size();
	const bool checkCuboids = q.getMesh().getCuboids().size() == mesh.getCuboids().size();
	const bool checkF2f = q.getMesh().getF2f().size() == mesh.getF2f().size();
	const bool checkV2lV = q.getMesh().getV2lV().size() == mesh.getV2lV().size();
	CHECK(checkVertices);
	CHECK(checkCuboids);
	CHECK(checkF2f);
	CHECK(checkV2lV);
}

TEST_CASE("Test for basic quantities of interest vertex connectivity") {
	Mesh mesh;
	QuantitiesOfInterest q(mesh);
	//Check that border vertices are connected to 1 cuboid only.
	for (int i = 0; i < 8; i++) {
		const Vertex vertex = mesh.getVertices()[i];
		CHECK(q.vertexConnectivity(vertex) == 1);
	}
}

TEST_CASE("Test for divided quantities of interest vertex connectivity") {
	Mesh mesh;
	mesh.SplitAlongXY(0, 0.5);
	QuantitiesOfInterest q(mesh);
	//Check that split vertex is connected to 2 cuboids.
	const Vertex vertex = mesh.getVertices()[10];
	CHECK(q.vertexConnectivity(vertex) == 2);
}

TEST_CASE("A single subface split works as expected", "[SubFaceTree]")
{
  SubFaceTree subfaces;
  std::vector<halfFace> F2f{ border_id,{0,1},border_id,border_id,border_id,border_id };
  F2f[1] = subfaces.splitHalfFace({ 1,0 }, { 0,1 }, Axis::x, { 0.5, 0.5, 0.5 }, { 1,0 }, { 5,0 });
  std::vector<halfFace> subFaces;
  for (auto it = subfaces.begin(F2f[1]); it != subfaces.end(); ++it) {
	  subFaces.push_back(*it);
  }
  CHECK(subFaces.size() == 2);
  CHECK(subFaces[0] == halfFace({1, 0}));
  CHECK(subFaces[1] == halfFace({ 5, 0 }));
}

TEST_CASE("A subface can be split twice", "[SubFaceTree]")
{
  SubFaceTree subfaces;
  std::vector<halfFace> F2f{ border_id,{0,1},border_id,border_id,border_id,border_id };
  F2f[1] = subfaces.splitHalfFace({ 1,0 }, { 0,1 }, Axis::x, { 0.5, 0.5, 0.5 }, { 1,0 }, { 5,0 });
  F2f[1] = subfaces.splitHalfFace(F2f[1], { 0,1 }, Axis::y, { 0.25, 0.5, 0.5 }, { 1,0 }, { 6,0 });
  std::vector<halfFace> subFaces;
  for (auto it = subfaces.begin(F2f[1]); it != subfaces.end(); ++it) {
	  subFaces.push_back(*it);
  }
  CHECK(subFaces.size() == 3);
  CHECK(subFaces[0] == halfFace({1, 0}));
  CHECK(subFaces[1] == halfFace({6, 0}));
  CHECK(subFaces[2] == halfFace({5, 0}));
}

TEST_CASE("A subface can be split a few times", "[SubFaceTree]")
{
	SubFaceTree subfaces;
	std::vector<halfFace> F2f{ border_id,{0,1},border_id,border_id,border_id,border_id };
	F2f[1] = subfaces.splitHalfFace({ 1,0 }, { 0,1 }, Axis::x, { 0.5, 0.5, 0.5 }, { 1,0 }, { 5,0 });
	F2f[1] = subfaces.splitHalfFace(F2f[1], { 0,1 }, Axis::y, { 0.25, 0.5, 0.5 }, { 1,0 }, { 6,0 });
	F2f[1] = subfaces.splitHalfFace(F2f[1], { 0,1 }, Axis::y, { 0.75, 0.5, 0.5 }, { 5,0 }, { 7,0 });
	F2f[1] = subfaces.splitHalfFace(F2f[1], { 0,1 }, Axis::x, { 0.75, 0.25, 0.5 }, { 5,0 }, { 8,0 });
	F2f[1] = subfaces.splitHalfFace(F2f[1], { 0,1 }, Axis::y, { 0.75, 0.75, 0.5 }, { 7,0 }, { 9,0 });
	std::vector<halfFace> subFaces;
	for (auto it = subfaces.begin(F2f[1]); it != subfaces.end(); ++it) {
		subFaces.push_back(*it);
	}
	CHECK(std::find(subFaces.begin(), subFaces.end(), halfFace({ 9,0 })) != subFaces.end());
	CHECK(subFaces.size() == 6);
}

TEST_CASE("Vertices can be found on the border of a subface tree", "[SubFaceTree]") {
	SubFaceTree subfaces;
	std::vector<halfFace> F2f;
	std::fill_n(std::back_inserter(F2f), 5*6, halfFace(border_id));
	F2f[1] = subfaces.splitHalfFace({ 1,0 }, { 0,1 }, Axis::y, { 0.5, 0.5, 0.5 }, { 1,0 }, { 2,0 });
	F2f[1] = subfaces.splitHalfFace(F2f[1], { 0,1 }, Axis::x, { 0.5, 0.25, 0.5 }, { 1,0 }, { 3,0 });
	F2f[1] = subfaces.splitHalfFace(F2f[1], { 0,1 }, Axis::x, { 0.5, 0.75, 0.5 }, { 2,0 }, { 4,0 });
	halfFace face{border_id};
	CHECK(subfaces.findVertexBorder(F2f[1], { 1.0, 0.5, .5 }, Axis::y, face));
	CHECK(face == halfFace(3, 0));
}

TEST_CASE("A subface can be changed", "[SubFaceTree]")
{
	SubFaceTree subfaces;
	std::vector<halfFace> F2f{ 6 * 10 };
	std::fill_n(std::back_inserter(F2f), 60, halfFace(border_id));
	F2f[1] = subfaces.splitHalfFace({ 1,0 }, { 0,1 }, Axis::x, { 0.5, 0.5, 0.5 }, { 1,0 }, { 5,0 });
	F2f[1] = subfaces.splitHalfFace(F2f[1], { 0,1 }, Axis::y, { 0.25, 0.5, 0.5 }, { 1,0 }, { 6,0 });
	F2f[1] = subfaces.splitHalfFace(F2f[1], { 0,1 }, Axis::y, { 0.75, 0.5, 0.5 }, { 5,0 }, { 7,0 });
	F2f[1] = subfaces.splitHalfFace(F2f[1], { 0,1 }, Axis::x, { 0.75, 0.25, 0.5 }, { 5,0 }, { 8,0 });
	F2f[1] = subfaces.splitHalfFace(F2f[1], { 0,1 }, Axis::y, { 0.75, 0.75, 0.5 }, { 7,0 }, { 9,0 });
	auto it =  subfaces.find(F2f[1], {0.625, 0.25, 0.5});
	*it = {3,0};
	std::vector<halfFace> subFaces;
	for (auto it = subfaces.begin(F2f[1]); it != subfaces.end(); ++it) {
		subFaces.push_back(*it);
	}
	CHECK(std::find(subFaces.begin(), subFaces.end(), halfFace({ 3,0 })) != subFaces.end());
	CHECK(subFaces.size() == 6);
}

TEST_CASE("A subface tree can be split in two", "[SubFaceTree]")
{
	SubFaceTree subfaces;
	std::vector<halfFace> F2f{ 6 * 10 };
	std::fill_n(std::back_inserter(F2f), 60, halfFace(1,0));
	F2f[1] = subfaces.splitHalfFace({ 1,0 }, { 0,1 }, Axis::x, { 0.5, 0.5, 0.5 }, { 1,0 }, { 5,0 });
	F2f[1] = subfaces.splitHalfFace(F2f[1], { 0,1 }, Axis::y, { 0.25, 0.5, 0.5 }, { 1,0 }, { 6,0 });
	F2f[1] = subfaces.splitHalfFace(F2f[1], { 0,1 }, Axis::y, { 0.75, 0.5, 0.5 }, { 5,0 }, { 7,0 });
	F2f[1] = subfaces.splitHalfFace(F2f[1], { 0,1 }, Axis::x, { 0.75, 0.25, 0.5 }, { 5,0 }, { 8,0 });
	F2f[1] = subfaces.splitHalfFace(F2f[1], { 0,1 }, Axis::y, { 0.75, 0.75, 0.5 }, { 7,0 }, { 9,0 });
	auto split_res = subfaces.splitTree(F2f[1], Axis::x, { 0.6, 0.5, 0.5 }, { 1,0 }, { 2,0 }, F2f);
	subfaces.updateParent(split_res.first, { 1,0 });
	subfaces.updateParent(split_res.second, { 2,0 });
	std::vector<halfFace> subFaces;
	for (auto it = subfaces.begin(split_res.first); it != subfaces.end(); ++it) {
		subFaces.push_back(*it);
	}
	CHECK(subFaces.size() == 5);
	subFaces.clear();
	for (auto it = subfaces.begin(split_res.second); it != subfaces.end(); ++it) {
		subFaces.push_back(*it);
	}
	CHECK(subFaces.size() == 4);
}

TEST_CASE("A test for the mesh constructor which should initialize the cuboid with the initial vertices and faces.", "[Mesh]")
{
	Mesh mesh;
	CHECK(mesh.getCuboids().size() == 1);
	CHECK(mesh.getCuboids()[0].v1 == 0);
	CHECK(mesh.getCuboids()[0].v2 == 1);
	CHECK(mesh.getCuboids()[0].v3 == 2);
	CHECK(mesh.getCuboids()[0].v4 == 3);
	CHECK(mesh.getCuboids()[0].v5 == 4);
	CHECK(mesh.getCuboids()[0].v6 == 5);
	CHECK(mesh.getCuboids()[0].v7 == 6);
	CHECK(mesh.getCuboids()[0].v8 == 7);
	CHECK(mesh.getVertices().size() == 8);
	const Vertex a = { 0.0, 0.0, 0.0 };
	const Vertex getA = mesh.getVertices()[0];
	REQUIRE(getA == a);
	const Vertex b = { 1.0, 0.0, 0.0 };
	const Vertex getB = mesh.getVertices()[1];
	REQUIRE(getB == b);
	const Vertex c = { 1.0, 1.0, 0.0 };
	const Vertex getC = mesh.getVertices()[2];
	REQUIRE(getC == c); 
	const Vertex d = { 0.0, 1.0, 0.0 };
	const Vertex getD = mesh.getVertices()[3];
	REQUIRE(getD == d);
	const Vertex e = { 0.0, 0.0, 1.0 };
	const Vertex getE = mesh.getVertices()[4];
	REQUIRE(getE == e);
	const Vertex f = { 1.0, 0.0, 1.0 };
	const Vertex getF = mesh.getVertices()[5];
	REQUIRE(getF == f);
	const Vertex g = { 1.0, 1.0, 1.0 };
	const Vertex getG = mesh.getVertices()[6];
	REQUIRE(getG == g);
	const Vertex h = { 0.0, 1.0, 1.0 };
	const Vertex getH = mesh.getVertices()[7];
	REQUIRE(getH == h);

	CHECK(mesh.getF2f().size() == 6);
	CHECK(mesh.getV2lV().size() == 8);

	for (int i = 0; i < 6; i++) {
		CHECK(mesh.getF2f()[i].id == border_id);
	}
	for (uint8_t i = 0; i < 8; i++)
	{
		CHECK(mesh.getV2lV()[i] == localVertex(0, i));
	}

}

TEST_CASE("Bad behaviour splits on different axis.", "[Mesh]")
{
	Mesh mesh1;
	//bad behaviours on XY plane
	CHECK(mesh1.SplitAlongXY(0, 1.6) == -1);
	CHECK(mesh1.SplitAlongXY(0, -1.4) == -1);

	//bad behaviours on XZ plane
	CHECK(mesh1.SplitAlongXZ(0, 1.8) == -1);
	CHECK(mesh1.SplitAlongXZ(0, -0.5) == -1);
	
	//bad behaviours on YZ plane
	CHECK(mesh1.SplitAlongYZ(0, 1.7) == -1);
	CHECK(mesh1.SplitAlongYZ(0, -1.2) == -1);
}

TEST_CASE("Good behaviour on a single split along XY plane.", "[Mesh]") 
{
	Mesh mesh;
	CHECK(mesh.SplitAlongXY(0, 0.5) == 1);
	CHECK(mesh.getCuboids().size() == 2);
	SECTION("Number of vertices is correct") {
		CHECK(mesh.getVertices().size() == 12);
	}
	SECTION("The two elements touch eachother") {
		CHECK(mesh.Twin({ 0,1 }) == halfFace(1, 0));
	}
	mesh.Save("OneSplitXY");
}

TEST_CASE("Good behaviour on a single split along YZ plane.", "[Mesh]")
{
	Mesh mesh;
	CHECK(mesh.SplitAlongYZ(0, 0.5) == 1);
	CHECK(mesh.getCuboids().size() == 2);
	SECTION("Number of vertices is correct") {
		CHECK(mesh.getVertices().size() == 12);
	}
	SECTION("The two elements touch eachother") {
		CHECK(mesh.Twin({ 0,3 }) == halfFace(1, 5));
	}
	mesh.Save("OneSplitYZ");
}

TEST_CASE("Good behaviour on a single split along XZ plane.", "[Mesh]")
{
	Mesh mesh;
	CHECK(mesh.SplitAlongXZ(0, 0.5) == 1);
	CHECK(mesh.getCuboids().size() == 2);
	SECTION("Number of vertices is correct") {
		CHECK(mesh.getVertices().size() == 12);
	}
	SECTION("The two elements touch eachother") {
		CHECK(mesh.Twin({ 0,2 }) == halfFace(1, 4));
	}
	mesh.Save("OneSplitXZ");
}
TEST_CASE("Split a cube equally into fourths") {
	Mesh mesh;
	CHECK(mesh.SplitAlongXZ(0, 0.5) == 1);
	CHECK(mesh.SplitAlongXY(1, 0.5) == 2);
	CHECK(mesh.SplitAlongXY(0, 0.5) == 3);
	SECTION("Number of vertices is correct") {
		CHECK(mesh.getVertices().size() == 18);
	}
	SECTION("Sanity check, touching faces is commutative") {
		CHECK(SanityChecks::AllAdjacent(mesh));
	}
	QuantitiesOfInterest q(mesh);
	CHECK(q.vertexConnectivity(mesh.getVertices()[12]) == 4);
	CHECK(q.vertexConnectivity(mesh.getVertices()[13]) == 4);
	//std::cout << q.incidenceMatrix() << std::endl;
	mesh.Save("Fourths");
}



TEST_CASE("Bad behaviour for 2 splits at the same split point on the split axis along the XY plane.") {
	Mesh mesh;
	CHECK(mesh.SplitAlongXY(0, 0.5) == 1);
	CHECK(mesh.SplitAlongXY(1, 0.5) == -1);
}

TEST_CASE("Bad behaviour for 2 splits at the same split point on the split axis along the XZ plane.") {
	Mesh mesh;
	CHECK(mesh.SplitAlongXZ(0, 0.5) == 1);
	CHECK(mesh.SplitAlongXZ(1, 0.5) == -1);
}

TEST_CASE("Bad behaviour for 2 splits at the same split point on the split axis along the YZ plane.") {
	Mesh mesh;
	CHECK(mesh.SplitAlongYZ(0, 0.5) == 1);
	CHECK(mesh.SplitAlongYZ(1, 0.5) == -1);
}

TEST_CASE("Multiple splits on different axis (simultanuously) combined (16 cuboids in total)") {
	Mesh mesh;
	double step = 0.06;
	for (int i = 0; i < 15; i += 3) {
		CHECK(mesh.SplitAlongXY(i, (i+1) * step) == i + 1);
		CHECK(mesh.SplitAlongXZ(i+1, (i+1) * step) == i + 2);
		CHECK(mesh.SplitAlongYZ(i+2, (i+1) * step) == i + 3);
	}
	CHECK(mesh.getCuboids().size() == 16);
	CHECK(SanityChecks::AllAdjacent(mesh));
	mesh.Save("sixteen_cuboids");
}

TEST_CASE("Multiple splits on different axis with splitting for each axis done separately (again, 16 cuboids in total)") {
	Mesh mesh;
	double step = 0.06;
	for (int i = 0; i < 5; i++) {
		CHECK(mesh.SplitAlongXY(i, (i + 1) * step) == i + 1);
	}
	for (int i = 5; i < 10; i++) {
		CHECK(mesh.SplitAlongXZ(i, (i + 1) * step) == i + 1);
	}
	for (int i = 10; i < 15; i++) {
		CHECK(mesh.SplitAlongYZ(i, (i + 1) * step) == i + 1);
	}
	CHECK(mesh.getCuboids().size() == 16);
	CHECK(SanityChecks::AllAdjacent(mesh));
	mesh.Save("separate_axis_splitted_cuboids");
}

TEST_CASE("Simplest case for findVertexRewrite") {
	Mesh mesh;
	mesh.SplitAlongXY(0, 0.5); // Split cube in two
	mesh.SplitAlongYZ(0, 0.5); // Spit the botomm again in two
	// See if we can find a vertex
	auto [found, id] = mesh.mergeVertexIfExists({ 0.5, 0 ,0.5 }, { halfFace(1, 0), halfFace(1, 4) }, 1, Axis::x);
	CHECK(found);
	CHECK(mesh.getVertices()[id] == Vertex({ 0.5, 0, 0.5 }));
}


TEST_CASE("Testing splitree") {
	Mesh mesh;
	mesh.SplitAlongXY(0, 0.5); // Split cube in two
	mesh.SplitAlongXZ(0, 0.4); // Spit the botomm again in two
	mesh.SplitAlongXZ(1, 0.3);
	uint32_t foo = mesh.SplitAlongYZ(1, 0.7);
	mesh.SplitAlongXZ(foo, 0.1);
	mesh.SplitAlongYZ(0, 0.2);
	CHECK(SanityChecks::AllAdjacent(mesh));
	
}

// Test splitting a bigface
TEST_CASE("4 by 4 split on the left half") {
	Mesh mesh;
	CHECK(mesh.SplitAlongXZ(0, 0.25) == 1);
	CHECK(mesh.SplitAlongXZ(1, 0.5) == 2);
	CHECK(mesh.SplitAlongXZ(2, 0.75) == 3);
	CHECK(mesh.SplitAlongXY(0, 0.75) == 4);
	CHECK(mesh.SplitAlongXY(0, 0.5) == 5);
	CHECK(mesh.SplitAlongXY(0, 0.25) == 6);
	CHECK(mesh.SplitAlongYZ(2, 0.75) == 7);
	CHECK(mesh.SplitAlongYZ(2, 0.5) == 8);
	CHECK(mesh.SplitAlongYZ(2, 0.25) == 9);
	CHECK(mesh.SplitAlongXY(1, 0.35) == 10);
	CHECK(SanityChecks::AllAdjacent(mesh));

	//check some complex vertex connectivities.
	// This mesh has 3 CORNER CUBOIDS.
	QuantitiesOfInterest q(mesh);
	int x = 0;
	for (int i = 0; i < mesh.getCuboids().size(); i++) {
		if (q.isCornerCuboid(mesh.getCuboids()[i])) {
			x++;
		}
	}
	CHECK(x == 3);
	//Check that split vertex is connected to 3 cuboids.
	const Vertex vertex = mesh.getVertices()[30];
	//TODO: Another bug: mesh vertex with id 30 is connected to 3 cuboids in the visualization, but only connects to 2 of them.
	//CHECK(q.vertexConnectivity(vertex) == 3);
	mesh.Save("eq_cuboids");
}

TEST_CASE("Subdived connect to subdivided") {
	Mesh mesh;
	mesh.SplitAlongYZ(0, 0.5);
	mesh.SplitAlongXY(1, 0.5);
	mesh.SplitAlongXZ(1, 0.5);
	mesh.SplitAlongXZ(2, 0.5);
	uint32_t middle = mesh.SplitAlongXY(0, 0.2);
	mesh.SplitAlongXY(middle, 0.8);
	middle = mesh.SplitAlongXZ(middle, 0.2);
	mesh.SplitAlongXZ(middle, 0.8);
	middle = mesh.SplitAlongYZ(middle, 0.25);
	SanityChecks::AllAdjacent(mesh);
	mesh.Save("Subdiv_Subdiv");
}

TEST_CASE("Try to invoke tricky find vertex behaviour") {
	Mesh mesh;
	mesh.SplitAlongYZ(0, 0.5);
	mesh.SplitAlongXY(1, 0.5);
	mesh.SplitAlongXZ(1, 0.5);
	mesh.SplitAlongXZ(2, 0.5);
	uint32_t middle = mesh.SplitAlongXY(0, 0.2);
	mesh.SplitAlongXY(middle, 0.8);
	middle = mesh.SplitAlongXZ(middle, 0.2);
	mesh.SplitAlongXZ(middle, 0.8);
	middle = mesh.SplitAlongYZ(middle, 0.25);
	mesh.SplitAlongXY(middle, 0.5);
	CHECK(SanityChecks::AllAdjacent(mesh));
	mesh.Save("Tricky_findvertex");
}

TEST_CASE("Test the free list") {
	Mesh mesh;
	mesh.SplitAlongYZ(0, 0.5);
	mesh.SplitAlongXY(1, 0.5);
	mesh.SplitAlongXZ(1, 0.5);
	mesh.SplitAlongXZ(2, 0.5);
	uint32_t top = mesh.SplitAlongXY(0, 0.5);
	mesh.SplitAlongXY(top, 0.5);
	mesh.SplitAlongXZ(0, 0.5);
	mesh.SplitAlongXZ(top, 0.5);
	mesh.SplitAlongXY(0, 0.25);
	mesh.SplitAlongXZ(0, 0.25);
	CHECK(SanityChecks::AllAdjacent(mesh));
	REQUIRE(mesh.getSft().nodes.size() == 4);
	mesh.Save("Free_list");
}

TEST_CASE("Construct a uniform mesh") {
	Mesh mesh = Mesh(10,10,10);
	CHECK(SanityChecks::AllAdjacent(mesh));
	REQUIRE(mesh.getCuboids().size() == 1000);
	mesh.Save("Uniform_10x10x10");
}

TEST_CASE("Construct semi large random mesh") {
	Mesh* mesh = helpers::random_mesh(5000);
	CHECK(SanityChecks::AllAdjacent(*mesh));
	REQUIRE(mesh->getCuboids().size() == 5000);
	mesh->Save("Random_5000");
}

TEST_CASE("Check incidence matrix of initial cuboid") {
	//initial cuboid: 1 cuboid and 8 vertices, so a 8 x 1 matrix, all with ones, since all vertices are connected to the cuboid.
	Mesh mesh;
	QuantitiesOfInterest q(mesh);
	for (int i = 0; i < 8; i++) {
		CHECK(q.incidenceMatrix().coeff(i,0) == true);
	}
}

TEST_CASE("Check the vertex edge incidence matrix of initial mesh")
{
	Mesh mesh;
	QuantitiesOfInterest q(mesh);
	const MatrixXf m = q.VertexEdgeIncidenceMatrix();
	const Eigen::FullPivLU<MatrixXf> lu_decomp(m);
	//Checking the property that the rank of the vertex-edge incidence matrix is less or equal than the amount of vertices minus one.
	CHECK(lu_decomp.rank() <= mesh.getVertices().size() - 1);
	for (auto i = 0; i < q.getAllEdges().size(); i++) {
		//Check that entries from vertex 0 are all -1, since vertex 0 is exactly located at the origin, so there are no vertices further away from vertex 0.
		if (m(i, 0) != 0) {
			CHECK(m(i, 0) == -1);
		}
		if (m(i, 6) != 0) {
			CHECK(m(i, 6) == 1);
		}
		//Check that entries from vertex 6 are all 1, since vertex 6 is the furthest away from the origin, so there are no vertices further away from the origin compared to vertex 6.
	}
	mesh.Save("init");
}

TEST_CASE("Check border elements") {
	Mesh mesh;
	QuantitiesOfInterest q(mesh);
	CHECK(q.isCornerCuboid(mesh.getCuboids()[0]) == true);

}

TEST_CASE("Test non-divided case for maximal segments") {
	Mesh mesh;
	QuantitiesOfInterest q(mesh);
	// Should contain one single face, since there is no split at all.
	for (uint32_t i = 0; i < 6; i++) {
		CHECK(q.getMaximalSegmentOf(halfFace(0, i)).size() == 1);
	}
}
TEST_CASE("Test simple divided case for maximal segments") {
	Mesh mesh;
	mesh.SplitAlongXY(0, 0.5);
	QuantitiesOfInterest q(mesh);
	CHECK(q.getMaximalSegmentOf(halfFace(0, 2)).size() == 2);
}

TEST_CASE("Test somewhat non trivial case for maximal segments") {
	Mesh mesh(3,3,3);
	mesh.SplitAlongXY(1, 0.1);
	mesh.SplitAlongXY(0, 0.2);
	QuantitiesOfInterest q(mesh);
	CHECK(q.getMaximalSegmentOf(halfFace(0, 0)).size() == 3);
}

TEST_CASE("Test non trivial case for maximal segments") {
	Mesh mesh(3, 3, 3);
	mesh.SplitAlongXY(2, 0.2);
	mesh.SplitAlongYZ(1, 0.5);
	mesh.SplitAlongXY(0, 0.1);
	QuantitiesOfInterest q(mesh);
	mesh.Save("max_segments_test");
	CHECK(q.getMaximalSegmentOf(halfFace(0, 0)).size() == 4);
	for (const auto hf : q.getMaximalSegmentOf(halfFace(0, 0)))
	{
		std::cout << '<' << hf.getCuboid() << ',' << static_cast<int>(hf.getLocalId()) << '>';
	}
}

TEST_CASE("Spline degree for cartesian mesh is correct") {
	SplineMesh<3, 2> splines(2,2,3);
	int N_x = (3 + 1) * 2 - (2 + 1) * (2 - 1);
	int N_y = (3 + 1) * 2 - (2 + 1) * (2 - 1);
	int N_z = (3 + 1) * 3 - (2 + 1) * (3 - 1);
	auto System = splines.generateGlobalMatrix();
	auto QR = Eigen::FullPivHouseholderQR<Eigen::MatrixXf>(System.transpose());
	auto Q = QR.matrixQ();
	auto kernel = Q.block(0, QR.rank(), Q.rows(), Q.cols() - QR.rank());
	CHECK(kernel.cols() == (N_x*N_y*N_z));
}
