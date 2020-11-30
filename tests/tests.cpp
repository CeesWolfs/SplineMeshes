#include <catch2/catch.hpp>
#include "../SubFaceTree.hpp"
#include "../Mesh.hpp"
#include "../Types.hpp"



TEST_CASE("A single subface split works as expected", "[SubFaceTree]")
{
  SubFaceTree subfaces;
  std::vector<halfFace> F2f{ (uint32_t)-1,{0,1},(uint32_t)-1,(uint32_t)-1,(uint32_t)-1,(uint32_t)-1 };
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
  std::vector<halfFace> F2f{ (uint32_t)-1,{0,1},(uint32_t)-1,(uint32_t)-1,(uint32_t)-1,(uint32_t)-1 };
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
	std::vector<halfFace> F2f{ (uint32_t)-1,{0,1},(uint32_t)-1,(uint32_t)-1,(uint32_t)-1,(uint32_t)-1 };
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

TEST_CASE("A subface can be changed", "[SubFaceTree]")
{
	SubFaceTree subfaces;
	std::vector<halfFace> F2f{ 6 * 10 };
	std::fill_n(std::back_inserter(F2f), 60, halfFace(-1));
	F2f[1] = subfaces.splitHalfFace({ 1,0 }, { 0,1 }, Axis::x, { 0.5, 0.5, 0.5 }, { 1,0 }, { 5,0 });
	F2f[1] = subfaces.splitHalfFace(F2f[1], { 0,1 }, Axis::y, { 0.25, 0.5, 0.5 }, { 1,0 }, { 6,0 });
	F2f[1] = subfaces.splitHalfFace(F2f[1], { 0,1 }, Axis::y, { 0.75, 0.5, 0.5 }, { 5,0 }, { 7,0 });
	F2f[1] = subfaces.splitHalfFace(F2f[1], { 0,1 }, Axis::x, { 0.75, 0.25, 0.5 }, { 5,0 }, { 8,0 });
	F2f[1] = subfaces.splitHalfFace(F2f[1], { 0,1 }, Axis::y, { 0.75, 0.75, 0.5 }, { 7,0 }, { 9,0 });
	auto it =  subfaces.find(F2f[1], {5,0}, {0.625, 0.25, 0.5});
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
	CHECK(mesh.getV2f().size() == 6);

	for (int i = 0; i < 6; i++) {
		CHECK(mesh.getF2f()[i].id == -1);
		halfFace curr = { 0, (uint8_t)i };
		REQUIRE(mesh.getV2f()[i] == curr);
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


//TODO: This should pass but it throws a vector subscript out of range message when updating the twin or parent.
/**
TEST_CASE("Good behaviour on split along XY plane.", "[Mesh]") 
{
	Mesh mesh;
	CHECK(mesh.SplitAlongXY(0, 0.5) == 1);
	CHECK(mesh.getCuboids().size() == 2);
}
*/