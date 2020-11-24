#include <catch2/catch.hpp>
#include "../SubFaceTree.hpp"

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
	std::vector<halfFace> F2f{ (uint32_t)-1,{0,1},(uint32_t)-1,(uint32_t)-1,(uint32_t)-1,(uint32_t)-1 };
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