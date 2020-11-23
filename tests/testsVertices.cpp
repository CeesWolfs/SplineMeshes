#include <catch2/catch.hpp>
#include "../Types.hpp"

constexpr Vertex a = { 1.0, 1.9, 2.9 };
constexpr Vertex b = { 1.0, 1.9, 2.9 };
constexpr Vertex c = { 1.0000001, 1.9, 2.9 };
constexpr Vertex d = { 2.9, 100, 3 };

TEST_CASE("test operator==() for equal vertices", "[Vertex]")
{
	REQUIRE(a == b);
}

TEST_CASE("test operator==() for unequal vertices 1", "[Vertex]")
{
	REQUIRE_FALSE(a == c);
}

TEST_CASE("test operator==() for unequal vertices 2", "[Vertex]")
{
	REQUIRE_FALSE(a == d);
}