#ifndef _TYPES_HPP // Header guard
#define _TYPES_HPP
#include <utility>
#include <array>
#include <iostream>

// Constants
constexpr float eps = 1e-7;
constexpr uint32_t border_id = static_cast<uint32_t>(-1 & ~0x7);

// Utility functions
template<typename T, typename T2>
static constexpr inline bool contains(const T& collection, T2 val) {
	return std::find(collection.begin(), collection.end(), val) != collection.end();
}

static inline bool floatSame(const float a, const float b) {
	return std::abs(a-b) < eps;
}

// Structs
struct Vertex
{
	float x, y, z;
	bool operator==(const Vertex& other) const {
		const bool same_x = floatSame(this->x, other.x);
		const bool same_y = floatSame(this->y, other.y);
		const bool same_z = floatSame(this->z, other.z);
		return (same_x && same_y && same_z);
	}
	bool operator>=(const Vertex& other) const {
		return (x >= other.x && y >= other.y && z >= other.z);
	}
	Vertex operator+(const Vertex& other) const {
		return { x + other.x, y + other.y, z + other.z };
	}
	Vertex operator-(const Vertex& other) const {
		return { x - other.x, y - other.y, z - other.z };
	} 
	Vertex& operator+=(const Vertex& other) {
		x += other.x;
		y += other.y;
		z += other.z;
		return *this;
	}
	Vertex operator/(float div) const {
		return { x / div, y / div, z / div };
	}
};

struct Edge
{
	uint32_t v1, v2;
	uint32_t elem;
	bool operator==(const Edge& other) const {
		return (v1 == other.v1 && v2 == other.v2) || (v1 == other.v2 && v2 == other.v1);
	}
};

union Cuboid {
	struct
	{
		uint32_t v1, v2, v3, v4, v5, v6, v7, v8;
	};
	std::array<uint32_t, 8> vertices;
};

/*
* Local vertex 
*/
struct localVertex {
	uint32_t id;
	bool operator==(const localVertex& other) const {
		return id == other.id;
	}
	uint8_t getLocalId() const {
		return id & 0x7;
	}
	uint32_t getCuboid() const {
		return id >> 3;
	}
	localVertex(uint32_t cuboid_id, uint8_t local_id) { id = (cuboid_id << 3) + local_id; };
};

/**
 * half face stores both its parent cuboid, and its local id
 * like <cuboid, local_id> e.g. <1,4>. Six faces per cuboid so, local
 * id 0-5 -> stored in lowest three bits. Local id 6 means the half face is
 * a reference to the subhalfface data structure.
*/
struct halfFace
{
	uint32_t id;
	bool operator==(const halfFace& other) const {
		return id == other.id;
	}
	char* toStr(char buf[20]) const {
		snprintf(buf, 20, "<%d,%d>", this->getCuboid(), this->getLocalId());
		return buf;
	}
	uint8_t getLocalId() const {
		return id & 0x7;
	}
	uint32_t getCuboid() const {
		return id >> 3;
	}
	// Check if it is a pointer to a node in the subfacetree. Id 6 incidates that the node is left, id 7 -> right.
	bool isSubdivided() const {
		return getLocalId() >= 6;
	}
	bool isBorder() const {
		return getLocalId() == border_id || id == border_id;
	}
	halfFace(uint32_t cuboid_id, uint8_t local_id) {
		id = (cuboid_id << 3) + local_id;
	}
	halfFace(uint32_t id_num) : id(id_num) {}
};

// Typedefs
typedef std::pair<halfFace, halfFace> HalfFacePair;

//Enums
enum class Axis
{
 	x,
 	y,
    z
};

//Node struct
struct Node
{
	halfFace parent;
	float split_coord;
	Axis split_axis;
	halfFace lower_child;
	halfFace top_child;
};
#endif