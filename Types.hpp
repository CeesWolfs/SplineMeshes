#ifndef _TYPES_HPP // Header guard
#define _TYPES_HPP
#include <utility>
#include <array>
#include <iostream>

constexpr float eps = 1e-7;

static inline bool floatSame(const float a, const float b) {
	return std::abs(a-b) < eps;
}

typedef struct _vertex
{
	float x, y, z;
	bool operator==(const _vertex& other) const {
		const bool same_x = floatSame(this->x, other.x);
		const bool same_y = floatSame(this->y, other.y);
		const bool same_z = floatSame(this->z, other.z);
		return (same_x && same_y && same_z);
	}
	bool operator>=(const _vertex& other) const {
		return (x >= other.x && y >= other.y && z >= other.z);
	}
	_vertex operator+(const _vertex& other) const {
		return { x + other.x, y + other.y, z + other.z };
	}
	_vertex operator-(const _vertex& other) const {
		return { x - other.x, y - other.y, z - other.z };
	} 
	_vertex& operator+=(const _vertex& other) {
		x += other.x;
		y += other.y;
		z += other.z;
		return *this;
	}
	_vertex operator/(float div) const {
		return { x / div, y / div, z / div };
	}
} Vertex;

typedef struct _edge 
{
	uint32_t v1, v2;
	bool operator==(const _edge& other) const {
		return (v1 == other.v1 && v2 == other.v2) || (v1 == other.v2 && v2 == other.v1);
	}
	_edge(uint32_t first, uint32_t second) {
		v1 = first;
		v2 = second;
	}
} Edge;

typedef union _cuboid {
	struct
	{
		uint32_t v1, v2, v3, v4, v5, v6, v7, v8;
	};
	std::array<uint32_t, 8> vertices;
} Cuboid;

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
constexpr uint32_t border_id = static_cast<uint32_t>(-1 & ~0x7);

typedef struct _halfFace
{
	uint32_t id;
	bool operator==(const _halfFace& other) const {
		return id == other.id;
	}
	char* toStr(char buf[20]) const {
		snprintf(buf,20,"<%d,%d>",this->getCuboid(), this->getLocalId());
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
	_halfFace(uint32_t cuboid_id, uint8_t local_id) {
		id = (cuboid_id << 3) + local_id;
	}
	_halfFace(uint32_t id_num) : id(id_num) {}
} halfFace;

typedef std::pair<halfFace, halfFace> HalfFacePair;

typedef enum class _axis
{
 	x,
 	y,
    z
} Axis;
#endif