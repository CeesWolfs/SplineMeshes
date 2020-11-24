#ifndef _TYPES_HPP // Header guard
#define _TYPES_HPP
#include <utility>
#include <array>
#include <iostream>

constexpr float eps = 1e-7;

typedef struct _vertex
{
	float x, y, z;
	bool operator==(const _vertex& other) const {
		const bool same_x = std::abs(this->x - other.x) < eps;
		const bool same_y = std::abs(this->y - other.y) < eps;
		const bool same_z = std::abs(this->z - other.z) < eps;
		return (same_x && same_y && same_z);
	}
} Vertex;

typedef union _cuboid {
	struct
	{
		uint32_t v1, v2, v3, v4, v5, v6, v7, v8;
	};
	std::array<uint32_t, 8> vertices;
} Cuboid;

/**
 * half face stores both its parent cuboid, and its local id
 * like <cuboid, local_id> e.g. <1,4>. Six faces per cuboid so, local
 * id 0-5 -> stored in lowest three bits. Local id 6 means the half face is
 * a reference to the subhalfface data structure. Local id 7 denotes a border half
*/
constexpr uint32_t border_id = -1;

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
		return getLocalId() == 6 || getLocalId() == 7;
	}
	bool isBorder() const {
		return id == -1;
	}
	_halfFace(uint32_t cuboid_id, uint8_t local_id) {
		id = (cuboid_id << 3) + local_id;
	}
	_halfFace(uint32_t id_num) : id(id_num) {}
} halfFace;

typedef enum class _axis
{
 	x,
 	y,
    z
} Axis;
#endif