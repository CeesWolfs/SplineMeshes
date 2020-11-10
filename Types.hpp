#pragma once
#include <cstdint>
#include <cstdio>
#include <utility>

constexpr float eps = 1e-6;

typedef struct _vertex
{
	float x, y;
} Vertex;

typedef struct _element
{
	uint32_t v1, v2, v3, v4;
} Element;

constexpr uint32_t border_id = -1;
typedef struct _halfEdge
{
	uint32_t id;
	bool operator==(const _halfEdge& other) const {
		return id == other.id;
	}
	char* toStr(char buf[20]) const {
		snprintf(buf,20,"<%d,%d>",this->getElement(), this->getLocalId());
		return buf;
	}
	uint8_t getLocalId() const {
		return id & 0x3;
	}
	uint32_t getElement() const {
		return id >> 2;
	}
	bool isSubdivided() const {
		return (id & (1 << 31)) != 0u;
	}
	bool isBorder() const {
		return this->id == border_id;
	}
	_halfEdge() {}
	_halfEdge(uint32_t face, uint8_t local_id) {
		id = (face << 2) + local_id;
	}
	_halfEdge(uint32_t id_num) : id(id_num) {}
} halfEdge;

enum axis
{
	x,
	y
};
