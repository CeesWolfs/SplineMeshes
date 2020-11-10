#pragma once
#include <cstdint>
#include <cstdio>
#include <utility>

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
	const uint8_t getLocalId() const {
		return id & 0x3;
	}
	const uint32_t getElement() const {
		return id >> 2;
	}
	const bool isSubdivided() const {
		return id & (1 << 31);
	}
	const bool isBorder() const {
		return this->id == border_id;
	}
	_halfEdge(uint32_t face, uint8_t local_id) {
		id = (face << 2) + local_id;
	}
	_halfEdge(uint32_t id_num) {
		id = id_num;
	}
} halfEdge;

enum axis
{
	x,
	y
};