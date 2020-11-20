#ifndef _MESH_HPP // Header guard
#define _MESH_HPP
#include <vector>
#include <numeric>
#include <stdint.h>
#include <cstdio>
#include <string>
#include <fstream>
#include "Types.hpp"

class Mesh
{
private:
    /* data */
    std::vector<Vertex> vertices;
    std::vector<Cuboid> cuboids;
    // Stores a mapping of Half faces to twin half faces
    std::vector<halfFace> F2f;

public:
    halfFace Twin(const halfFace hf) const;
    uint32_t SplitAlongXY(uint32_t elem_id, float z_split);
    uint32_t SplitAlongYZ(uint32_t elem_id, float x_split);
    uint32_t SplitAlongXZ(uint32_t elem_id, float y_split);
    Mesh(/* args */);
    ~Mesh();
};

/*
*
*
*
*
*/

#endif