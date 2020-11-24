#ifndef _MESH_HPP // Header guard
#define _MESH_HPP
#include <vector>
#include <numeric>
#include <stdint.h>
#include <cstdio>
#include <string>
#include <fstream>
#include <algorithm>
#include "Types.hpp"

class Mesh
{
private:
    /* data */
    std::vector<Vertex> vertices;
    std::vector<Cuboid> cuboids;

    // Stores a mapping of Half faces to twin half faces
    std::vector<halfFace> F2f;
    std::vector<halfFace> V2f;

public:
    /**
     * Returns the twin half face of the given half face "hf"
    */
    halfFace Twin(const halfFace& hf) const;

    /**
     * Gets vertex index of a vertex in the vertices vector if it exists.
     * Otherwise -1 is returned which means that vertex is not in the vertices vector.
     * 
     * Wordt case time complexity = O(n) = bad and inefficient
    */
    bool mergeVertexIfExists(const Vertex& v, uint32_t& vref);

    /*
    * Better merging function
    * //TODO: implement more complicated but much faster function, returns true if merged
    */
    bool Mesh::mergeVertexIfExistsNew(const Vertex& v, uint32_t& vref, const uint32_t cuboid_id, const float split_point, Axis split_axis);

    /**
     * Add function which pushes the new half and twin half faces of the new cuboid in vector F2F
     * 
    */
    void addHalfFaces(const uint32_t cuboid_id, const Axis split_axis);

    /**
     * Create a new cuboid above the split line, and let its bottom face point to top face bottom element
     * Split the four original faces in two, update all twin faces
     * Create 4 new vertices, merge if vertex already exists in neigboring element
     *
    */
    uint32_t SplitAlongXY(uint32_t cuboid_id, float z_split);

    /**
     * Create a new cuboid to the left of split line, and let its right face point to left face right element.
     * Split the four original faces in two, update all twin faces.
     * Create 4 new vertices, merge if vertex already exists in neigboring element.
     *
    */
    uint32_t SplitAlongYZ(uint32_t cuboid_id, float x_split);

    /**
     * w.r.t XZ plane, same needs to happens as the splits defined above but now with another orientation.
     * Split the four original faces in two, update all twin faces.
     * Create 4 new vertices, merge if vertex already exists in neigboring element.
    */
    uint32_t SplitAlongXZ(uint32_t cuboid_id, float y_split);

    /* Constructor of mesh object */
    Mesh();

    /* Destructor of mesh object */
    ~Mesh();
};

#endif