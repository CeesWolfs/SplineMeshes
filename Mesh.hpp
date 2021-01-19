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
#include "SubFaceTree.hpp"

/*
* local Half face id to vertex id
*/
static constexpr std::array<std::array<uint8_t, 4>, 6> Hf2Ve = { {
    {0,1,2,3},
    {4,5,6,7},
    {3,2,6,7},
    {1,2,6,5},
    {0,1,5,4},
    {0,3,7,4}
}};

/* Face to axis */
static constexpr Axis Hf2Ax[6] = { Axis::z, Axis::z, Axis::y, Axis::x, Axis::y, Axis::x };

/*
* local vertex to containing half faces
*/
static constexpr std::array<std::array<uint8_t, 3>, 8> Lv2Hf = { {
    {0, 4, 5},
    {0, 3, 4},
    {0, 2, 3},
    {0, 2, 5},
    {1, 4, 5},
    {1, 3, 4},
    {1, 2, 3},
    {1, 2, 5}
} };

/* 
* Common local vertices between two half faces
*/
static constexpr std::array<std::array<std::array<uint8_t, 2>, 6>, 6> Hf2Clv = {{
    {{{0,1},{-1,-1},{2,3},{1,2},{0,1},{0,3}}},
    {{{-1,-1},{4,5},{6,7},{5,6},{5,4},{4,7}}},
    {{{2,3},{6,7},{3,2},{2,6},{-1,-1},{3,7}}},
    {{{1,2},{5,6},{2,6},{1,2},{1,5},{-1,-1}}},
    {{{0,1},{5,4},{-1,-1},{1,5},{0,1},{0,4}}},
    {{{0,3},{4,7},{3,7},{-1,-1},{0,4},{0,3}}}
}};

/*
* Half faces to check depending on the split along an axis (used in splitting algorithm only).  
* Hfs2Check[0] array contains half faces to check for split along X Axis 
* Hfs2Check[1] array contains half faces to check for split along Y Axis
* Hfs2Check[2] array contains half faces to check for split along Z Axis
*/
static constexpr std::array<std::array<std::array<uint8_t, 2>, 4>, 3> Hfs2Check = {{
    {{ {0,4}, {0,2}, {1,2}, {1,4} }},
    {{ {0,5}, {0,3}, {1,3}, {1,5} }},
    {{ {5,4}, {4,3}, {3,2}, {2,5} }}
}};


class Mesh 
{

private:
    /* data */
    std::vector<Vertex> vertices;
    std::vector<Cuboid> cuboids;
    // Stores a mapping of Half faces to twin half faces
    std::vector<halfFace> F2f;
    // Map vertex IDs to a local vertex within an element that contains the vertex
    std::vector<localVertex> V2lV;
    SubFaceTree sft;

    /*
    * Split a halfFace in two, divide subhalfFaces and update all twins
    */
    void splitHalfFace(const halfFace toSplit, const halfFace lower, const halfFace higher, const Axis split_axis, const Vertex& split_point);
    
    /*
    * Updates a halfFace to a new halfFace, auto update all twins
    */
    void updateHalfFace(const halfFace hf, const halfFace new_hf, const Vertex& middle);
    
    /*
    * Update the twin to point to a new halfFace
    */
    void updateTwin(const halfFace twin, const halfFace old_hf, const halfFace new_hf, const Vertex& middle);

public:

    /* Static helper functions */
    static constexpr uint8_t opposite_face(uint8_t local_id) {
        switch (local_id & 0x7)
        {
        case 0: return 1;
        case 1: return 0;
        case 2: return 4;
        case 4: return 2;
        case 3: return 5;
        case 5: return 3;
        default:
            return 255;
        }
    }

    /**
    * Getters for the private vectors to access them publicly via the Mesh class.
    */
    const std::vector<Vertex>& getVertices() const;
    const std::vector<Cuboid>& getCuboids() const;
    const std::vector<halfFace>& getF2f() const;
    const std::vector<localVertex>& getV2lV() const;
    const SubFaceTree& getSft() const;

    /*
    * Saves the mesh structure in a .ply file format to be used to visualize the mesh
    */
    void Save(const std::string& filename);

    /**
     * Returns a reference to the twin half face of the given half face "hf"
    */
    halfFace& Twin(const halfFace& hf);

    /**
     * Returns a constant reference to the twin half face of the given half face "hf"
    */
    const halfFace& Twin(const halfFace& hf) const;

    /* 
     * Note not an efficient function, used only for testing purposes
     * Tests wether two elements are directly adjacent, that is touch at a face
     */
    bool Adjacent(uint32_t elem1, uint32_t elem2) const;

    /*
    * A simpler generalized find vertex method, finds a vertex on the border of a face, checks all three required elements for the vertex
    */
    std::pair<bool, uint32_t> mergeVertexIfExists(const Vertex& v, HalfFacePair hftoCheck, uint8_t local_id, Axis split_axis);

    /**
     * Add function which pushes the new half and twin half faces of the new cuboid in vector F2F
     * This method adds 6 half faces to the new cuboid.
    */
    void addHalfFaces(const uint32_t cuboid_id, const Axis split_axis);

    /**
     * Split method for splitting cuboid along a given axis and creating the required half faces and necessary updates.
    */
    uint32_t SplitAlongAxis(uint32_t cuboid_id, float split_point, Axis axis);

    /**
     * Alias method that executes split method "SplitAlongAxis" along XY plane
    */
    uint32_t SplitAlongXY(uint32_t cuboid_id, float z_split);

    /**
     * Alias method that executes split method "SplitAlongAxis" along YZ plane
    */
    uint32_t SplitAlongYZ(uint32_t cuboid_id, float x_split);

    /**
     * Alias method that executes split method "SplitAlongAxis" along XZ plane
    */
    uint32_t SplitAlongXZ(uint32_t cuboid_id, float y_split);

    /* Constructor of mesh object */
    Mesh();

    /* Construct a uniform mesh */
    Mesh(int Nx, int Ny, int Nz);

    /* Destructor of mesh object */
    ~Mesh() = default;
};
#endif
