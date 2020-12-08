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

class Mesh
{
private:
    /* data */
    std::vector<Vertex> vertices;
    std::vector<Cuboid> cuboids;
    SubFaceTree sft;
    // Stores a mapping of Half faces to twin half faces
    std::vector<halfFace> F2f;
    // Map vertex IDs to a local vertex within an element that contains the vertex
    std::vector<localVertex> V2lV;

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
    /**
    * Getters for the private vectors to access them publicly via the Mesh class.
    */
    const std::vector<Vertex>& getVertices() const;
    const std::vector<Cuboid>& getCuboids() const;
    const std::vector<halfFace>& getF2f() const;
    const std::vector<localVertex>& getV2lV() const;

    const SubFaceTree& getSft() const;

    void Save(const std::string& filename);

    /**
     * Returns a reference to the twin half face of the given half face "hf"
    */
    halfFace& Twin(const halfFace& hf);
    /**
     * Returns a constant reference to the twin half face of the given half face "hf"
    */
    const halfFace& Twin(const halfFace& hf) const;

    /**
     * Gets vertex index of a vertex in the vertices vector if it exists.
     * Otherwise -1 is returned which means that vertex is not in the vertices vector.
     *
     * Wordt case time complexity = O(n) = bad and inefficient
    */
    bool mergeVertexIfExists(const Vertex& v, uint32_t& vref);

    /*
     * Helper functions for checks that are needed to find vertex of split on x, y and z axises
     */
    // bool seachVertexByAxis(const std::vector<halfFace>& hfts, const Axis splitAxis, const Vertex& toFind) const;
    bool findVertexAxisX(
        const halfFace& hf1,
        const halfFace& hf2,
        const halfFace& hf3,
        const halfFace& hf4,
        const Vertex& vToFind
    ) const;
    bool findVertexAxisY(
        const halfFace& hf1,
        const halfFace& hf2,
        const halfFace& hf3,
        const halfFace& hf4,
        const Vertex& vToFind
    ) const;
    bool findVertexAxisZ(
        const halfFace& hf1,
        const halfFace& hf2,
        const halfFace& hf3,
        const halfFace& hf4,
        const Vertex& vToFind
    ) const;

    /* 
     * Note not an efficient function, used only for testing purposes
     * Tests wether two elements are directly adjacent, that is touch at a face
     */
    bool Adjacent(uint32_t elem1, uint32_t elem2) const;

    /*
    * A simpler generalized find vertex method, finds a vertex on the border of a face, checks all three required elements for the vertex
    */
    std::pair<bool, uint32_t> mergeVertexIfExistsRewrite(const Vertex& v, HalfFacePair hftoCheck, uint8_t local_id, Axis split_axis);


    /*
    *
    * Better merging function. Assign vref to merged vertex.
    * //TODO: implement more complicated but much faster function, returns true if merged
    */
    bool mergeVertexIfExistsNew(
        const Vertex& v,
        uint32_t& vref,
        const uint32_t cuboid_id,
        Axis split_axis
    );

    /**
     * Add function which pushes the new half and twin half faces of the new cuboid in vector F2F
     * This method adds 6 half faces to the new cuboid.
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
    ~Mesh() = default;
};

#endif
