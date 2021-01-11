#ifndef _QUANTITIESOFINTEREST_HPP // Header guard
#define _QUANTITIESOFINTEREST_HPP
#include <vector>
#include <numeric>
#include <stdint.h>
#include <cstdio>
#include <string>
#include <fstream>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "Mesh.hpp"
#include "Types.hpp"

using Eigen::MatrixXf;
typedef Eigen::Triplet<double> Triplet;

/* A vertex is connected to at most 8 elements, so the connectivity information fits in this simple struct */
struct VertexConnectivity {
    std::array<uint32_t, 8> elements;
    uint32_t number;
};

class QuantitiesOfInterest {

    private:
        const Mesh& mesh;
        Eigen::SparseMatrix<bool>  incidence;
    public:
        //Default constructor
        QuantitiesOfInterest();

        //Constructor with mesh
        QuantitiesOfInterest(const Mesh& mesh);

        //getter for mesh
        const Mesh& getMesh() const;

        //The amount of elements connected to the given vertex.
        VertexConnectivity vertexConnectivity(uint32_t vertex) const;

        // Wether a vertex is on the border
        bool isBorderVertex(uint32_t vertex) const;

        //Check if the given element reaches one of the 8 corners of the mesh.
        bool isCornerCuboid(const Cuboid& cuboid);

        //maximal segments of the given start face.
        const std::vector<halfFace> getMaximalSegmentOf(halfFace currFace);

        //Unsigned indicence matrix which shows connectivity between the half faces and their vertices.
        const Eigen::SparseMatrix<bool>& incidenceMatrix();

        //Signed Vertex-Edge incidence matrix which shows whether the difference between the vertex coordinates >= origin
        const MatrixXf VertexEdgeIncidenceMatrix();

        //Get all 12 edges of the given cuboid.
        const std::vector<Edge> getEdges(const Cuboid& cuboid) const;

        // Get all the edges of the mesh.
        const std::vector<Edge> getAllEdges() const;

        //Destructor
        ~QuantitiesOfInterest();


};


#endif