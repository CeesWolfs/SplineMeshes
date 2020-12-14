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

class QuantitiesOfInterest {

    private:
        Mesh mesh;
        Eigen::SparseMatrix<bool>  incidence;

    public:
        //Default constructor
        QuantitiesOfInterest();

        //Constructor with mesh
        QuantitiesOfInterest(const Mesh& mesh);

        //getter for mesh
        const Mesh& getMesh() const;

        //The amount of elements connected to the given vertex.
        int vertexConnectivity(const Vertex& vertex);

        //Check if the given element reaches one of the 8 corners of the mesh.
        bool isCornerCuboid(const Cuboid& cuboid);

        //maximal segments of the given start face.
        const std::vector<halfFace> getMaximalSegmentOf(halfFace currFace);

        //Indicence matrix which shows connectivity between the half faces and their vertices.
        const Eigen::SparseMatrix<bool>& incidenceMatrix();

        //Destructor
        ~QuantitiesOfInterest();


};


#endif