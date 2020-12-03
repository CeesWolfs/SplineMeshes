#ifndef _QUANTITIESOFINTEREST_HPP // Header guard
#define _QUANTITIESOFINTEREST_HPP
#include <vector>
#include <numeric>
#include <stdint.h>
#include <cstdio>
#include <string>
#include <fstream>
#include <algorithm>
#include "Mesh.hpp"
#include "Types.hpp"

class QuantitiesOfInterest {

    private:
        Mesh mesh;

    public:
        //Default constructor
        QuantitiesOfInterest();

        //Constructor with mesh
        QuantitiesOfInterest(const Mesh& mesh);

        //The amount of faces connected to the given vertex.
        int vertexConnectivity(const Vertex& vertex);

        //All maximal segments of the given axis.
        const std::vector<std::vector<halfFace>> maximalSegments(const Axis axis);

        //Indicence matrix which shows connectivity between the half faces and their vertices.
        const std::vector<std::vector<int>> incidenceMatrix();

        //Destructor
        ~QuantitiesOfInterest();


};


#endif