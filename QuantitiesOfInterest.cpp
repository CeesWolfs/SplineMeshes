#include "QuantitiesOfInterest.hpp"

/**
 * To check properties of initial mesh.
 */
QuantitiesOfInterest::QuantitiesOfInterest(): mesh() {
}

/**
 * To check properties of already existing mesh.
 */
QuantitiesOfInterest::QuantitiesOfInterest(const Mesh& m) {
    this->mesh = m;
}

/**
 * Returns the amount of half-faces which are connected to/ linked with the given vertex.
 */
int vertexConnectivity(const Vertex& vertex) {
    //TODO:
    return -1;
}

/**
 * All maximal segments of the given axis. 
 * This should return all the sets of connected half-faces in the given direction/axis.
 */
const std::vector<std::vector<halfFace>> maximalSegments(const Axis axis) {
    //TODO:
    const std::vector<std::vector<halfFace>> res;
    return res;
}

/**
 * Indicence matrix which shows connectivity between the half faces and their vertices.
 * Insert for each half face a 1 if it is connected with a vertex. Same holds for twin faces.
 * Row indices (outer vector ids) should represent vertex ids and column indices (inner vector ids) 
 * should represent half-face ids to which the (row) vertex is connected to. 
 * If there is no connection between the half-face and vertex, then a 0 is inserted at that position.
 */
const std::vector<std::vector<int>> incidenceMatrix() {
    //TODO:
    const std::vector<std::vector<int>> res;
    return res;
}


/**
 * Destructor
 */
QuantitiesOfInterest::~QuantitiesOfInterest() {
}


