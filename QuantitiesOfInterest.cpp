#include "QuantitiesOfInterest.hpp"

/**
 * To check properties of initial mesh.
 */
QuantitiesOfInterest::QuantitiesOfInterest(): mesh() {

    //initialize (n x m) incidence matrix with zeros.
    for (int i = 0; i < mesh.getVertices().size(); i++) {
        incidence.push_back({});
    }
    for (int j = 0; j < mesh.getVertices().size(); j++) {
        for (int i = 0; i < mesh.getF2f().size(); i++) {
            incidence[j].push_back(0);
        }
    }
}

/**
 * To check properties of already existing mesh.
 */
QuantitiesOfInterest::QuantitiesOfInterest(const Mesh& m) {
    this->mesh = m;

    //initialize (n x m) incidence matrix with zeros.
    for (int i = 0; i < m.getV2lV().size(); i++) {
        incidence.push_back({});
    }
    for (int j = 0; j < m.getV2lV().size(); j++) {
        for (int i = 0; i < m.getF2f().size(); i++) {
            incidence[j].push_back(0);
        }
    }
}

const Mesh& QuantitiesOfInterest::getMesh() const {
    return mesh;
}

/**
 * Returns the amount of elements which are connected to/ linked with the given vertex.
 */
int QuantitiesOfInterest::vertexConnectivity(const Vertex& vertex) {
    //find index of given vertex in vertices vector
    int idx = -1;
    for (int i = 0; i < mesh.getVertices().size(); i++) {
        if (mesh.getVertices()[i] == vertex) {
            idx = i;
            break;
        }
    }
    int x = 0;
    //check how many elements are reachable to the given vertex.
    for (int j = 0; j < mesh.getCuboids().size(); j++) {
        for (int k : mesh.getCuboids()[j].vertices) {
            if (idx == k) {
                x++;
            }
        }
    }
    return x;
}

/**
 * All maximal segments of the given axis. 
 * This should return all the sets of connected half-faces in the given direction/axis.
 */
const std::vector<std::vector<halfFace>> QuantitiesOfInterest::maximalSegments(const Axis axis) {
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
const std::vector<std::vector<int>> QuantitiesOfInterest::incidenceMatrix() {
    //TODO:
    const std::vector<std::vector<int>> res;
    return res;
}


/**
 * Destructor
 */
QuantitiesOfInterest::~QuantitiesOfInterest() {
}


