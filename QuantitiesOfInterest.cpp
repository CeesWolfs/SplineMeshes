#include "QuantitiesOfInterest.hpp"
/**
 * To check properties of initial mesh.
 */
QuantitiesOfInterest::QuantitiesOfInterest(): mesh(), incidence(mesh.getVertices().size(), mesh.getCuboids().size()) {
    //initialize (n x m) incidence matrix with zeros.
    for (int j = 0; j < mesh.getVertices().size(); j++) {
        for (int i = 0; i < mesh.getCuboids().size(); i++) {
            incidence(j, i) = 0;
        }
    }
}

/**
 * To check properties of already existing mesh.
 */
QuantitiesOfInterest::QuantitiesOfInterest(const Mesh& m): incidence(m.getVertices().size(), m.getCuboids().size()) {
    this->mesh = m;

    //initialize (n x m) incidence matrix with zeros.
    for (int j = 0; j < m.getV2lV().size(); j++) {
        for (int i = 0; i < m.getCuboids().size(); i++) {
            incidence(j, i) = 0;
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
    uint32_t idx = -1;
    for (auto i = 0; i < mesh.getVertices().size(); i++) {
        if (mesh.getVertices()[i] == vertex) {
            idx = i;
            break;
        }
    }
    int x = 0;
    //check how many elements are reachable to the given vertex.
    for (int j = 0; j < mesh.getCuboids().size(); j++) {
        for (auto k : mesh.getCuboids()[j].vertices) {
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
 * Indicence matrix which shows connectivity between the elements and their vertices.
 * Insert for each element a 1 if it is connected with a vertex.
 * Row indices (outer vector ids) should represent vertex ids and column indices (inner vector ids) 
 * should represent cuboid ids to which the (row) vertex is connected to. 
 * If there is no connection between the cuboid and vertex, then a 0 is inserted at that position.
 */
const MatrixXf& QuantitiesOfInterest::incidenceMatrix() {
    for (int j = 0; j < mesh.getV2lV().size(); j++) {
        for (int i = 0; i < mesh.getCuboids().size(); i++) {
            if (mesh.getV2lV()[j].getCuboid() == i) {
                for (auto k : mesh.getCuboids()[i].vertices) {
                    if (k == j) {
                        incidence(j, i) = 1;
                    }
                }
            }
        }
    }
    return incidence;
}


/**
 * Destructor
 */
QuantitiesOfInterest::~QuantitiesOfInterest() {
}


