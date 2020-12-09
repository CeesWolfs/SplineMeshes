#include "QuantitiesOfInterest.hpp"
/**
 * To check properties of initial mesh.
 */
QuantitiesOfInterest::QuantitiesOfInterest(): mesh(), incidence(mesh.getVertices().size(), mesh.getCuboids().size()) {}

/**
 * To check properties of already existing mesh.
 */
QuantitiesOfInterest::QuantitiesOfInterest(const Mesh& m): incidence(m.getVertices().size(), m.getCuboids().size()) {
    this->mesh = m;
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
    // TODO implement non O(n) algo
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
 * Return the maximal segment which consists of /starts from the given face id.
 */
const std::vector<uint32_t> QuantitiesOfInterest::getMaximalSegmentOf(const uint32_t startFaceId) {
    //TODO: doesn't add all the correct face ids. 
    std::vector<uint32_t> res;
    const halfFace currFace = mesh.getF2f()[startFaceId];
    res.push_back(startFaceId);
    if (currFace.isBorder() && !currFace.isSubdivided()) return res;
    else {
 /*       const Cuboid currCuboid = mesh.getCuboids()[currFace.getCuboid()];
        auto prev_cub = currFace.getCuboid() - 1;
        auto prevHalfFace = halfFace(prev_cub, currFace.getLocalId());
        auto prevId = prevHalfFace.id;
        while (prev_cub >= 0 && prevId >= 0 && prevId < mesh.getF2f().size())
        {
            if (std::find(res.begin(), res.end(), prevHalfFace.id) == res.end()) {
                std::cout << "adding prev id" << prevHalfFace.id << std::endl;
                res.insert(res.begin(), prevHalfFace.id);
                prev_cub -= 1;
                prevHalfFace = halfFace(prev_cub, currFace.getLocalId());
                prevId = prevHalfFace.id;
            }
        }
        auto next_cub = currFace.getCuboid() + 1;
        auto nextHalfFace = halfFace(next_cub, currFace.getLocalId());
        auto nextId = nextHalfFace.id;
        while (next_cub < mesh.getCuboids().size() && nextId >= 0 && nextId < mesh.getF2f().size()) {
            if (std::find(res.begin(), res.end(), nextHalfFace.id) == res.end()) {
                std::cout << "adding next id" << nextHalfFace.id << std::endl;
                res.push_back(nextHalfFace.id);
                next_cub += 1;
                nextHalfFace = halfFace(next_cub, currFace.getLocalId());
                nextId = nextHalfFace.id;
            }
        }*/

        //TODO: second method also does not seem to get the neighbour half face to the vector. The local face position remains the same while the neighbouring cuboid will change. 
        auto twin = mesh.Twin(currFace);
        while (twin.id >= 0 && twin.id < mesh.getF2f().size()) {
            if (twin.isSubdivided()) {
                for (auto it = mesh.getSft().cbegin(twin); it != mesh.getSft().cend(); ++it)
                {
                    if ((*it).getCuboid() != currFace.getCuboid() && (*it).getLocalId() == currFace.getLocalId()) {
                        res.push_back((*it).id);
                    }
                }
            }
            else {
                if (twin.getCuboid() != currFace.getCuboid() && twin.getLocalId() == currFace.getLocalId()) {
                    res.push_back(twin.id);
                }
            }
            twin = mesh.Twin(twin);
        }
        return res;
    }
}


/**
 * Indicence matrix which shows connectivity between the elements and their vertices.
 * Insert for each element a 1 if it is connected with a vertex.
 * Row indices (outer vector ids) should represent vertex ids and column indices (inner vector ids) 
 * should represent cuboid ids to which the (row) vertex is connected to. 
 * If there is no connection between the cuboid and vertex, then a 0 is inserted at that position.
 * Todo check if needed, and implement faster algorithm
 */
const Eigen::SparseMatrix<bool>& QuantitiesOfInterest::incidenceMatrix() {
    std::vector<Triplet> tripletList;
    tripletList.reserve(mesh.getV2lV().size() * 3);
    for (int j = 0; j < mesh.getV2lV().size(); j++) {
        for (int i = 0; i < mesh.getCuboids().size(); i++) {
            if (mesh.getV2lV()[j].getCuboid() == i) {
                for (auto k : mesh.getCuboids()[i].vertices) {
                    if (k == j) {
                        tripletList.push_back({ j,i,true });
                    }
                }
            }
        }
    }
    incidence.setFromTriplets(tripletList.cbegin(), tripletList.cend());
    return incidence;
}


/**
 * Destructor
 */
QuantitiesOfInterest::~QuantitiesOfInterest() {
}


