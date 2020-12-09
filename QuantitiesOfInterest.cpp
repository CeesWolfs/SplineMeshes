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

static constexpr std::pair<uint8_t, uint8_t> axesToCheck(uint8_t local_face_id) {
    switch (local_face_id) {
        case 0: return <3,2>;
        case 1: return <3,2>;
        case 2: return <1,3>;
        case 4: return <1,3>;
        case 3: return <1,2>;
        case 5: return <1,2>;
    }
}

/**
 * Return the maximal segment which consists of /starts from the given face id.
 */
const std::vector<halfFace> QuantitiesOfInterest::getMaximalSegmentOf(halfFace currFace) {
    //TODO: doesn't add all the correct face ids. 
    std::vector<halfFace> res;
    if (currFace.isBorder()) return res;
    res.push_back(currFace);
    const auto dirs_to_check = axesToCheck(currFace.getLocalId());

    const auto perfect_match = [](halfFace hf, halfFace twin) {
        // TODO Check if only split in axis that doesnt matter
        if(hf.isSubdivided()) return false;
        return (mesh.Twin(twin) == hf);
    }

    const auto gotoAdjacent = [&](halfFace next) {
        auto next_elem = mesh.Twin(next);
        if(!perfect_match(next, next_elem) || next_elem.isBorder()) return border_id;
        return halfFace(next_elem, currFace.getLocalId());
    };

    // Check the first direction in the positive direction
    auto next = gotoAdjacent(halfFace(currFace.getCuboid(), dirs_to_check[0]));
    while (!next.isBorder()) {
        res.push_back(next);
        gotoAdjacent(halfFace(next.getCuboid(), dirs_to_check[0]))
    }
    // Check the first direction in the negative direction
    auto prev = gotoAdjacent(halfFace(currFace.getCuboid(), opposite_face(dirs_to_check[0])));
    while (!prev.isBorder()) {
        res.insert(res.begin(), prev);
        gotoAdjacent(halfFace(prev.getCuboid(), opposite_face(dirs_to_check[0])));
    }

    std::vector<halfFace> other_dir;
    // Check the first direction in the positive direction
    auto next = gotoAdjacent(halfFace(currFace.getCuboid(), dirs_to_check[1]));
    while (!next.isBorder()) {
        other_dir.push_back(next);
        gotoAdjacent(halfFace(next.getCuboid(), dirs_to_check[1]))
    }
    // Check the first direction in the negative direction
    auto prev = gotoAdjacent(halfFace(currFace.getCuboid(), opposite_face(dirs_to_check[1])));
    while (!prev.isBorder()) {
        other_dir.insert(res.begin(), prev);
        gotoAdjacent(halfFace(prev.getCuboid(), opposite_face(dirs_to_check[1])));
    }

    return (res.size() >= other_dir.size()) ? res : other_dir;
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


