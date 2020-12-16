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
 * Check whether the given cuboid touches a corner of the mesh. There can be a maximum of 8 corner cuboids, since there are 8 corner vertices.
 */
bool QuantitiesOfInterest::isCornerCuboid(const Cuboid &cuboid) {
    for (auto v : cuboid.vertices) {
        if (v >= 0 && v <= 7) return true;
    }
    return false;
}

static constexpr std::pair<uint8_t, uint8_t> axesToCheck(uint8_t local_face_id) {
    switch (local_face_id) {
    case 0: case 1: return { 3,2 };
    case 2: case 4: return { 1,3 };
    case 3: case 5: return { 1,2 };
    }
}

/**
 * Return the maximal segment which consists of /starts from the given face id.
 */
const std::vector<halfFace> QuantitiesOfInterest::getMaximalSegmentOf(halfFace currFace) {
    //TODO: doesn't add all the correct face ids. 
    std::vector<halfFace> res;
    res.push_back(currFace);
    //if (currFace.isBorder()) return res;
    const auto dirs_to_check = axesToCheck(currFace.getLocalId());

    const auto perfect_match = [](halfFace &hf, halfFace &twin, Mesh &mesh) {
        // TODO Check if only split in axis that doesnt matter
        if(hf.isSubdivided()) return false;
        auto idx = static_cast<size_t>(twin.getCuboid()) * 6 + twin.getLocalId();
        auto twin_of_twin = (idx < 0 || idx >= mesh.getF2f().size()) ? halfFace(border_id) : mesh.Twin(twin);
        return (twin_of_twin == hf);
    };

    const auto gotoAdjacent = [&](halfFace next) {
        auto idx = static_cast<size_t>(next.getCuboid()) * 6 + next.getLocalId();
        auto next_elem = (idx < 0 || idx >= mesh.getF2f().size()) ? halfFace(border_id) : mesh.Twin(next);
        if(!perfect_match(next, next_elem, mesh) || next_elem.isBorder()) return halfFace(border_id);
        return halfFace(next_elem.getCuboid(), currFace.getLocalId());
    };

    // Check the first direction in the positive direction
    auto next = gotoAdjacent(halfFace(currFace.getCuboid(), dirs_to_check.first));
    while (!next.isBorder()) {
        res.push_back(next);
        next = gotoAdjacent(halfFace(next.getCuboid(), dirs_to_check.first));
        if (next.isBorder()) {
            res.push_back(next);
        }
    }
    // Check the first direction in the negative direction
    auto prev = gotoAdjacent(halfFace(currFace.getCuboid(), mesh.opposite_face(dirs_to_check.first)));
    while (!prev.isBorder()) {
        res.insert(res.begin(), prev);
        prev = gotoAdjacent(halfFace(prev.getCuboid(), mesh.opposite_face(dirs_to_check.first)));
        if (prev.isBorder()) {
            res.push_back(prev);
        }
    }

    std::vector<halfFace> other_dir;
    // Check the first direction in the positive direction
    next = gotoAdjacent(halfFace(currFace.getCuboid(), dirs_to_check.second));
    while (!next.isBorder()) {
        other_dir.push_back(next);
        next = gotoAdjacent(halfFace(next.getCuboid(), dirs_to_check.second));
        if (next.isBorder()) {
            res.push_back(next);
        }
    }
    // Check the first direction in the negative direction
    prev = gotoAdjacent(halfFace(currFace.getCuboid(), mesh.opposite_face(dirs_to_check.second)));
    while (!prev.isBorder()) {
        other_dir.insert(res.begin(), prev);
        prev = gotoAdjacent(halfFace(prev.getCuboid(), mesh.opposite_face(dirs_to_check.second)));
        if (prev.isBorder()) {
            res.push_back(prev);
        }
    }

    return (res.size() >= other_dir.size()) ? res : other_dir;
}


/**
 * Unsigned indicence matrix which shows connectivity between the elements and their vertices.
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
* Signed incidence matrix to show connectivity how vertices are paired with each other. 
*/
const MatrixXf QuantitiesOfInterest::VertexEdgeIncidenceMatrix()
{
    std::vector<Edge> edges = getAllEdges();
    std::cout << edges.size() << std::endl;
    MatrixXf matrix(edges.size(), mesh.getVertices().size());

    //initial matrix
    for (int i = 0; i < edges.size(); i++) {
        for (int j = 0; j < mesh.getVertices().size(); j++) {
            matrix(i, j) = 0;
        }
    }

    Vertex origin{ 0,0,0 };
    for (int i = 0; i < edges.size(); i++) {
        if (mesh.getVertices()[edges[i].v1] - mesh.getVertices()[edges[i].v2] >= origin) {
            matrix(i, edges[i].v2) = -1;
            matrix(i, edges[i].v1) = 1;
        }
        else if (mesh.getVertices()[edges[i].v2] - mesh.getVertices()[edges[i].v1] >= origin) {
            matrix(i, edges[i].v2) = 1;
            matrix(i, edges[i].v1) = -1;
        }
    }
    return matrix;
}

/**
* Retrieving every edge of the mesh.
*/
const std::vector<Edge> QuantitiesOfInterest::getAllEdges() const {
    std::vector<Edge> res;
    for (auto i = 0; i < mesh.getCuboids().size(); i++) {
        for (Edge curr : getEdges(mesh.getCuboids()[i])) {
            bool contains = false;
            for (Edge known: res) {
                if (known == curr) {
                    contains = true;
                    break;
                }
            }
            if (!contains) {
                res.push_back(curr);
            }
        }
    }
    return res;
}
/**
* Retrieving the 12 edges of a given cuboid.
*/
const std::vector<Edge> QuantitiesOfInterest::getEdges(const Cuboid &cuboid) const {
    std::vector<Edge> res;
    res.push_back(Edge(cuboid.v1, cuboid.v2));
    res.push_back(Edge(cuboid.v1, cuboid.v4));
    res.push_back(Edge(cuboid.v1, cuboid.v5));
    res.push_back(Edge(cuboid.v2, cuboid.v3));
    res.push_back(Edge(cuboid.v2, cuboid.v6));
    res.push_back(Edge(cuboid.v3, cuboid.v4));
    res.push_back(Edge(cuboid.v3, cuboid.v7));
    res.push_back(Edge(cuboid.v4, cuboid.v8));
    res.push_back(Edge(cuboid.v5, cuboid.v6));
    res.push_back(Edge(cuboid.v5, cuboid.v8));
    res.push_back(Edge(cuboid.v6, cuboid.v7));
    res.push_back(Edge(cuboid.v7, cuboid.v8));
    return res;
}


/**
 * Destructor
 */
QuantitiesOfInterest::~QuantitiesOfInterest() {
}


