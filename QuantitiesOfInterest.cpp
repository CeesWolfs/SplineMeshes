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
    assert(!currFace.isBorder());
    const auto dirs_to_check = axesToCheck(currFace.getLocalId());
    const auto perfect_match = [&](const halfFace hf, halfFace& twin) {
        if (twin.isBorder()) return false;
        if (twin.isSubdivided()) {
            // Check if we maybe still have a perfect match
            // Check for a vertex which exists in both halfFaces
            const auto local_vertices = Hf2Clv[hf.getLocalId()][currFace.getLocalId()];
            const auto& verts = mesh.getCuboids()[hf.getCuboid()];
            const std::array<uint32_t, 2> ver_ids = { verts.vertices[local_vertices[0]],verts.vertices[local_vertices[1]] };
            const auto twin_hf = *mesh.getSft().find(twin, mesh.getVertices()[ver_ids[0]]);
            // Check if both the required vertices exist in the found half Face
            const auto& twin_verts = mesh.getCuboids()[twin_hf.getCuboid()];
            twin = twin_hf;
            return contains(twin_verts.vertices, ver_ids[0]) && contains(twin_verts.vertices, ver_ids[1]);
        }
        const auto twin_of_twin = mesh.Twin(twin);
        if (twin_of_twin.isSubdivided()) {
            // Check if we maybe still have a perfect match
            // Check for a vertex which exists in both halfFaces
            const auto local_vertices = Hf2Clv[hf.getLocalId()][currFace.getLocalId()];
            const auto& verts = mesh.getCuboids()[hf.getCuboid()];
            const std::array<uint32_t, 2> ver_ids = { verts.vertices[local_vertices[0]],verts.vertices[local_vertices[1]] };
            // Check if both the required vertices exist in the twin
            const auto& twin_verts = mesh.getCuboids()[twin.getCuboid()];
            return contains(twin_verts.vertices, ver_ids[0]) && contains(twin_verts.vertices, ver_ids[1]);
        }
        return true;
    };
    const auto gotoAdjacent = [&](const halfFace hf) {
        auto next_elem = mesh.Twin(hf);
        if(!perfect_match(hf, next_elem)) return halfFace(border_id);
        return halfFace(next_elem.getCuboid(), currFace.getLocalId());
    };
    const auto goDirection = [&](const halfFace hf, const uint8_t direction, std::vector<halfFace>& segments) {
        auto next = gotoAdjacent(halfFace(currFace.getCuboid(), direction));
        while (!next.isBorder()) {
            segments.push_back(next);
            next = gotoAdjacent(halfFace(next.getCuboid(), direction));
        }
    };
    std::vector<halfFace> first_direction;
    // Check the first direction in the negative direction
    goDirection(currFace, mesh.opposite_face(dirs_to_check.first), first_direction);
    std::reverse(first_direction.begin(), first_direction.end());
    first_direction.push_back(currFace);
    // Check the first direction in the positive direction;
    goDirection(currFace, dirs_to_check.first, first_direction);

    std::vector<halfFace> second_direction;
    // Check the second direction in the negative direction
    goDirection(currFace, mesh.opposite_face(dirs_to_check.second), second_direction);
    std::reverse(first_direction.begin(), first_direction.end());
    second_direction.push_back(currFace);
    // Check the second direction in the positive direction;
    goDirection(currFace, dirs_to_check.second, second_direction);

    return (first_direction.size() >= second_direction.size()) ? first_direction : second_direction;
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


