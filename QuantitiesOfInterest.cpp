#include "QuantitiesOfInterest.hpp"
/**
 * To check properties of initial mesh.
 */
QuantitiesOfInterest::QuantitiesOfInterest(): mesh(Mesh()), incidence(mesh.getVertices().size(), mesh.getCuboids().size()) {}

/**
 * To check properties of already existing mesh.
 */
QuantitiesOfInterest::QuantitiesOfInterest(const Mesh& m) : incidence(m.getVertices().size(), m.getCuboids().size()), mesh(m) {}

const Mesh& QuantitiesOfInterest::getMesh() const {
    return mesh;
}

int QuantitiesOfInterest::interiorFaces() const
{
    robin_hood::unordered_set<uint32_t> hfs;
    int F = 0;
    for (size_t cub = 0; cub < mesh.getCuboids().size(); cub++) {
        for (uint8_t hf = 1; hf < 4; hf++) {
            const halfFace half(cub, hf);
            const auto twin = mesh.Twin(half);
            if (twin.isBorder()) continue;
            if (twin.isSubdivided()) {
                for (auto it = mesh.getSft().cbegin(twin); it != mesh.getSft().cend(); ++it) {
                    if (mesh.Twin(*it).isSubdivided()) {
                        hfs.insert((*it).id);
                    }
                    else { F++; }
                }
            }
            else {
                F++;
            }
        }
    }
    for (const auto key : hfs) {
        std::cout << key << '\n';
    }
    return F + hfs.size();
}

/**
 * Returns the amount of elements which are connected to/ linked with the given vertex.
 */
VertexConnectivity QuantitiesOfInterest::vertexConnectivity(uint32_t vertex) const {
    std::array<uint32_t, 8> elements{-1,-1,-1,-1,-1,-1,-1,-1}; // At most 8 elements can be connected to a vertex
    const Vertex& coord = mesh.getVertices()[vertex];
    localVertex lv = mesh.getV2lV()[vertex];
    uint32_t x = 0;
    auto moveToNext = [&](uint32_t& cuboid, uint8_t direction) {
        // first check if the vertex lies inside the face
        const Vertex& face = mesh.getVertices()[mesh.getCuboids()[cuboid].vertices[Hf2Ve[direction][0]]];
        bool inFace = false;
        switch (Hf2Ax[direction])
        {

        case Axis::x: inFace = floatSame(coord.x, face.x); break;
        case Axis::y: inFace = floatSame(coord.y, face.y); break;
        case Axis::z: inFace = floatSame(coord.z, face.z); break;
        }
        if (!inFace) return false;
        auto twin = mesh.Twin(halfFace(cuboid, direction));
        uint32_t new_cuboid;
        if (twin.isBorder()) return false;
        if (twin.isSubdivided()) { new_cuboid = (*mesh.getSft().find(twin, coord)).getCuboid(); }
        else new_cuboid = twin.getCuboid();
        if (contains(elements, new_cuboid)) return false;
        elements[x++] = new_cuboid;
        cuboid = new_cuboid;
        return true;
    };
    elements[x++] = lv.getCuboid();
    auto directions = Lv2Hf[lv.getLocalId()];
    uint32_t currentCuboid = lv.getCuboid();
    // Check all 7 neccasarry cuboids with early stopping
    if (moveToNext(currentCuboid, directions[0])) {
        uint32_t temp = currentCuboid;
        if (moveToNext(currentCuboid, directions[1])) moveToNext(currentCuboid, directions[2]);
        currentCuboid = temp;
        moveToNext(currentCuboid, directions[2]);
    }
    currentCuboid = lv.getCuboid();
    if (moveToNext(currentCuboid, directions[1])) {
        moveToNext(currentCuboid, directions[2]);
    }
    currentCuboid = lv.getCuboid();
    moveToNext(currentCuboid, directions[2]);
    return {elements, x};
}

// Loop over all half faces which contain the vertex
// If one of those faces the border, then return true
bool QuantitiesOfInterest::isBorderVertex(uint32_t vertex) const
{
    localVertex lv = mesh.getV2lV()[vertex];
    for (const uint8_t hf : Lv2Hf[lv.getLocalId()]) {
        if (mesh.Twin(halfFace(lv.getCuboid(), hf)) == border_id) return true;
    }
    return false;
}

bool QuantitiesOfInterest::isEVertex(uint32_t vertex) const
{
    std::array<uint32_t, 8> elements{ -1,-1,-1,-1,-1,-1,-1,-1 }; // At most 8 elements can be connected to a vertex
    std::array<uint32_t, 2> non_unique{ -1,-1 };
    const Vertex& coord = mesh.getVertices()[vertex];
    localVertex lv = mesh.getV2lV()[vertex];
    uint32_t x = 0;
    uint32_t n_u = 0;
    auto moveToNext = [&](uint32_t& cuboid, uint8_t direction) {
        // first check if the vertex lies inside the face
        const Vertex& face = mesh.getVertices()[mesh.getCuboids()[cuboid].vertices[Hf2Ve[direction][0]]];
        bool inFace = false;
        switch (Hf2Ax[direction])
        {

        case Axis::x: inFace = floatSame(coord.x, face.x); break;
        case Axis::y: inFace = floatSame(coord.y, face.y); break;
        case Axis::z: inFace = floatSame(coord.z, face.z); break;
        }
        if (!inFace) {
            if (n_u < 2) non_unique[n_u++] = cuboid;
            return false;
        }
        auto twin = mesh.Twin(halfFace(cuboid, direction));
        uint32_t new_cuboid;
        if (twin.isBorder()) return false;
        if (twin.isSubdivided()) { new_cuboid = (*mesh.getSft().find(twin, coord)).getCuboid(); }
        else new_cuboid = twin.getCuboid();
        if (contains(elements, new_cuboid)) {
            if (n_u < 2) non_unique[n_u++] = new_cuboid;
            return false;
        }
        elements[x++] = new_cuboid;
        cuboid = new_cuboid;
        return true;
    };
    elements[x++] = lv.getCuboid();
    auto directions = Lv2Hf[lv.getLocalId()];
    uint32_t currentCuboid = lv.getCuboid();
    // Check all 7 neccasarry cuboids with early stopping
    if (moveToNext(currentCuboid, directions[0])) {
        uint32_t temp = currentCuboid;
        if (moveToNext(currentCuboid, directions[1])) {
            moveToNext(currentCuboid, directions[2]);
        }
        else {
            return false;
        }
        currentCuboid = temp;
        moveToNext(currentCuboid, directions[2]);
    }
    currentCuboid = lv.getCuboid();
    if (moveToNext(currentCuboid, directions[1])) {
        moveToNext(currentCuboid, directions[2]);
    }
    currentCuboid = lv.getCuboid();
    moveToNext(currentCuboid, directions[2]);
    if (x == 6) {
        // Check if the duplicates touch each other
        return !mesh.Adjacent(non_unique[0], non_unique[1]);
    }
    return false;
}

bool QuantitiesOfInterest::isPVertex(uint32_t vertex) const
{
    const auto [elements, count] = vertexConnectivity(vertex);
    for (int i = 0; i < count; ++i) {
        if (!contains(mesh.getCuboids()[elements[i]].vertices, vertex)) return false;
    }
    return true;
}

// Loop over all half faces which contain the edge
bool QuantitiesOfInterest::isBorderEdge(Edge edge) const
{
    //Both edge vertices should be at the border.
    return isBorderVertex(edge.v1) && isBorderVertex(edge.v2);   
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

bool QuantitiesOfInterest::isBorderCuboid(uint32_t cuboid_id)
{
    //if any halffaces touch a border, the cuboid is at the border
    for (uint8_t hf = 0; hf < 6; ++hf)
    {
        if(mesh.Twin(halfFace(cuboid_id, hf)).isBorder()) return true;
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
        auto next = gotoAdjacent(halfFace(hf.getCuboid(), direction));
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
    std::reverse(second_direction.begin(), second_direction.end());
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
 */
const Eigen::SparseMatrix<bool>& QuantitiesOfInterest::ElementVertexIncidenceMatrix() {
    std::vector<Triplet> tripletList;
    tripletList.reserve(mesh.getV2lV().size() * 3);
    for (int j = 0; j < mesh.getV2lV().size(); j++) {
        for (int i = 0; i < mesh.getCuboids().size(); i++) {
            if (contains(mesh.getCuboids()[i].vertices, j)) {
                tripletList.push_back({ j,i,true });
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
        for (Edge curr : getEdges(i)) {
            if (!contains(res, curr)) {
                res.push_back(curr);
            }
        }
    }
    return res;
}
/**
* Retrieving the 12 edges of a given cuboid.
*/
const std::vector<Edge> QuantitiesOfInterest::getEdges(uint32_t cuboid_id) const {
    std::vector<Edge> res;
    const Cuboid& cuboid = mesh.getCuboids()[cuboid_id];
    res.push_back(Edge{ cuboid.v1, cuboid.v2, cuboid_id });
    res.push_back(Edge{ cuboid.v1, cuboid.v4, cuboid_id });
    res.push_back(Edge{ cuboid.v1, cuboid.v5, cuboid_id });
    res.push_back(Edge{ cuboid.v2, cuboid.v3, cuboid_id });
    res.push_back(Edge{ cuboid.v2, cuboid.v6, cuboid_id });
    res.push_back(Edge{ cuboid.v3, cuboid.v4, cuboid_id });
    res.push_back(Edge{ cuboid.v3, cuboid.v7, cuboid_id });
    res.push_back(Edge{ cuboid.v4, cuboid.v8, cuboid_id });
    res.push_back(Edge{ cuboid.v5, cuboid.v6, cuboid_id });
    res.push_back(Edge{ cuboid.v5, cuboid.v8, cuboid_id });
    res.push_back(Edge{ cuboid.v6, cuboid.v7, cuboid_id });
    res.push_back(Edge{ cuboid.v7, cuboid.v8, cuboid_id });
    return res;
}


/**
 * Destructor
 */
QuantitiesOfInterest::~QuantitiesOfInterest() {
}


