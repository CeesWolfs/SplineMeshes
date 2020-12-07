#include "Mesh.hpp"

#define TINYPLY_IMPLEMENTATION
#include "tinyply.h"

/*
* Default construct the mesh the a unit cube
*/
Mesh::Mesh(/* args */) {
    // Add 8 initial vertices
    vertices.push_back({ 0.0,0.0,0.0 });
    vertices.push_back({ 1.0,0.0,0.0 });
    vertices.push_back({ 1.0,1.0,0.0 });
    vertices.push_back({ 0.0,1.0,0.0 });
    vertices.push_back({ 0.0,0.0,1.0 });
    vertices.push_back({ 1.0,0.0,1.0 });
    vertices.push_back({ 1.0,1.0,1.0 });
    vertices.push_back({ 0.0,1.0,1.0 });
    // Add initial 3D cuboid
    cuboids.push_back({ 0,1,2,3,4,5,6,7 });

    // Add 6 border faces
    for (int i = 0; i < 6; i++) {
        F2f.push_back(halfFace(border_id));
    }
    for (uint8_t i = 0; i < 8; i++)
    {
        V2lV.push_back({ 0,i });
    }
}

const std::vector<Vertex>& Mesh::getVertices() const {
    return vertices;
}
const std::vector<Cuboid>& Mesh::getCuboids() const {
    return cuboids;
}

const std::vector<halfFace>& Mesh::getF2f() const {
    return F2f;
}

const std::vector<localVertex>& Mesh::getV2lV() const {
    return V2lV;
}

const SubFaceTree& Mesh::getSft() const
{
    return sft;
}

void Mesh::Save(const std::string& filename)
{
    std::filebuf fb_binary;
    fb_binary.open(filename + ".ply", std::ios::out);
    std::ostream outstream_binary(&fb_binary);
    if (outstream_binary.fail()) throw std::runtime_error("failed to open " + filename);
    tinyply::PlyFile file;
    file.add_properties_to_element("vertex", { "x", "y", "z" },
        tinyply::Type::FLOAT32, vertices.size(), reinterpret_cast<uint8_t*>(vertices.data()), tinyply::Type::INVALID, 0);
    std::vector<uint32_t> face_vert_ids;
    face_vert_ids.reserve(6 * cuboids.size());
    for(const auto& cub : cuboids)
    {
        for (size_t i = 0; i < 6; i++)
        {
            face_vert_ids.push_back(cub.vertices[Hf2Ve[i][0]]);
            face_vert_ids.push_back(cub.vertices[Hf2Ve[i][1]]);
            face_vert_ids.push_back(cub.vertices[Hf2Ve[i][2]]);
            face_vert_ids.push_back(cub.vertices[Hf2Ve[i][3]]);
        }
    }
    file.add_properties_to_element("face", { "vertex_indices" },
        tinyply::Type::UINT32, cuboids.size() * 6, reinterpret_cast<uint8_t*>(face_vert_ids.data()), tinyply::Type::UINT8, 4);
    // Write a text file
    file.write(outstream_binary, false);
}

void Mesh::splitHalfFace(const halfFace toSplit, const halfFace lower, const halfFace higher, const Axis split_axis, const Vertex& split_point)
{
    auto twin = Twin(toSplit);
    if (twin.isBorder()) return;
    if (twin.isSubdivided()) {
        // Divide the subFaces over the to new halfFaces
        auto split_res = sft.splitTree(twin, split_axis, split_point, lower , higher, F2f);
        sft.updateParent(split_res.first, lower);
        sft.updateParent(split_res.second, higher);
        Twin(lower) = split_res.first;
        Twin(higher) = split_res.second;
    }
    else {
        // Just split the twin
        Twin(twin) = sft.splitHalfFace(Twin(twin), twin, split_axis, split_point, lower, higher);
    }
}

void Mesh::updateHalfFace(const halfFace hf, const halfFace new_hf, const Vertex& middle)
{
    auto twin = Twin(hf);
    if (twin.isBorder()) return;
    if (twin.isSubdivided())
    {
        // Update the parent of the nodes
        sft.updateParent(twin, new_hf);
        // Update all the subFaces
        for (auto it = sft.cbegin(twin); it != sft.cend(); ++it) {
            updateTwin(twin, hf, new_hf, middle);
        }
    }
    else {
        updateTwin(twin, hf, new_hf, middle);
    }
}

void Mesh::updateTwin(const halfFace twin, const halfFace old_hf, const halfFace new_hf, const Vertex& middle)
{
    if (twin.isSubdivided()) {
        auto it = sft.find(twin, old_hf, middle);
        *it = new_hf;
    }
    else {
        Twin(twin) = new_hf;
    }

}


halfFace& Mesh::Twin(const halfFace& hf) {
    return F2f[static_cast<size_t>(hf.getCuboid()) * 6 + hf.getLocalId()];
}

const halfFace& Mesh::Twin(const halfFace& hf) const
{
    return F2f[static_cast<size_t>(hf.getCuboid())*6  + hf.getLocalId()];
}

/* Static helper functions */
constexpr uint8_t opposite_face(uint8_t local_id) {
    switch (local_id & 0x7)
    {
    case 0: return 1;
    case 1: return 0;
    case 2: return 4;
    case 4: return 2;
    case 3: return 5;
    case 5: return 3;
    default:
        return 255;
    }
}

constexpr int localIndexinFace(uint8_t face_ind, uint8_t local_vertex_idx) {
    auto res = std::find(std::cbegin(Hf2Ve[face_ind]), std::cend(Hf2Ve[face_ind]), local_vertex_idx);
    assert(res != std::cend(Hf2Ve[face_ind]));
    return std::distance(std::cbegin(Hf2Ve[face_ind]), res);
}

bool Mesh::mergeVertexIfExistsRewrite(const Vertex& v, uint32_t& vref, HalfFacePair toCheck, uint8_t local_id, Axis split_axis)
{
    bool vertexFound = false;
    auto hftoCheck = toCheck.first;
    auto hftoCheck2 = toCheck.second;
    const auto twinCheck = Twin(hftoCheck);
    // HalfFace in which the vertex to be found lays
    halfFace face{ (uint32_t)-1 };
    halfFace hftoCheck3{ (uint32_t)-1 };
    // First check the first necassary halfFace
    if(twinCheck.isBorder()) {}
    else if (twinCheck.isSubdivided())
    {
        vertexFound = sft.findVertexBorder(twinCheck, v, split_axis, face);
        if (vertexFound) {
            // Find the local id of the vertex in the touching element
            const auto local_vertex_in_opposite = Hf2Ve[opposite_face(hftoCheck.getLocalId())][localIndexinFace(hftoCheck.getLocalId(), local_id)];
            vref = cuboids[face.getCuboid()].vertices[local_vertex_in_opposite];
            return true;
        }
        if(!face.isBorder())
            hftoCheck3 = halfFace(face.getCuboid(), hftoCheck2.getLocalId());
    }
    else {
        hftoCheck3 = halfFace(twinCheck.getCuboid(), hftoCheck2.getLocalId());
    }
    if (!hftoCheck3.isBorder()) {
        const auto twinCheck3 = Twin(hftoCheck3);
        // Not found yet, check the diagonal element from the main element
        if (twinCheck3.isSubdivided() && !twinCheck3.isBorder()) {
            vertexFound = sft.findVertexBorder(twinCheck3, v, split_axis, face);
            if (vertexFound) {
                // Find the local id of the vertex in the touching element
                const auto local_vertex_in = Hf2Ve[opposite_face(hftoCheck3.getLocalId())][localIndexinFace(hftoCheck3.getLocalId(), local_id)];
                vref = cuboids[face.getCuboid()].vertices[local_vertex_in];
                return true;
            }
        }
    }
    const auto twinCheck2 = Twin(hftoCheck2);
    if (twinCheck2.isSubdivided() && !twinCheck2.isBorder()) {
        vertexFound = sft.findVertexBorder(twinCheck2, v, split_axis, face);
        if (vertexFound) {
            // Find the local id of the vertex in the touching element
            const auto local_vertex_in_opposite = Hf2Ve[opposite_face(hftoCheck2.getLocalId())][localIndexinFace(hftoCheck2.getLocalId(), local_id)];
            vref = cuboids[face.getCuboid()].vertices[local_vertex_in_opposite];
            return true;
        }
    }
    return false;
}

bool Mesh::Adjacent(uint32_t elem1, uint32_t elem2) const
{
    for (size_t i = 0; i < 6; i++)
    {
        const auto twin = Twin(halfFace(elem1, i));
        if (twin.isBorder()) continue;
        if (twin.isSubdivided()) {
            for (auto it = sft.cbegin(twin); it != sft.cend(); ++it)
            {
                if ((*it).getCuboid() == elem2) return true;
            }
        }
        else {
            if (twin.getCuboid() == elem2) return true;
        }
    }
    return false;
}

void Mesh::addHalfFaces(const uint32_t cuboid_id, const Axis split_axis) {
    // For now just push back the twins of the original element
    (split_axis == Axis::z) ? F2f.push_back(halfFace(cuboid_id, 1)) : F2f.push_back(Twin(halfFace(cuboid_id, 0)));
    F2f.push_back(Twin(halfFace(cuboid_id, 1)));
    F2f.push_back(Twin(halfFace(cuboid_id, 2)));
    F2f.push_back(Twin(halfFace(cuboid_id, 3)));
    (split_axis == Axis::y) ? F2f.push_back((halfFace(cuboid_id, 2))) : F2f.push_back(Twin(halfFace(cuboid_id, 4)));
    (split_axis == Axis::x) ? F2f.push_back(halfFace(cuboid_id, 3)) : F2f.push_back(Twin(halfFace(cuboid_id, 5)));
}



uint32_t Mesh::SplitAlongXY(uint32_t cuboid_id, float z_split) {
    // border checks
    if ((z_split <= vertices[cuboids[cuboid_id].v1].z) || z_split >= vertices[cuboids[cuboid_id].v5].z) {
        return -1; // Splitpoint not in cuboid
    }
    const uint32_t new_cuboid_id = cuboids.size();

    // All the old vertices
    const Vertex v1_old = vertices[cuboids[cuboid_id].v1];
    const Vertex v2_old = vertices[cuboids[cuboid_id].v2];
    const Vertex v3_old = vertices[cuboids[cuboid_id].v3];
    const Vertex v4_old = vertices[cuboids[cuboid_id].v4];

    // All the new vertices
    const Vertex v1_new = { v1_old.x, v1_old.y, z_split };
    const Vertex v2_new = { v2_old.x, v2_old.y, z_split };
    const Vertex v3_new = { v3_old.x, v3_old.y, z_split };
    const Vertex v4_new = { v4_old.x, v4_old.y, z_split };

    const auto middle = (v1_new + v3_new) / 2;

    // Check whether the new vertices already exist in the vectors array.
    uint32_t v1_idx = 0;
    uint32_t v2_idx = 0;
    uint32_t v3_idx = 0;
    uint32_t v4_idx = 0;

    if (!mergeVertexIfExistsRewrite(v1_new, v1_idx, {halfFace(cuboid_id, 5), halfFace(cuboid_id, 4)}, 4, Axis::z)) {
        // New vertex push it back, and push back V2lV
        v1_idx = vertices.size();
        vertices.push_back(v1_new);
        V2lV.push_back({new_cuboid_id, 0});
    }
    if (!mergeVertexIfExistsRewrite(v2_new, v2_idx, { halfFace(cuboid_id, 4), halfFace(cuboid_id, 3) }, 5, Axis::z)) {
        v2_idx = vertices.size();
        vertices.push_back(v2_new);
        V2lV.push_back({ new_cuboid_id, 1 });
    }
    if (!mergeVertexIfExistsRewrite(v3_new, v3_idx, { halfFace(cuboid_id, 3), halfFace(cuboid_id, 2) }, 6, Axis::z)) {
        v3_idx = vertices.size();
        vertices.push_back(v3_new);
        V2lV.push_back({ new_cuboid_id, 2 });
    }
    if (!mergeVertexIfExistsRewrite(v4_new, v4_idx, { halfFace(cuboid_id, 2), halfFace(cuboid_id, 5) }, 7, Axis::z)) {
        v4_idx = vertices.size();
        V2lV.push_back({ new_cuboid_id, 3 });
        vertices.push_back(v4_new);
    }

    // Update old cuboid vertices
    cuboids.push_back(cuboids[cuboid_id]);
    cuboids[cuboid_id].v5 = v1_idx;
    cuboids[cuboid_id].v6 = v2_idx;
    cuboids[cuboid_id].v7 = v3_idx;
    cuboids[cuboid_id].v8 = v4_idx;

    // Update new cuboid vertices
    Cuboid& new_cuboid = cuboids[new_cuboid_id];
    new_cuboid.v1 = v1_idx;
    new_cuboid.v2 = v2_idx;
    new_cuboid.v3 = v3_idx;
    new_cuboid.v4 = v4_idx;

    addHalfFaces(cuboid_id, Axis::z);

    // Update the twin faces (mark them as subdivided).
    splitHalfFace(halfFace(cuboid_id, 2), halfFace(cuboid_id, 2), halfFace(new_cuboid_id, 2), Axis::z, middle);
    splitHalfFace(halfFace(cuboid_id, 3), halfFace(cuboid_id, 3), halfFace(new_cuboid_id, 3), Axis::z, middle);
    splitHalfFace(halfFace(cuboid_id, 4), halfFace(cuboid_id, 4), halfFace(new_cuboid_id, 4), Axis::z, middle);
    splitHalfFace(halfFace(cuboid_id, 5), halfFace(cuboid_id, 5), halfFace(new_cuboid_id, 5), Axis::z, middle);
    // Update the top halfFace
    updateHalfFace(halfFace(cuboid_id, 1), halfFace(new_cuboid_id, 1), middle);
    // Point the top of the old cuboid to the new cuboid
    Twin(halfFace(cuboid_id, 1)) = halfFace(new_cuboid_id, 0);
    
    return new_cuboid_id;
}


uint32_t Mesh::SplitAlongYZ(uint32_t cuboid_id, float x_split) {
    // border checks
    if ((x_split <= vertices[cuboids[cuboid_id].v1].x) || x_split >= vertices[cuboids[cuboid_id].v2].x) {
        return -1; // Splitpoint not in cuboid
    }

    // All the old vertices
    const Vertex v1_old = vertices[cuboids[cuboid_id].v1];
    const Vertex v2_old = vertices[cuboids[cuboid_id].v4];
    const Vertex v3_old = vertices[cuboids[cuboid_id].v8];
    const Vertex v4_old = vertices[cuboids[cuboid_id].v5];

    // All the new vertices
    const Vertex v1_new = { x_split, v1_old.y, v1_old.z };
    const Vertex v2_new = { x_split, v2_old.y, v2_old.z };
    const Vertex v3_new = { x_split, v3_old.y, v3_old.z };
    const Vertex v4_new = { x_split, v4_old.y, v4_old.z };

    const auto middle = (v1_new + v3_new) / 2;

    const uint32_t new_cuboid_id = cuboids.size();

    uint32_t v1_idx = 0;
    uint32_t v2_idx = 0;
    uint32_t v3_idx = 0;
    uint32_t v4_idx = 0;

    if (!mergeVertexIfExistsRewrite(v1_new, v1_idx, { halfFace(cuboid_id, 4), halfFace(cuboid_id, 0) }, 1, Axis::x)) {
        v1_idx = vertices.size();
        V2lV.push_back({ new_cuboid_id, 0 });
        vertices.push_back(v1_new);
    }
    if (!mergeVertexIfExistsRewrite(v2_new, v2_idx, { halfFace(cuboid_id, 2), halfFace(cuboid_id, 0) }, 2, Axis::x)) {
        v2_idx = vertices.size();
        vertices.push_back(v2_new);
        V2lV.push_back({ new_cuboid_id, 3 });
    }
    if (!mergeVertexIfExistsRewrite(v3_new, v3_idx, { halfFace(cuboid_id, 2), halfFace(cuboid_id, 1) }, 6, Axis::x)) {
        v3_idx = vertices.size();
        V2lV.push_back({ new_cuboid_id, 7 });
        vertices.push_back(v3_new);
    }
    if (!mergeVertexIfExistsRewrite(v4_new, v4_idx, { halfFace(cuboid_id, 4), halfFace(cuboid_id, 1) }, 5, Axis::x)) {
        v4_idx = vertices.size();
        vertices.push_back(v4_new);
        V2lV.push_back({ new_cuboid_id, 4 });
    }

     // Update old cuboid vertices
    cuboids.push_back(cuboids[cuboid_id]);

    cuboids[cuboid_id].v2 = v1_idx;
    cuboids[cuboid_id].v3 = v2_idx;
    cuboids[cuboid_id].v7 = v3_idx;
    cuboids[cuboid_id].v6 = v4_idx;

    // Update new cuboid vertices
    Cuboid &new_cuboid = cuboids[new_cuboid_id];
    new_cuboid.v1 = v1_idx;
    new_cuboid.v4 = v2_idx;
    new_cuboid.v8 = v3_idx;
    new_cuboid.v5 = v4_idx;

    addHalfFaces(cuboid_id, Axis::x);

    // Update the twin faces (mark them as subdivided).
    splitHalfFace(halfFace(cuboid_id, 0), halfFace(cuboid_id, 0), halfFace(new_cuboid_id, 0), Axis::x, middle);
    splitHalfFace(halfFace(cuboid_id, 1), halfFace(cuboid_id, 1), halfFace(new_cuboid_id, 1), Axis::x, middle);
    splitHalfFace(halfFace(cuboid_id, 2), halfFace(cuboid_id, 2), halfFace(new_cuboid_id, 2), Axis::x, middle);
    splitHalfFace(halfFace(cuboid_id, 4), halfFace(cuboid_id, 4), halfFace(new_cuboid_id, 4), Axis::x, middle);
    // Update the top halfFace
    updateHalfFace(halfFace(cuboid_id, 3), halfFace(new_cuboid_id, 3), middle);
    // Point the top of the old cuboid to the new cuboid
    Twin(halfFace(cuboid_id, 3)) = halfFace(new_cuboid_id, 5);
    
    return new_cuboid_id;
}

uint32_t Mesh::SplitAlongXZ(uint32_t cuboid_id, float y_split) {
    // border checks
    if ((y_split <= vertices[cuboids[cuboid_id].v2].y) || y_split >= vertices[cuboids[cuboid_id].v3].y) {
        return -1; // Splitpoint not in cuboid
    }

    // All the old vertices
    const Vertex v1_old = vertices[cuboids[cuboid_id].v1];
    const Vertex v2_old = vertices[cuboids[cuboid_id].v2];
    const Vertex v3_old = vertices[cuboids[cuboid_id].v6];
    const Vertex v4_old = vertices[cuboids[cuboid_id].v5];

    // All the new vertices
    const Vertex v1_new = { v1_old.x, y_split, v1_old.z };
    const Vertex v2_new = { v2_old.x, y_split, v2_old.z };
    const Vertex v3_new = { v3_old.x, y_split, v3_old.z };
    const Vertex v4_new = { v4_old.x, y_split, v4_old.z };

    const auto middle = (v1_new + v3_new) / 2;

    const uint32_t new_cuboid_id = cuboids.size();

    uint32_t v1_idx = 0;
    uint32_t v2_idx = 0;
    uint32_t v3_idx = 0;
    uint32_t v4_idx = 0;

    if (!mergeVertexIfExistsRewrite(v1_new, v1_idx, { halfFace(cuboid_id, 5), halfFace(cuboid_id, 0) }, 3, Axis::y)) {
        v1_idx = vertices.size();
        V2lV.push_back({ new_cuboid_id, 0 });
        vertices.push_back(v1_new);
    }
    if (!mergeVertexIfExistsRewrite(v2_new, v2_idx, { halfFace(cuboid_id, 3), halfFace(cuboid_id, 0) }, 2, Axis::y)) {
        v2_idx = vertices.size();
        vertices.push_back(v2_new);
        V2lV.push_back({ new_cuboid_id, 1 });
    }
    if (!mergeVertexIfExistsRewrite(v3_new, v3_idx, { halfFace(cuboid_id, 3), halfFace(cuboid_id, 1) }, 6, Axis::y)) {
        v3_idx = vertices.size();
        V2lV.push_back({ new_cuboid_id, 5 });
        vertices.push_back(v3_new);
    }
    if (!mergeVertexIfExistsRewrite(v4_new, v4_idx, { halfFace(cuboid_id, 5), halfFace(cuboid_id, 1) }, 7, Axis::y)) {
        v4_idx = vertices.size();
        vertices.push_back(v4_new);
        V2lV.push_back({ new_cuboid_id, 4 });
    }

    // Update old cuboid vertices
    cuboids.push_back(cuboids[cuboid_id]);
    cuboids[cuboid_id].v4 = v1_idx;
    cuboids[cuboid_id].v3 = v2_idx;
    cuboids[cuboid_id].v7 = v3_idx;
    cuboids[cuboid_id].v8 = v4_idx;

    // Update new cuboid vertices
    Cuboid& new_cuboid = cuboids[new_cuboid_id];
    new_cuboid.v1 = v1_idx;
    new_cuboid.v2 = v2_idx;
    new_cuboid.v6 = v3_idx;
    new_cuboid.v5 = v4_idx;

    addHalfFaces(cuboid_id, Axis::y);

    // Update the twin faces (mark them as subdivided).
    splitHalfFace(halfFace(cuboid_id, 0), halfFace(cuboid_id, 0), halfFace(new_cuboid_id, 0), Axis::y, middle);
    splitHalfFace(halfFace(cuboid_id, 1), halfFace(cuboid_id, 1), halfFace(new_cuboid_id, 1), Axis::y, middle);
    splitHalfFace(halfFace(cuboid_id, 3), halfFace(cuboid_id, 3), halfFace(new_cuboid_id, 3), Axis::y, middle);
    splitHalfFace(halfFace(cuboid_id, 5), halfFace(cuboid_id, 5), halfFace(new_cuboid_id, 5), Axis::y, middle);
    // Update the top halfFace
    updateHalfFace(halfFace(cuboid_id, 2), halfFace(new_cuboid_id, 2), middle);
    // Point the top of the old cuboid to the new cuboid
    Twin(halfFace(cuboid_id, 2)) = halfFace(new_cuboid_id, 4);

    return new_cuboid_id;
}