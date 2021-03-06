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

Mesh::Mesh(int Nx, int Ny, int Nz)
{
    vertices.reserve((Nx + 1) * (Ny + 1) * (Nz + 1));
    cuboids.reserve(Nx * Ny * Nz);
    V2lV.reserve((Nx + 1) * (Ny + 1) * (Nz + 1));
    F2f.reserve(Nx * Ny * Nz * 6);
    const float Lx = 1 / static_cast<float>(Nx);
    const float Ly = 1 / static_cast<float>(Ny);
    const float Lz = 1 / static_cast<float>(Nz);
    // Construct all the vertices
    for (size_t k = 0; k <= Nz; k++)
    {
        for (size_t j = 0; j <= Ny; j++)
        {
            for (size_t i = 0; i <= Nx; i++)
            {
                vertices.emplace_back(Vertex{i*Lx, j*Ly, k*Lz});
            }
        }
    }
    auto toVertIndex = [=](uint32_t i, uint32_t j, uint32_t k) {return i + j * (Nx + 1) + k * (Nx + 1) * (Ny + 1); };
    // Construct all the cuboids
    for (size_t k = 0; k < Nz; k++)
    {
        for (size_t j = 0; j < Ny; j++)
        {
            for (size_t i = 0; i < Nx; i++)
            {
                cuboids.emplace_back(Cuboid{toVertIndex(i,j,k),toVertIndex(i+1,j,k),toVertIndex(i+1,j+1,k),toVertIndex(i,j+1,k),toVertIndex(i,j,k+1),toVertIndex(i + 1,j,k+1),toVertIndex(i + 1,j + 1,k+1),toVertIndex(i,j + 1,k+1) });
            }
        }
    }
    auto toCubIndex = [=](uint32_t i, uint32_t j, uint32_t k) {return i + j * Nx + k * Nx * Ny; };
    // Construct all the halfFaces
    for (size_t k = 0; k < Nz; k++)
    {
        for (size_t j = 0; j < Ny; j++)
        {
            for (size_t i = 0; i < Nx; i++)
            {
                F2f.emplace_back((k == 0) ? border_id : halfFace(toCubIndex(i, j, k - 1), 1));
                F2f.emplace_back((k == (Nz - 1)) ? border_id : halfFace(toCubIndex(i, j, k + 1), 0));
                F2f.emplace_back((j == (Ny - 1)) ? border_id : halfFace(toCubIndex(i, j + 1, k), 4));
                F2f.emplace_back((i == (Nx - 1) ) ? border_id : halfFace(toCubIndex(i+1, j, k), 5));
                F2f.emplace_back((j == 0) ? border_id : halfFace(toCubIndex(i, j - 1, k), 2));
                F2f.emplace_back((i == 0) ? border_id : halfFace(toCubIndex(i - 1, j, k), 3));
            }
        }
    }
    static constexpr uint8_t lookup[8] = {0, 1, 3, 2, 4, 5, 7, 6};
    for (size_t k = 0; k <= Nz; k++)
    {
        for (size_t j = 0; j <= Ny; j++)
        {
            for (size_t i = 0; i <= Nx; i++)
            {
                int cube_x = 2*(i/2);
                int cube_y = 2*(j/2);
                int cube_z = 2*(k/2);
                int local_x = i % 2;
                int local_y = j % 2;
                int local_z = k % 2;
                if (cube_x == Nx) {
                    cube_x--;
                    local_x = 1;
                }
                if (cube_y == Ny) {
                    cube_y--;
                    local_y = 1;
                }
                if (cube_z == Nz) {
                    cube_z--;
                    local_z = 1;
                }
                uint8_t local_index = lookup[local_z * 4 + local_y * 2 + local_x];
                V2lV.emplace_back(localVertex(toCubIndex(cube_x, cube_y, cube_z), local_index));
            }
        }
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
            updateTwin(*it, hf, new_hf, middle);
        }
    }
    else {
        updateTwin(twin, hf, new_hf, middle);
    }
}

void Mesh::updateTwin(const halfFace twin, const halfFace old_hf, const halfFace new_hf, const Vertex& middle)
{
    if (Twin(twin).isSubdivided()) {
        auto it = sft.find(Twin(twin), middle);
        assert(*it == old_hf);
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

constexpr int localIndexinFace(uint8_t face_ind, uint8_t local_vertex_idx) {
    auto res = std::find(std::cbegin(Hf2Ve[face_ind]), std::cend(Hf2Ve[face_ind]), local_vertex_idx);
    assert(res != std::cend(Hf2Ve[face_ind]));
    return std::distance(std::cbegin(Hf2Ve[face_ind]), res);
}

std::pair<bool, uint32_t> Mesh::mergeVertexIfExists(const Vertex& v, HalfFacePair toCheck, uint8_t local_id, Axis split_axis)
{
    // HalfFace in which the vertex to be found lays
    halfFace face{border_id};
    uint32_t vref{border_id};
    auto const checkHf = [&, split_axis](halfFace hf) {
        if (hf.isBorder()) return false;
        const auto twin = Twin(hf);
        if (twin.isSubdivided()) {
            if (sft.findVertexBorder(twin, v, split_axis, face)) {
                // Find the local id of the vertex in the touching element
                const auto local_vertex_in_opposite = Hf2Ve[opposite_face(hf.getLocalId())][localIndexinFace(hf.getLocalId(), local_id)];
                vref = cuboids[face.getCuboid()].vertices[local_vertex_in_opposite];
                // Check if the vertex actually corresponds, this is sometimes not the case if the other side is also subdivided
                if (vertices[vref] == v) {
                    return true;
                }
            }
        }
        return false;
    };
    // Check the first halfFace
    if (checkHf(toCheck.first)) {
        return { true, vref };
    }
    // Check the right diagonal face
    halfFace front_elem = face.isBorder() ? toCheck.first : face;
    halfFace diagonalHf = front_elem.isBorder() ? border_id: halfFace(front_elem.getCuboid(), toCheck.second.getLocalId());
    if (checkHf(diagonalHf)) {
        return { true, vref };
    }
    // Finally check the second face
    if (checkHf(toCheck.second)) {
        return { true, vref };
    }
    return { false, vref };
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


uint32_t Mesh::SplitAlongAxis(uint32_t cuboid_id, float split_point, Axis axis) {
    uint8_t face_to_split = -1;
    switch (axis) {
        case Axis::x : 
            face_to_split = 3;
            break;
        case Axis::y :
            face_to_split = 2;
            break;
        case Axis::z :
            face_to_split = 1;
            break;
        default:
            std::cout << "Invalid axis given. Returning -1 ..." << std::endl;
            return -1;
    }

    // border checks, return -1 if splitpoint is not in cuboid
    if (axis == Axis::x) {
        if ((split_point <= vertices[cuboids[cuboid_id].v1].x) || split_point >= vertices[cuboids[cuboid_id].v2].x) {
            return -1;
        }
    }
    else if (axis == Axis::y) {
        if ((split_point <= vertices[cuboids[cuboid_id].v2].y) || split_point >= vertices[cuboids[cuboid_id].v3].y) {
            return -1;
        }
    }
    else {
        if ((split_point <= vertices[cuboids[cuboid_id].v1].z) || split_point >= vertices[cuboids[cuboid_id].v5].z) {
            return -1;
        }
    }

    // All the old vertices
    std::array<Vertex, 4> v_old;
    std::generate(v_old.begin(), v_old.end(), [&, idx = 0]() mutable {
        switch (axis) {
            case Axis::z:
                return vertices[cuboids[cuboid_id].vertices[idx++]];
            default:
                return vertices[cuboids[cuboid_id].vertices[Hf2Ve[face_to_split][idx++]]];;
        }
    });

    // All the new vertices
    std::array<Vertex, 4> v_new;
    std::generate(v_new.begin(), v_new.end(), [&, idx = 0]() mutable {
        switch (axis) {
            case Axis::x :
                return Vertex{ split_point, v_old[idx].y, v_old[idx++].z };
            case Axis::y :
                return Vertex{ v_old[idx].x, split_point, v_old[idx++].z };
            default:
                return Vertex{ v_old[idx].x, v_old[idx++].y, split_point };
        }
    });

    const auto middle = (v_new[0] + v_new[2]) / 2;
    const uint32_t new_cuboid_id = cuboids.size();
    std::array<uint32_t, 4> vertex_inds;
    
    for (size_t i = 0; i < vertex_inds.size(); i++)
    {
        HalfFacePair hfp({ uint32_t(-1) }, { uint8_t(-1) });
        switch (axis) {
            case Axis::x:
                hfp = { {cuboid_id, Hfs2Check[0][i][0]}, {cuboid_id, Hfs2Check[0][i][1]} };
                break;
            case Axis::y:
                hfp = { {cuboid_id,  Hfs2Check[1][i][0]}, {cuboid_id, Hfs2Check[1][i][0]} };
                break;
            default:
                hfp = { {cuboid_id, Hfs2Check[2][i][0]}, {cuboid_id, Hfs2Check[2][i][1]} };
                break;
        }
        
        const auto [found, vertex] = mergeVertexIfExists(v_new[i], hfp, Hf2Ve[face_to_split][i], axis);
        if (found) {
            vertex_inds[i] = vertex;
        }
        else {
            vertex_inds[i] = vertices.size();
            V2lV.push_back({ new_cuboid_id, Hf2Ve[opposite_face(face_to_split)][i] });
            vertices.push_back(v_new[i]);
        }
    }

    // Update V2lV for the points that now belong to the new element
    for (const auto lv : Hf2Ve[face_to_split]) {
        V2lV[cuboids[cuboid_id].vertices[lv]] = localVertex(new_cuboid_id, lv);
    }

    // Update all the vertices for the new and old element
    cuboids.push_back(cuboids[cuboid_id]);
    for (size_t i = 0; i < vertex_inds.size(); i++)
    {
        cuboids[cuboid_id].vertices[Hf2Ve[face_to_split][i]] = vertex_inds[i];
        cuboids[new_cuboid_id].vertices[Hf2Ve[opposite_face(face_to_split)][i]] = vertex_inds[i];
    }

    addHalfFaces(cuboid_id, axis);

    // Update the twin faces (mark them as subdivided).
    for (size_t hf = 0; hf < 6; hf++)
    {
        if (hf == face_to_split || hf == opposite_face(face_to_split)) continue;
        splitHalfFace(halfFace(cuboid_id, hf), halfFace(cuboid_id, hf), halfFace(new_cuboid_id, hf), axis, middle);
    }

    // Update the top halfFace
    updateHalfFace(halfFace(cuboid_id, face_to_split), halfFace(new_cuboid_id, face_to_split), middle);
    
    // Point the top of the old cuboid to the new cuboid
    Twin(halfFace(cuboid_id, face_to_split)) = halfFace(new_cuboid_id, opposite_face(face_to_split));

    return new_cuboid_id;
}

uint32_t Mesh::SplitAlongXY(uint32_t cuboid_id, float z_split) {
    return SplitAlongAxis(cuboid_id, z_split, Axis::z);
}


uint32_t Mesh::SplitAlongYZ(uint32_t cuboid_id, float x_split) {
    return SplitAlongAxis(cuboid_id, x_split, Axis::x);
}

uint32_t Mesh::SplitAlongXZ(uint32_t cuboid_id, float y_split) {
    return SplitAlongAxis(cuboid_id, y_split, Axis::y);
}