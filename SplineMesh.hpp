#pragma once
#include "Splines.hpp"
/*
* Stores all the continuity matrices for all faces
*/
template<int N, int C>
class SplineMesh
{
private:
    int numElements = 0;
    bool constraintsValid = false;
    Mesh mesh;
    void insertFaceConstraints(HalfFacePair pair, bool virtualSplitLeft);
    void invalidateConstraints(uint32_t cuboid_id);
    static const uint64_t toKey(HalfFacePair pair) { return static_cast<uint64_t>(pair.first.id) + (static_cast<uint64_t>(pair.second.id) << 32); }
    static const HalfFacePair fromKey(uint64_t key) { return { halfFace(static_cast<uint32_t>(key)), halfFace(static_cast<uint32_t>(key >> 32)) }; }
public:
    const Mesh& get_mesh() { return mesh; }
    SparseMat generateGlobalMatrix();
    void regenerateConstraints();
    uint32_t SplitAlongXZ(uint32_t cuboid_id, float y_split);
    uint32_t SplitAlongXY(uint32_t cuboid_id, float z_split);
    uint32_t SplitAlongYZ(uint32_t cuboid_id, float x_split);
    robin_hood::unordered_map<uint64_t, Face<N, C>> constraints;
    SplineMesh(Mesh&& mesh) { this->mesh = mesh; regenerateConstraints(); constraintsValid = true; }
    SplineMesh() = default;
    SplineMesh(int Nx, int Ny, int Nz) : mesh(Nx, Ny, Nz) {}
    ~SplineMesh() = default;
};

template<int N, int C>
inline void SplineMesh<N, C>::insertFaceConstraints(HalfFacePair pair, bool virtualSplitLeft)
{
    const halfFace hf = pair.first;
    const halfFace twin = pair.second;
    const auto key = toKey(pair);
    if (constraints.find(key) == constraints.end()) {
        if (hf.getLocalId() == 1) {
            constraints.insert({ key, Face<N,C>{ genLMatrix<N,C,Axis::z>(mesh, pair, virtualSplitLeft), genRMatrix<N,C,Axis::z>(mesh, pair, mesh.Twin(twin) != hf) } });
        }
        else if (hf.getLocalId() == 3) {
            constraints.insert({ key, Face<N,C>{ genLMatrix<N,C,Axis::x>(mesh, pair, virtualSplitLeft), genRMatrix<N,C,Axis::x>(mesh, pair, mesh.Twin(twin) != hf) } });
        }
        else {
            constraints.insert({ key, Face<N,C>{ genLMatrix<N,C,Axis::y>(mesh, pair, virtualSplitLeft), genRMatrix<N,C,Axis::y>(mesh, pair, mesh.Twin(twin) != hf) } });
        }
    }
}

template<int N, int C>
inline void SplineMesh<N, C>::invalidateConstraints(uint32_t cuboid_id)
{
    constraintsValid = false;
    // Invalidate all constraints of this cuboid
    for (size_t hf = 0; hf < 6; hf++)
    {
        const auto cur_hf = halfFace(cuboid_id, hf);
        const auto twin = mesh.Twin(cur_hf);
        if (twin.isBorder()) continue;
        if (twin.isSubdivided()) {
            for (auto it = mesh.getSft().cbegin(twin); it != mesh.getSft().cend(); ++it)
            {
                const auto pair = HalfFacePair{ cur_hf, *it };
                constraints.erase(toKey(pair));
            }
        }
        else {
            const auto pair = HalfFacePair{ cur_hf, twin };
            constraints.erase(toKey(pair));
        }
    }
}

template<int N, int C>
inline SparseMat SplineMesh<N, C>::generateGlobalMatrix()
{
    if (!constraintsValid) regenerateConstraints();
    const size_t numConstraints = constraints.size();
    const auto subMatSize_N = (N + 1) * (N + 1) * (C + 1);
    const auto subMatSize_M = (N + 1) * (N + 1) * (N + 1);
    SparseMat global(numConstraints * subMatSize_N, numElements * subMatSize_M);
    // Iterate over all the constraints
    size_t i = 0;
    for (const auto& [key, Face] : constraints)
    {
        const auto pair = fromKey(key);
        // Perform rowwise insertion of the sub matrices in the global matrix
        for (size_t n = 0; n < subMatSize_N; n++)
        {
            global.startVec(i * subMatSize_N + n);
            if (pair.first.getCuboid() < pair.second.getCuboid()) {
                for (SparseMat::InnerIterator it(Face.lowerConstraint, n); it; ++it)
                {
                    global.insertBack(i * subMatSize_N + n, it.col() + pair.first.getCuboid() * subMatSize_M) = it.value();
                }
                for (SparseMat::InnerIterator it(Face.higherConstraint, n); it; ++it)
                {
                    global.insertBack(i * subMatSize_N + n, it.col() + pair.second.getCuboid() * subMatSize_M) = it.value();
                }
            }
            else {
                for (SparseMat::InnerIterator it(Face.higherConstraint, n); it; ++it)
                {
                    global.insertBack(i * subMatSize_N + n, it.col() + pair.second.getCuboid() * subMatSize_M) = it.value();
                }
                for (SparseMat::InnerIterator it(Face.lowerConstraint, n); it; ++it)
                {
                    global.insertBack(i * subMatSize_N + n, it.col() + pair.first.getCuboid() * subMatSize_M) = it.value();
                }
            }
        }
        i++;
    }
    global.finalize();
    global.makeCompressed();
    return global;
}

template<int N, int C>
void SplineMesh<N, C>::regenerateConstraints() {
    numElements = mesh.getCuboids().size();
    // Loop over all the halfFaces with local id 1,2,3 in the mesh
    for (size_t cub = 0; cub < mesh.getCuboids().size(); ++cub)
    {
        for (size_t hf = 0; hf < 6; hf++)
        {
            if (hf == 0 || hf == 4 || hf == 5) continue;
            const auto cur_hf = halfFace(cub, hf);
            const auto twin = mesh.Twin(cur_hf);
            if (twin.isBorder()) continue;
            if (twin.isSubdivided()) {
                for (auto it = mesh.getSft().cbegin(twin); it != mesh.getSft().cend(); ++it)
                {
                    const auto pair = HalfFacePair{ cur_hf, *it };
                    insertFaceConstraints(pair, true);
                }
            }
            else {
                const auto pair = HalfFacePair{ cur_hf, twin };
                insertFaceConstraints(pair, false);
            }
        }
    }
}

template<int N, int C>
inline uint32_t SplineMesh<N, C>::SplitAlongXZ(uint32_t cuboid_id, float y_split)
{
    invalidateConstraints(cuboid_id);
    return mesh.SplitAlongXZ(cuboid_id, y_split);
}

template<int N, int C>
inline uint32_t SplineMesh<N, C>::SplitAlongXY(uint32_t cuboid_id, float z_split)
{
    invalidateConstraints(cuboid_id);
    return mesh.SplitAlongXY(cuboid_id, z_split);
}

template<int N, int C>
inline uint32_t SplineMesh<N, C>::SplitAlongYZ(uint32_t cuboid_id, float x_split)
{
    invalidateConstraints(cuboid_id);
    return mesh.SplitAlongYZ(cuboid_id, x_split);
}