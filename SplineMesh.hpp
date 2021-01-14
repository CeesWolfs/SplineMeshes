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
    void renderBasis(const std::string& filename, Eigen::VectorXf basis, uint32_t samples);
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
inline void SplineMesh<N, C>::renderBasis(const std::string& filename, Eigen::VectorXf basis, uint32_t samples)
{
    std::vector<double> points;
    std::vector<int> cells;
    uint32_t cell_size = 8;
    std::vector<double> spline_val;
    samples--;
    assert(samples > 0);
    // Render a spline basis
    for (int cube = 0; cube < mesh.getCuboids().size(); cube++) {
        const auto bl_corner = mesh.getVertices()[mesh.getCuboids()[cube].v1];
        const auto tr_corner = mesh.getVertices()[mesh.getCuboids()[cube].v7];
        const auto depth = tr_corner - bl_corner;
        // Render all points per cuboid
        for (size_t z_i = 0; z_i <= samples; z_i++)
        {
            float z = z_i * (depth.z / samples) + bl_corner.z;
            for (size_t y_i = 0; y_i <= samples; y_i++)
            {
                float y = y_i * (depth.y / samples) + bl_corner.y;
                for (size_t x_i = 0; x_i <= samples; x_i++)
                {
                    float x = x_i * (depth.x / samples) + bl_corner.x;
                    const auto u = static_cast<float>(x_i) / static_cast<float>(samples);
                    const auto v = static_cast<float>(y_i) / static_cast<float>(samples);
                    const auto w = static_cast<float>(z_i) / static_cast<float>(samples);
                    points.push_back(x);
                    points.push_back(y);
                    points.push_back(z);
                    spline_val.emplace_back(VolumeSpline<N, float>(u, v, w, basis.middleRows(cube * (N + 1) * (N + 1) * (N + 1), (N + 1) * (N + 1) * (N + 1))));
                }
            }
        }
        auto toVertIndex = [=](uint32_t i, uint32_t j, uint32_t k) {return cube * (samples + 1) * (samples + 1) * (samples + 1) + i + j * (samples + 1) + k * (samples + 1) * (samples + 1); };
        // Construct all the cuboids
        for (size_t k = 0; k < samples; k++)
        {
            for (size_t j = 0; j < samples; j++)
            {
                for (size_t i = 0; i < samples; i++)
                {
                    cells.push_back(toVertIndex(i, j, k));
                    cells.push_back(toVertIndex(i + 1, j, k));
                    cells.push_back(toVertIndex(i + 1, j + 1, k));
                    cells.push_back(toVertIndex(i, j + 1, k));
                    cells.push_back(toVertIndex(i, j, k + 1));
                    cells.push_back(toVertIndex(i + 1, j, k + 1));
                    cells.push_back(toVertIndex(i + 1, j + 1, k + 1));
                    cells.push_back(toVertIndex(i, j + 1, k + 1));
                }
            }
        }
    }
    leanvtk::VTUWriter writer;
    writer.add_scalar_field("Spline values", spline_val);
    writer.write_volume_mesh(filename, 3, 8, points, cells);
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