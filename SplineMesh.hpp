#pragma once
#include "Splines.hpp"
#include "QuantitiesOfInterest.hpp"
#include "lean_vtk.hpp"

struct LocalNullSpace {
    Eigen::MatrixXf kernel;
    std::vector<int> coefficients;
    std::array<uint32_t, 8> elements;
    uint32_t numControlPointsElement() { return coefficients.size()/8; }
};

/*
* Stores all the continuity matrices for all faces
*/
template<int Nx, int Cx, int Ny = Nx, int Cy = Cx, int Nz = Nx, int Cz = Cx>
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
    template<int i>
    LocalNullSpace LocalNullspace(uint32_t vertex);
    SparseMat generateGlobalMatrix();
    void regenerateConstraints();
    void renderBasis(const std::string& filename, Eigen::VectorXf basis, uint32_t samples);
    uint32_t SplitAlongXZ(uint32_t cuboid_id, float y_split);
    uint32_t SplitAlongXY(uint32_t cuboid_id, float z_split);
    uint32_t SplitAlongYZ(uint32_t cuboid_id, float x_split);
    uint32_t numControlPoints() const { return mesh.getCuboids().size() * (Nx + 1) * (Ny + 1) * (Nz + 1); }
    static uint32_t numControlPointsElement() { return (Nx + 1) * (Ny + 1) * (Nz + 1); }
    robin_hood::unordered_map<uint64_t, Face> constraints;
    SplineMesh(Mesh&& mesh) { this->mesh = mesh; regenerateConstraints(); constraintsValid = true; }
    SplineMesh() = default;
    SplineMesh(int Cellsx, int Cellsy, int Cellsz) : mesh(Cellsx, Cellsy, Cellsz) {}
    ~SplineMesh() = default;
};

template<int Nx, int Cx, int Ny, int Cy, int Nz, int Cz>
inline void SplineMesh<Nx, Cx, Ny, Cy, Nz, Cz>::insertFaceConstraints(HalfFacePair pair, bool virtualSplitLeft)
{
    const halfFace hf = pair.first;
    const halfFace twin = pair.second;
    const auto key = toKey(pair);
    if (constraints.find(key) == constraints.end()) {
        if (hf.getLocalId() == 1) {
            constraints.insert({ key, Face{ genLMatrix<Axis::z, Nx, Cx, Ny, Cy, Nz, Cz>(mesh, pair, virtualSplitLeft), genRMatrix<Axis::z, Nx, Cx, Ny, Cy, Nz, Cz>(mesh, pair, mesh.Twin(twin) != hf) } });
        }
        else if (hf.getLocalId() == 3) {
            constraints.insert({ key, Face{ genLMatrix<Axis::x, Nx, Cx, Ny, Cy, Nz, Cz>(mesh, pair, virtualSplitLeft), genRMatrix<Axis::x, Nx, Cx, Ny, Cy, Nz, Cz>(mesh, pair, mesh.Twin(twin) != hf) } });
        }
        else {
            constraints.insert({ key, Face{ genLMatrix<Axis::y, Nx, Cx, Ny, Cy, Nz, Cz>(mesh, pair, virtualSplitLeft), genRMatrix<Axis::y, Nx, Cx, Ny, Cy, Nz, Cz>(mesh, pair, mesh.Twin(twin) != hf) } });
        }
    }
}

template<int Nx, int Cx, int Ny, int Cy, int Nz, int Cz>
inline void SplineMesh<Nx, Cx, Ny, Cy, Nz, Cz>::invalidateConstraints(uint32_t cuboid_id)
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

template<int Nx, int Cx, int Ny, int Cy, int Nz, int Cz>
inline SparseMat SplineMesh<Nx, Cx, Ny, Cy, Nz, Cz>::generateGlobalMatrix()
{
    if (!constraintsValid) regenerateConstraints();
    const size_t numConstraints = constraints.size();
    const int max_degree = std::max({ Nx, Ny, Nz });
    const int max_continuity = std::max({Cx, Cy, Cz});
    // Conservatively size rows per face
    const auto subMatSize_M = (Nx + 1) * (Ny + 1) * (Nz + 1);
    const auto subMatSize_N = (max_degree + 1) * (max_degree + 1) * (max_continuity + 1);
    SparseMat global(numConstraints * subMatSize_N, numElements * subMatSize_M);
    // Iterate over all the constraints
    size_t row = 0;
    for (const auto& [key, Face] : constraints)
    {
        const auto pair = fromKey(key);
        // Perform rowwise insertion of the sub matrices in the global matrix
        for (size_t n = 0; n < Face.higherConstraint.rows(); n++)
        {
            global.startVec(row);
            if (pair.first.getCuboid() < pair.second.getCuboid()) {
                for (SparseMat::InnerIterator it(Face.lowerConstraint, n); it; ++it)
                {
                    global.insertBack(row, it.col() + pair.first.getCuboid() * subMatSize_M) = it.value();
                }
                for (SparseMat::InnerIterator it(Face.higherConstraint, n); it; ++it)
                {
                    global.insertBack(row, it.col() + pair.second.getCuboid() * subMatSize_M) = it.value();
                }
            }
            else {
                for (SparseMat::InnerIterator it(Face.higherConstraint, n); it; ++it)
                {
                    global.insertBack(row, it.col() + pair.second.getCuboid() * subMatSize_M) = it.value();
                }
                for (SparseMat::InnerIterator it(Face.lowerConstraint, n); it; ++it)
                {
                    global.insertBack(row, it.col() + pair.first.getCuboid() * subMatSize_M) = it.value();
                }
            }
            row++;
        }
    }
    global.conservativeResize(row, numElements* subMatSize_M);
    global.finalize();
    global.makeCompressed();
    return global;
}

template<int Nx, int Cx, int Ny, int Cy, int Nz, int Cz>
inline void SplineMesh<Nx, Cx, Ny, Cy, Nz, Cz>::regenerateConstraints() {
    static_assert(Cx < Nx, "Smoothness degree must be lower than the spline degree");
    static_assert(Cy < Ny, "Smoothness degree must be lower than the spline degree");
    static_assert(Cz < Nz, "Smoothness degree must be lower than the spline degree");
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

template<int Nx, int Cx, int Ny, int Cy, int Nz, int Cz>
inline void SplineMesh<Nx, Cx, Ny, Cy, Nz, Cz>::renderBasis(const std::string& filename, Eigen::VectorXf basis, uint32_t samples)
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
                    spline_val.emplace_back(VolumeSpline<float, Nx, Ny, Nz>(u, v, w, basis.middleRows(cube * (Nx + 1) * (Ny + 1) * (Nz + 1), (Nx + 1) * (Ny + 1) * (Nz + 1))));
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

template<int Nx, int Cx, int Ny, int Cy, int Nz, int Cz>
inline uint32_t SplineMesh<Nx, Cx, Ny, Cy, Nz, Cz>::SplitAlongXZ(uint32_t cuboid_id, float y_split)
{
    invalidateConstraints(cuboid_id);
    return mesh.SplitAlongXZ(cuboid_id, y_split);
}

template<int Nx, int Cx, int Ny, int Cy, int Nz, int Cz>
inline uint32_t SplineMesh<Nx, Cx, Ny, Cy, Nz, Cz>::SplitAlongXY(uint32_t cuboid_id, float z_split)
{
    invalidateConstraints(cuboid_id);
    return mesh.SplitAlongXY(cuboid_id, z_split);
}

template<int Nx, int Cx, int Ny, int Cy, int Nz, int Cz>
inline uint32_t SplineMesh<Nx, Cx, Ny, Cy, Nz, Cz>::SplitAlongYZ(uint32_t cuboid_id, float x_split)
{
    invalidateConstraints(cuboid_id);
    return mesh.SplitAlongYZ(cuboid_id, x_split);
}

template<int Nx, int Cx, int Ny, int Cy, int Nz, int Cz>
template<int i>
inline LocalNullSpace SplineMesh<Nx, Cx, Ny, Cy, Nz, Cz>::LocalNullspace(uint32_t vertex)
{
    static_assert(i < std::min({ Nx,Ny,Nz }), "We must have at least two non zero coefficients per direction");
    static_assert(i > 0, "i must be > 0");
    // Check if the vertex is conformal
    QuantitiesOfInterest q(mesh);
    const auto [elements, num] = q.vertexConnectivity(vertex);
    if (num != 8) return {};
    if (!constraintsValid) regenerateConstraints();
    const auto subMatSize_N_z = (Ny + 1) * (Nx + 1) * (Cz + 1);
    const auto subMatSize_N_y = (Nz + 1) * (Nx + 1) * (Cy + 1);
    const auto subMatSize_N_x = (Nz + 1) * (Ny + 1) * (Cx + 1);
    const auto subMatSize_M = (Nz - i + 1) * (Ny - i + 1) * (Nx - i + 1);
    Eigen::MatrixXf localMatrix(4 * subMatSize_N_x + 4 * subMatSize_N_y + subMatSize_N_z * 4, 8 * subMatSize_M);
    std::vector<int> non_zeros;
    non_zeros.reserve(subMatSize_M * 8);
    localMatrix.setZero();
    uint32_t row{ 0 };
    uint8_t index{ 0 };
    std::for_each(elements.begin(), elements.end(), [&](uint32_t elem){
        // Find the local index of the vertex
        const auto& vertices = mesh.getCuboids()[elem].vertices;
        const uint8_t local_index = std::find(vertices.begin(), vertices.end(), vertex) - vertices.begin();
        // find the points which are non-zero
        static constexpr bool x_lower[] = { true, false, false, true, true, false, false, true };
        static constexpr bool y_lower[] = { true, true, false, false, true, true, false, false };
        static constexpr bool z_lower[] = { true, true, true, true, false, false, false, false };
        for (int z_i = 0; z_i < Nz - i + 1; z_i++)
        {
            for (int y_i = 0; y_i < Ny - i + 1; y_i++)
            {
                for (int x_i = 0; x_i < Nx - i + 1; x_i++)
                {
                    int Z_i = z_i + (z_lower[local_index] ? 0 : i);
                    int Y_i = y_i + (y_lower[local_index] ? 0 : i);
                    int X_i = x_i + (x_lower[local_index] ? 0 : i);
                    non_zeros.push_back(Z_i * (Nx + 1) * (Ny + 1) + Y_i * (Nx + 1) + X_i);
                }
            }
        }
    });
    for (const auto elem : elements)
    {
        // Find the local index of the vertex
        const auto& vertices = mesh.getCuboids()[elem].vertices;
        const uint8_t local_index = std::find(vertices.begin(), vertices.end(), vertex) - vertices.begin();
        if (local_index > 7) return {};
        // find the touching faces constraints
        for (uint8_t l_face : Lv2Hf[local_index]) {
            if (l_face < 1 || l_face > 3) continue;
            halfFace face = halfFace(elem, l_face);
            halfFace twin = mesh.Twin(face);
            if (twin.isSubdivided())
            {
                // find the halfface which contains the vertex
                twin = *mesh.getSft().find(twin, mesh.getVertices()[vertex]);
            }
            uint32_t second_index = std::find(elements.begin(), elements.end(), twin.getCuboid()) - elements.begin();
            //find the given constraint
            auto constraintFace = (*constraints.find(toKey({face,twin}))).second;
            // iterate over all the columns
            for (int col = 0; col < subMatSize_M; ++col) {
                localMatrix.block(row, index * subMatSize_M + col, constraintFace.lowerConstraint.rows(), 1) = constraintFace.lowerConstraint.col(non_zeros[index * subMatSize_M + col]);
                localMatrix.block(row, second_index * subMatSize_M + col, constraintFace.higherConstraint.rows(), 1) = constraintFace.higherConstraint.col(non_zeros[second_index * subMatSize_M + col]);
            }
            row += constraintFace.lowerConstraint.rows();
        }
        index++;
    }
    auto QR = Eigen::FullPivHouseholderQR<Eigen::MatrixXf>(localMatrix.transpose());
    auto Q = QR.matrixQ();
    auto kernel = Q.block(0, QR.rank(), Q.rows(), Q.cols() - QR.rank());
    return LocalNullSpace{kernel, non_zeros, elements};
}
