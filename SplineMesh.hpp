#pragma once
#include "Splines.hpp"
#include "lean_vtk.hpp"
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
    int LocalNullspace(uint32_t vertex);
    SparseMat generateGlobalMatrix();
    void regenerateConstraints();
    void renderBasis(const std::string& filename, Eigen::VectorXf basis, uint32_t samples);
    uint32_t SplitAlongXZ(uint32_t cuboid_id, float y_split);
    uint32_t SplitAlongXY(uint32_t cuboid_id, float z_split);
    uint32_t SplitAlongYZ(uint32_t cuboid_id, float x_split);
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
inline int SplineMesh<Nx, Cx, Ny, Cy, Nz, Cz>::LocalNullspace(uint32_t vertex)
{
    // Check if the vertex is conformal
    localVertex lv = mesh.getV2lV()[vertex];
    uint32_t x = 0;
    std::array<uint32_t, 8> elements{-1,-1,-1,-1,-1,-1,-1,-1};
    auto moveToNext = [&](uint32_t& cuboid, uint8_t direction) {
        auto twin = mesh.Twin(halfFace(cuboid, direction));
        if (twin.isBorder()) return false;
        if (twin.isSubdivided()) { return false; }
        auto new_cuboid = twin.getCuboid();
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
        else { return 0; }
        currentCuboid = temp;
        moveToNext(currentCuboid, directions[2]);
    }
    else { return 0; }
    currentCuboid = lv.getCuboid();
    if (moveToNext(currentCuboid, directions[1])) {
        moveToNext(currentCuboid, directions[2]);
    }
    else { return 0; }
    currentCuboid = lv.getCuboid();
    moveToNext(currentCuboid, directions[2]);
    if (x != 8) return 0;
    const auto subMatSize_N_z = (Ny-i + 1) * (Nx-i + 1) * (Cz + 1);
    const auto subMatSize_N_y = (Nz - i + 1) * (Nx - i + 1) * (Cy + 1);
    const auto subMatSize_N_x = (Nz - i + 1) * (Ny - i + 1) * (Cx + 1);
    const auto subMatSize_M = (Nz-i + 1) * (Ny-i + 1) * (Nx-i + 1);
    Eigen::MatrixXf localMatrix(4 * subMatSize_N_x + 4 * subMatSize_N_y * 4 + subMatSize_N_z * 4, 8 * subMatSize_M);
    uint32_t row{ 0 };
    uint8_t index{ 0 };
    for (const auto elem: elements)
    {
        // Find the local index of the vertex
        const auto& vertices = mesh.getCuboids()[elem].vertices;
        const uint8_t local_index = std::find(vertices.begin(), vertices.end(), vertex) - vertices.begin();
        if (local_index > 7) return 0;
        // The faces which contain
        for (uint8_t local_face : Lv2Hf[local_index]) {
            if (local_face < 1 || local_face > 3) continue;
            // build constraint
            halfFace face = halfFace(elem, local_face);
            const halfFace twin = mesh.Twin(face);
            // find the twin element in the local elements
            uint8_t second_elem = std::find(elements.begin(), elements.end(), twin.getCuboid()) - elements.begin();
            if (face.getLocalId() == 1) {
                localMatrix.block(row, subMatSize_M * index, subMatSize_N_z, subMatSize_M) = genLMatrix<Axis::z, Nx - i, Cx, Ny - i, Cy, Nz - i, Cz>(mesh, { face,twin }, false);
                localMatrix.block(row, subMatSize_M * second_elem, subMatSize_N_z, subMatSize_M) = genRMatrix<Axis::z, Nx - i, Cx, Ny - i, Cy, Nz - i, Cz>(mesh, { face,twin }, false);
                row += subMatSize_N_z;
            }
            else if (face.getLocalId() == 2) {
                localMatrix.block(row, subMatSize_M * index, subMatSize_N_y, subMatSize_M) = genLMatrix<Axis::y, Nx - i, Cx, Ny - i, Cy, Nz - i, Cz>(mesh, { face,twin }, false);
                localMatrix.block(row, subMatSize_M * second_elem, subMatSize_N_y, subMatSize_M) = genRMatrix<Axis::y, Nx - i, Cx, Ny - i, Cy, Nz - i, Cz>(mesh, { face,twin }, false);
                row += subMatSize_N_y;
            }
            else {
                localMatrix.block(row, subMatSize_M * index, subMatSize_N_x, subMatSize_M) = genLMatrix<Axis::x, Nx - i, Cx, Ny - i, Cy, Nz - i, Cz>(mesh, { face,twin }, false);
                localMatrix.block(row, subMatSize_M * second_elem, subMatSize_N_x, subMatSize_M) = genRMatrix<Axis::x, Nx - i, Cx, Ny - i, Cy, Nz - i, Cz>(mesh, { face,twin }, false);
                row += subMatSize_N_x;
            }
        }
        index++;
    }
    // Solve for the nullspace
    auto QR = Eigen::FullPivHouseholderQR<Eigen::MatrixXf>(localMatrix.transpose());
    auto Q = QR.matrixQ();
    auto kernel = Q.block(0, QR.rank(), Q.rows(), Q.cols() - QR.rank());
    //std::cout << kernel;
    return Q.cols() - QR.rank();
    return 0;
}
