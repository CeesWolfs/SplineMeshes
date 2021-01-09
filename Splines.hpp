/*
* 16 3D points per element->
*
*/
#include <robin_hood.h>
#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <unsupported/Eigen/KroneckerProduct>
#include "Types.hpp"
#include "Mesh.hpp"

using SparseMat = Eigen::SparseMatrix<float, Eigen::RowMajor>;

template<int N>
static constexpr Eigen::Matrix<float, N, N> Identity() {
    return Eigen::Matrix<float, N, N>::Identity();
}

// Bit of c++ hackery to get compile time pascal triangle generation
template<int N>
struct PascalTriangle
{
    constexpr PascalTriangle() : values()
    {
        for (auto line = 1; line <= N; ++line)
        {
            int C = 1;
            for (int i = 1; i <= line; i++)
            {
                values[line - 1][i - 1] = C;
                C = C * (line - i) / i;
            }
        }
    }
    int values[N][N];
};

constexpr inline float bernstein(float u, int n, int i) {
    float x = 1 - u;
    float res = 1;
    for (int k = 0; k < (n - i); k++)
    {
        res *= x;
    }
    for (int k = 0; k < i; k++)
    {
        res *= u;
    }
    return res;
}

// N -> Degree of the spline Polynomial
template<int N>
static inline auto SRHS(float a) {
    static constexpr auto comb = PascalTriangle<N + 1>();
    Eigen::Matrix<float, N + 1, N + 1> res = Eigen::Matrix<float, N + 1, N + 1>::Zero();
    for (int i = 0; i <= N; i++)
    {
        for (int j = i; j <= N; j++) {
            res(i, j) = bernstein(a, N - i, N - j) * comb.values[N - i][N - j];
        }
    }
    return res;
}

// N -> Degree of the spline Polynomial
template<int N>
static inline auto SLHS(float a)
{
    static constexpr auto comb = PascalTriangle<N + 1>();
    Eigen::Matrix<float, N + 1, N + 1> res = Eigen::Matrix<float, N + 1, N + 1>::Zero();
    for (int i = 0; i <= N; i++)
    {
        for (int j = 0; j <= i; j++) {
            res(i, j) = bernstein(a, i, j) * comb.values[i][j];
        }
    }
    return res;
}

// Generate the continuity for the left hand side so u->0
// N: Degree of the b-spline
// C: to what continuity condition 0: endpoints, 1: first derivative ....
template<int N, int C>
auto CLHS(const float h) {
    Eigen::Matrix<float, C + 1, N + 1> res = Eigen::Matrix<float, C + 1, N + 1>::Zero();
    static_assert(C < N, "Smoothness degree must be lower than the spline degree");
    static constexpr auto comb = PascalTriangle<N>();
    float multiplier = 1;
    for (auto i = 0; i <= C; i++)
    {
        float sign = 1;
        for (auto j = 0; j <= i; j++)
        {
            res(i, N - j) = multiplier * comb.values[i][j] * sign;
            sign = -sign;
        }
        multiplier /= h;
    }
    return res;
}

// Generate the continuity for the right hand side so u->1 
// Generates the negated matrix for convenience
// N: Degree of the b-spline
// C: to what continuity condition 0: endpoints, 1: first derivative ....
template<int N, int C>
auto CRHS(const float h) {
    Eigen::Matrix<float, C + 1, N + 1> res = Eigen::Matrix<float, C + 1, N + 1>::Zero();
    static_assert(C < N, "Smoothness degree must be lower than the spline degree");
    static constexpr auto comb = PascalTriangle<N>();
    float multiplier = 1;
    for (auto i = 0; i <= C; i++)
    {
        // Generate all right constraints with a minus
        float sign = -1;
        for (auto j = i; j >= 0; j--)
        {
            res(i, j) = multiplier * comb.values[i][j] * sign;
            sign = -sign;
        }
        multiplier /= h;
    }
    return res;
}

template<int N>
static inline const auto STgen(std::pair<float, float> bounds, std::pair<float, float> twin_bounds)
{
    if (((twin_bounds.first - bounds.first) > eps) && ((bounds.second - twin_bounds.second) > eps)) {
        // Split on the left and on the right side
        const float a0 = (twin_bounds.second - bounds.first) / (bounds.second - bounds.first);
        const float a1 = (twin_bounds.first - bounds.first) / (bounds.second - bounds.first);
        return (SRHS<N>(a1) * SLHS<N>(a0)).eval();
    }
    else if ((twin_bounds.first - bounds.first) > eps) {
        // Only split on the right side
        const float a1 = (twin_bounds.first - bounds.first) / (bounds.second - bounds.first);
        return SRHS<N>(a1);
    }
    else if ((bounds.second - twin_bounds.second) > eps) {
        // Only split on the left side
        const float a0 = (twin_bounds.second - bounds.first) / (bounds.second - bounds.first);
        return SLHS<N>(a0);
    }
    else {
        // Do not split
        return Identity<N+1>();
    }
}


// Store the constraints that correspond to a pair of halfFaces
template<int N, int C>
struct Face
{
    SparseMat lowerConstraint;
    SparseMat higherConstraint;
};

inline float getDepth(std::pair<Vertex, Vertex> corners, Axis ax) {
    const auto depth = corners.second - corners.first;
    if (ax == Axis::x) {
        return depth.x;
    }
    else if (ax == Axis::y) {
        return depth.y;
    }
    return depth.z;
}

inline std::pair<Vertex, Vertex> getCorners(const Mesh& mesh, halfFace hf) {
    const uint32_t elem = hf.getCuboid();
    const uint8_t local_id = hf.getLocalId();
    const auto bl_corner = mesh.getVertices()[mesh.getCuboids()[elem].v1];
    const auto tr_corner = mesh.getVertices()[mesh.getCuboids()[elem].v7];
    return { bl_corner, tr_corner };
}

template<int N, int C, Axis ax>
static inline const SparseMat genRMatrix(const Mesh& mesh, HalfFacePair pair, bool needsSplit)
{
    const auto first_corners = getCorners(mesh, pair.second);
    const auto second_corners = getCorners(mesh, pair.first);
    const float h_right = getDepth(first_corners, ax) / N;
    if (!needsSplit) {
        if constexpr (ax == Axis::x) {
            SparseMat res = Eigen::kroneckerProduct(Identity<(N + 1)* (N + 1)>(), CRHS<N, C>(h_right)).sparseView();
            res.makeCompressed();
            return res;
        }
        else if (ax == Axis::y) {
            SparseMat res = Eigen::kroneckerProduct(Identity<N + 1>(), Eigen::kroneckerProduct(CRHS<N, C>(h_right), Identity<N + 1>())).sparseView();
            res.makeCompressed();
            return res;
        }
        else {
            SparseMat res = Eigen::kroneckerProduct(CRHS<N, C>(h_right), Identity<(N + 1)* (N + 1)>()).sparseView();
            res.makeCompressed();
            return res;
        }
    }
    const auto X = [&]() { 
        if constexpr (ax == Axis::x) return CRHS<N, C>(h_right);
        else return STgen<N>({ first_corners.first.x, first_corners.second.x }, { second_corners.first.x, second_corners.second.x }); }();
    const auto Y = [&]() { 
        if constexpr (ax == Axis::y) return CRHS<N, C>(h_right);
        else return STgen<N>({ first_corners.first.y, first_corners.second.y }, { second_corners.first.y, second_corners.second.y }); }();
    const auto Z = [&]() { 
        if constexpr (ax == Axis::z) return CRHS<N, C>(h_right);
        else return STgen<N>({ first_corners.first.z, first_corners.second.z }, { second_corners.first.z, second_corners.second.z }); }();
    SparseMat res = Eigen::kroneckerProduct(Z, Eigen::kroneckerProduct(Y, X)).sparseView();
    res.makeCompressed();
    return res;
}

template<int N, int C, Axis ax>
static inline const SparseMat genLMatrix(const Mesh& mesh, HalfFacePair pair, bool needsSplit)
{
    const auto first_corners = getCorners(mesh, pair.first);
    const auto second_corners = getCorners(mesh, pair.second);
    const float h_left = getDepth(first_corners, ax) / N;
    if (!needsSplit) {
        if constexpr (ax == Axis::x) {
            SparseMat res =  Eigen::kroneckerProduct(Identity<(N + 1)* (N + 1)>(), CLHS<N, C>(h_left)).sparseView();
            res.makeCompressed();
            return res;
        }
        else if (ax == Axis::y) {
            SparseMat res = Eigen::kroneckerProduct(Identity<N + 1>(), Eigen::kroneckerProduct(CLHS<N, C>(h_left), Identity<N + 1>())).sparseView();
            res.makeCompressed();
            return res;
        }
        else {
            SparseMat res =  Eigen::kroneckerProduct(CLHS<N, C>(h_left), Identity<(N + 1)* (N + 1)>()).sparseView();
            res.makeCompressed();
            return res;
        }
    }
    const auto X = [&]() { 
        if constexpr (ax == Axis::x) return CLHS<N, C>(h_left);
        else return STgen<N>({ first_corners.first.x, first_corners.second.x }, { second_corners.first.x, second_corners.second.x }); }();
    const auto Y = [&]() { 
        if constexpr (ax == Axis::y) return CLHS<N, C>(h_left);
        else return STgen<N>({ first_corners.first.y, first_corners.second.y }, { second_corners.first.y, second_corners.second.y }); }();
    const auto Z = [&]() { 
        if constexpr (ax == Axis::z) return CLHS<N, C>(h_left);
        else return STgen<N>({ first_corners.first.z, first_corners.second.z }, { second_corners.first.z, second_corners.second.z }); }();
    SparseMat res = Eigen::kroneckerProduct(Z, Eigen::kroneckerProduct(Y, X)).sparseView();
    res.makeCompressed();
    return res;
}

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
    static const uint64_t toKey(HalfFacePair pair) { return static_cast<uint64_t>(pair.first.id) + (static_cast<uint64_t>(pair.second.id) << 32); }
    static const HalfFacePair fromKey(uint64_t key) { return { halfFace(static_cast<uint32_t>(key)), halfFace(static_cast<uint32_t>(key >> 32)) }; }
public:
    SparseMat generateGlobalMatrix() const;
    void regenerateConstraints();
    robin_hood::unordered_map<uint64_t, Face<N,C>> constraints;
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
inline SparseMat SplineMesh<N,C>::generateGlobalMatrix() const
{
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
void SplineMesh<N,C>::regenerateConstraints() {
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
};
