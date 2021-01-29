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

// N -> Maximum degree of the spline
template<int N>
constexpr inline float bernstein(float u, int n, int i) {
    static constexpr auto comb = PascalTriangle<N + 1>();
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
    return res*comb.values[n][i];
}

// N -> Degree of the spline Polynomial
// ControlType -> A scalar (float/double) or vector for multidimensional spline output
// Evaluates a volumetric spline at a certain point
template<typename ControlType, int Nx, int Ny = Nx, int Nz = Nx>
inline ControlType VolumeSpline(float u, float v, float w, Eigen::Matrix<ControlType, (Nx+1)*(Ny+1)*(Nz+1), 1> control_points) {
    ControlType result = 0;
    for (size_t i0 = 0; i0 <= Nz; i0++)
    {
        for (size_t i1 = 0; i1 <= Ny; i1++)
        {
            for (size_t i2 = 0; i2 <= Nx; i2++)
            {
                result += bernstein<Nx>(u, Nx, i2) * bernstein<Ny>(v, Ny, i1) * bernstein<Nz>(w, Nz, i0) * control_points[i2 + i1 * (Nx+1) + i0 * (Nx+1)*(Ny+1)];
            }
        }
    }
    return result;
}

// N -> Degree of the spline Polynomial
template<int N>
static inline auto SRHS(float a) {
    Eigen::Matrix<float, N + 1, N + 1> res = Eigen::Matrix<float, N + 1, N + 1>::Zero();
    for (int i = 0; i <= N; i++)
    {
        for (int j = i; j <= N; j++) {
            res(i, j) = bernstein<N>(a, N - i, j - i);
        }
    }
    return res;
}

// N -> Degree of the spline Polynomial
template<int N>
static inline auto SLHS(float a)
{
    Eigen::Matrix<float, N + 1, N + 1> res = Eigen::Matrix<float, N + 1, N + 1>::Zero();
    for (int i = 0; i <= N; i++)
    {
        for (int j = 0; j <= i; j++) {
            res(i, j) = bernstein<N>(a, i, j);
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
    static constexpr auto comb = PascalTriangle<std::max(C + 1, N + 1)>();
    float multiplier = 1;
    for (auto i = 0; i <= C; i++)
    {
        float sign = 1;
        for (auto j = 0; j <= std::min(i, N); j++)
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
    static constexpr auto comb = PascalTriangle<std::max(C+1,N+1)>();
    float multiplier = 1;
    for (auto i = 0; i <= C; i++)
    {
        // Generate all right constraints with a minus
        float sign = -1;
        for (auto j = std::min(i, N); j >= 0; j--)
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

template<Axis ax, int Nx, int Cx, int Ny=Nx, int Cy=Cx, int Nz=Nx, int Cz=Cx>
static inline const SparseMat genRMatrix(const Mesh& mesh, HalfFacePair pair, bool needsSplit)
{
    const auto first_corners = getCorners(mesh, pair.second);
    const auto second_corners = getCorners(mesh, pair.first);
    if (!needsSplit) {
        if constexpr (ax == Axis::x) {
            const float h_right = getDepth(first_corners, ax) / Nx;
            SparseMat res = Eigen::kroneckerProduct(Identity<(Nz + 1)* (Ny + 1)>(), CRHS<Nx, Cx>(h_right)).sparseView();
            res.makeCompressed();
            return res;
        }
        else if (ax == Axis::y) {
            const float h_right = getDepth(first_corners, ax) / Ny;
            SparseMat res = Eigen::kroneckerProduct(Identity<Nz + 1>(), Eigen::kroneckerProduct(CRHS<Ny, Cy>(h_right), Identity<Nx + 1>())).sparseView();
            res.makeCompressed();
            return res;
        }
        else {
            const float h_right = getDepth(first_corners, ax) / Nz;
            SparseMat res = Eigen::kroneckerProduct(CRHS<Nz, Cz>(h_right), Identity<(Ny + 1)* (Nx + 1)>()).sparseView();
            res.makeCompressed();
            return res;
        }
    }
    const auto X = [&]() { 
        const float h_right = getDepth(first_corners, ax) / Nx;
        if constexpr (ax == Axis::x) return CRHS<Nx, Cx>(h_right);
        else return STgen<Nx>({ first_corners.first.x, first_corners.second.x }, { second_corners.first.x, second_corners.second.x }); }();
    const auto Y = [&]() { 
        const float h_right = getDepth(first_corners, ax) / Ny;
        if constexpr (ax == Axis::y) return CRHS<Ny, Cy>(h_right);
        else return STgen<Ny>({ first_corners.first.y, first_corners.second.y }, { second_corners.first.y, second_corners.second.y }); }();
    const auto Z = [&]() { 
        const float h_right = getDepth(first_corners, ax) / Nz;
        if constexpr (ax == Axis::z) return CRHS<Nz, Cz>(h_right);
        else return STgen<Nz>({ first_corners.first.z, first_corners.second.z }, { second_corners.first.z, second_corners.second.z }); }();
    SparseMat res = Eigen::kroneckerProduct(Z, Eigen::kroneckerProduct(Y, X)).sparseView();
    res.makeCompressed();
    return res;
}

template<Axis ax, int Nx, int Cx, int Ny = Nx, int Cy = Cx, int Nz = Nx, int Cz = Cx>
static inline const SparseMat genLMatrix(const Mesh& mesh, HalfFacePair pair, bool needsSplit)
{
    const auto first_corners = getCorners(mesh, pair.first);
    const auto second_corners = getCorners(mesh, pair.second);
    if (!needsSplit) {
        if constexpr (ax == Axis::x) {
            const float h_left = getDepth(first_corners, ax) / Nx;
            SparseMat res =  Eigen::kroneckerProduct(Identity<(Nz + 1)* (Ny + 1)>(), CLHS<Nx, Cx>(h_left)).sparseView();
            res.makeCompressed();
            return res;
        }
        else if (ax == Axis::y) {
            const float h_left = getDepth(first_corners, ax) / Ny;
            SparseMat res = Eigen::kroneckerProduct(Identity<Nz + 1>(), Eigen::kroneckerProduct(CLHS<Ny, Cy>(h_left), Identity<Nx + 1>())).sparseView();
            res.makeCompressed();
            return res;
        }
        else {
            const float h_left = getDepth(first_corners, ax) / Nz;
            SparseMat res =  Eigen::kroneckerProduct(CLHS<Nz, Cz>(h_left), Identity<(Ny + 1)* (Nx + 1)>()).sparseView();
            res.makeCompressed();
            return res;
        }
    }
    const auto X = [&]() { 
        const float h_left = getDepth(first_corners, ax) / Nx;
        if constexpr (ax == Axis::x) return CLHS<Nx, Cx>(h_left);
        else return STgen<Nx>({ first_corners.first.x, first_corners.second.x }, { second_corners.first.x, second_corners.second.x }); }();
    const auto Y = [&]() { 
        const float h_left = getDepth(first_corners, ax) / Ny;
        if constexpr (ax == Axis::y) return CLHS<Ny, Cy>(h_left);
        else return STgen<Ny>({ first_corners.first.y, first_corners.second.y }, { second_corners.first.y, second_corners.second.y }); }();
    const auto Z = [&]() { 
        const float h_left = getDepth(first_corners, ax) / Nz;
        if constexpr (ax == Axis::z) return CLHS<Nz, Cz>(h_left);
        else return STgen<Nz>({ first_corners.first.z, first_corners.second.z }, { second_corners.first.z, second_corners.second.z }); }();
    SparseMat res = Eigen::kroneckerProduct(Z, Eigen::kroneckerProduct(Y, X)).sparseView();
    res.makeCompressed();
    return res;
}
