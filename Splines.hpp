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
template<int N, typename ControlType>
inline ControlType VolumeSpline(float u, float v, float w, Eigen::Matrix<ControlType, (N+1)*(N+1)*(N+1), 1> control_points) {
    ControlType result = 0;
    for (size_t i0 = 0; i0 <= N; i0++)
    {
        for (size_t i1 = 0; i1 <= N; i1++)
        {
            for (size_t i2 = 0; i2 <= N; i2++)
            {
                result += bernstein<N>(u, N, i2) * bernstein<N>(v, N, i1) * bernstein<N>(w, N, i0) * control_points[i2 + i1 * (N+1) + i0 * (N+1)*(N+1)];
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
