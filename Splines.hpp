/*
* 16 3D points per element->
*
*/
#include <robin_hood.h>
#include <blaze/blaze.h>
#include "Types.hpp"
#include "Mesh.hpp"

using SparseMat = blaze::CompressedMatrix<float>;

static constexpr int factorial(int n) {
    if (n <= 1) return 1;
    int res = 1;
    for (int i = 1; i < n; i++)
    {
        res *= i;
    }
    return res;
}

static constexpr int comb(int n, int k) {
    return factorial(n) / (factorial(n - k) * factorial(k));
}

constexpr float bernstein(float u, int n, int i) {
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
    return res * comb(n, i);
}

// N -> Degree of the spline Polynomial
template<int N>
static inline auto SRHS(float a) {
    blaze::StaticMatrix<float, N + 1, N + 1> res(0);
    for (int i = 0; i <= N; i++)
    {
        for (int j = 0; j <= N; j++) {
            if (i <= j) {
                res(i, j) = bernstein(a, N - i, N - j);
            }

        }
    }
    return res;
}

// N -> Degree of the spline Polynomial
template<int N>
static inline auto SLHS(float a)
{
    blaze::StaticMatrix<float, N + 1, N + 1> res(0);
    for (int i = 0; i <= N; i++)
    {
        for (int j = 0; j <= N; j++) {
            if (i >= j) {
                res(i, j) = bernstein(a, i, j);
            }
        }
    }
    return res;
}

// Generate the continuity for the left hand side so u->0
// N: Degree of the b-spline
// C: to what continuity condition 0: endpoints, 1: first derivative ....
template<int N, int C>
constexpr auto CRHS(const float h) {
    blaze::StaticMatrix<float, C + 1, N + 1> res(0);
    static_assert(C <= 2); // If the continuity is greater than 3 dont compile
    switch (C)
    {
    case 2:
        res(2, N) = 1 / (h * h);
        res(2, N - 1) = -2 / (h * h);
        res(2, N - 2) = 1 / (h * h);
    case 1:
        res(1, N) = 1 / h;
        res(1, N - 1) = -1 / h;
    case 0:
        res(0, N) = 1;
        break;
    default:
        break;
    }
    return res;
}

// Generate the continuity for the left hand side so u->0
// N: Degree of the b-spline
// C: to what continuity condition 0: endpoints, 1: first derivative ....
template<int N, int C>
constexpr auto CLHS(const float h) {
    blaze::StaticMatrix<float, C + 1, N + 1> res(0);
    static_assert(C <= 2); // If the continuity is greater than 3 dont compile
    switch (C)
    {
    case 2:
        res(2, 0) = 1 / (h * h);
        res(2, 1) = -2 / (h * h);
        res(2, 2) = 1 / (h * h);
    case 1:
        res(1, 0) = 1 / h;
        res(1, 1) = -1 / h;
    case 0:
        res(0, 0) = 1;
        break;
    default:
        break;
    }
    return res;
}
template<int N, Axis ax>
static inline SparseMat STgen(std::pair<float, float> bounds, std::pair<float, float> twin_bounds)
{
    if (((twin_bounds.first - bounds.first) > eps) && ((bounds.second - twin_bounds.second) > eps)) {
        // both need to be split
        const float a0 = (twin_bounds.second - bounds.first) / (bounds.second - bounds.first);
        const float a1 = (twin_bounds.first - bounds.first) / (bounds.second - bounds.first);
        return SRHS<N>(a1) * SLHS<N>(a0);
    }
    else if ((twin_bounds.first - bounds.first) > eps) {
        // Only one split
        const float a1 = (twin_bounds.first - bounds.first) / (bounds.second - bounds.first);
        return SRHS<N>(a1);
    }
    else if ((twin_bounds.first - bounds.first) > eps) {
        const float a0 = (twin_bounds.second - bounds.first) / (bounds.second - bounds.first);
        return SLHS<N>(a0);
    }
    else {
        return blaze::IdentityMatrix<float>(N + 1);
    }
}


// Store the constraints that correspond to a pair of halfFaces
template<int N, int C>
struct Face
{
    blaze::CompressedMatrix<float> higherConstraint;
    blaze::CompressedMatrix<float> lowerConstraint;
};

inline float getDepth(std::pair<Vertex, Vertex> corners, Axis ax) {
    const auto depth = corners.first - corners.second;
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
static inline SparseMat genRMatrix(const Mesh& mesh, HalfFacePair pair, bool needsSplit)
{
    const auto first_corners = getCorners(mesh, pair.first);
    const auto second_corners = getCorners(mesh, pair.second);
    const float h_right = getDepth(second_corners, ax) / N;
    if (!needsSplit) {
        if constexpr (ax == Axis::x) {
            return blaze::kron(blaze::IdentityMatrix<float>((N + 1) * (N + 1)), CRHS<N,C>(h_right));
        }
        else if (ax == Axis::y) {
            return blaze::kron(blaze::IdentityMatrix<float>(N + 1), blaze::kron(CRHS<N,C>(h_right), blaze::IdentityMatrix<float>(N + 1)));
        }
        else {
            return blaze::kron(CRHS<N,C>(h_right), blaze::IdentityMatrix<float>((N + 1) * (N + 1)));
        }
    }
    const auto X = [&]() -> SparseMat { if constexpr (ax == Axis::x) {
        return CRHS<N, C>(h_right);
    }
    return STgen<N, ax>({ second_corners.first.x, second_corners.second.x }, { first_corners.first.x, first_corners.second.x });
    }();
    const auto Y = [&]() -> SparseMat { if constexpr (ax == Axis::y) {
        return CRHS<N,C>(h_right);
    }
    return STgen<N, ax>({ second_corners.first.y, second_corners.second.y }, { first_corners.first.y, first_corners.second.y });
    }();
    const auto Z = [&]() -> SparseMat { if constexpr (ax == Axis::z) {
        return CRHS<N, C>(h_right);
    }
    return STgen<N, ax>({ second_corners.first.z, second_corners.second.z }, { first_corners.first.z, first_corners.second.z });
    }();
    return blaze::kron(Z, blaze::kron(Y, X));
}

template<int N, int C, Axis ax>
static inline SparseMat genLMatrix(const Mesh& mesh, HalfFacePair pair, bool needsSplit)
{
    const auto first_corners = getCorners(mesh, pair.first);
    const auto second_corners = getCorners(mesh, pair.second);
    const float h_left = getDepth(first_corners, ax) / N;
    if (!needsSplit) {
        if constexpr (ax == Axis::x) {
            return blaze::kron(blaze::IdentityMatrix<float>((N + 1) * (N + 1)), CLHS<N,C>(h_left));
        }
        else if (ax == Axis::y) {
            return blaze::kron(blaze::IdentityMatrix<float>(N + 1), blaze::kron(CLHS<N,C>(h_left), blaze::IdentityMatrix<float>(N + 1)));
        }
        else {
            return blaze::kron(CLHS<N,C>(h_left), blaze::IdentityMatrix<float>((N + 1) * (N + 1)));
        }
    }
    const auto X = [&]() -> SparseMat { if constexpr (ax == Axis::x) {
        return CLHS<N, C>(h_left);
    }
    return STgen<N, ax>({first_corners.first.x, first_corners.second.x}, { second_corners.first.x, second_corners.second.x });
    }();
    const auto Y = [&]() -> SparseMat { if constexpr (ax == Axis::y) {
        return CLHS<N, C>(h_left);
    }
    return STgen<N, ax>({ first_corners.first.y, first_corners.second.y }, { second_corners.first.y, second_corners.second.y });
    }();
    const auto Z = [&]() -> SparseMat { if constexpr (ax == Axis::z) {
        return CLHS<N, C>(h_left);
    }
    return STgen<N, ax>({ first_corners.first.z, first_corners.second.z }, { second_corners.first.z, second_corners.second.z });
    }();
    return blaze::kron(Z, blaze::kron(Y, X));
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
public:
    static const uint64_t toKey(HalfFacePair pair) { return static_cast<uint64_t>(pair.first.id) + (static_cast<uint64_t>(pair.second.id) << 32); }
    static const HalfFacePair fromKey(uint64_t key) { return { halfFace(static_cast<uint32_t>(key)), halfFace(static_cast<uint32_t>(key >> 32)) }; }
    blaze::CompressedMatrix<float> generateGlobalMatrix() const;
    void regenerateConstraints();
    robin_hood::unordered_map<uint64_t, Face<N,C>> constraints;
    SplineMesh(Mesh&& mesh) { this->mesh = mesh; regenerateConstraints(); constraintsValid = true; }
    SplineMesh() = default;
    ~SplineMesh() = default;
};

template<int N, int C>
inline blaze::CompressedMatrix<float> SplineMesh<N,C>::generateGlobalMatrix() const
{
    const size_t numConstraints = constraints.size();
    const auto subMatSize_N = (N + 1) * (N + 1) * (N + 1);
    const auto subMatSize_M = (N + 1) * (N + 1) * (C + 1);
    blaze::CompressedMatrix<float> global(numConstraints * subMatSize_M, numElements * subMatSize_N);
    // Iterate over all the constraints
    size_t i = 0;
    for (const auto& [key, Face] : constraints)
    {
        auto pair = fromKey(key);
        // Insert the constraints 
        auto sm = blaze::submatrix(global, /* row */ subMatSize_M * i, /* col */ subMatSize_N * pair.first.getCuboid(), subMatSize_M, subMatSize_N);
        sm = Face.lowerConstraint;
        sm = blaze::submatrix(global, /* row */ subMatSize_M * i, /* col */ subMatSize_N * pair.second.getCuboid(), subMatSize_M, subMatSize_N);
        sm = Face.higherConstraint;
        i++;
    }
    return global;
}

template<int N, int C>
void SplineMesh<N,C>::regenerateConstraints() {
    numElements = mesh.getCuboids().size();
    for (size_t cub = 0; cub < mesh.getCuboids().size(); ++cub)
    {
        for (size_t hf = 0; hf < 6; hf++)
        {
            // Skip all the halffaces on the higher axis
            if (hf == 1 || hf == 2 || hf == 3) continue;
            const auto cur_hf = halfFace(cub, hf);
            const auto twin = mesh.Twin(cur_hf);
            if (twin.isBorder()) continue;
            if (twin.isSubdivided()) {
                for (auto it = mesh.getSft().cbegin(twin); it != mesh.getSft().cend(); ++it)
                {
                    const auto pair = HalfFacePair{ cur_hf, *it };
                    const auto key = toKey(pair);
                    if (constraints.find(key) == constraints.end()) {
                        if(hf == 0) {
                            constraints.insert({ key, Face<N,C>{genLMatrix<N,C,Axis::z>(mesh, pair, true), genRMatrix<N,C,Axis::z>(mesh, pair, mesh.Twin(*it) == cur_hf)} });
                        }
                        else if (hf == 4) {
                            constraints.insert({ key, Face<N,C>{ genLMatrix<N,C,Axis::x>(mesh, pair, true), genRMatrix<N,C,Axis::x>(mesh, pair, mesh.Twin(*it) == cur_hf) } });
                        }
                        else {
                            constraints.insert({ key, Face<N,C>{ genLMatrix<N,C,Axis::y>(mesh, pair, true), genRMatrix<N,C,Axis::y>(mesh, pair, mesh.Twin(*it) == cur_hf) } });
                        }
                        
                    }
                }
            }
            else {
                const auto pair = HalfFacePair{ cur_hf, twin };
                const auto key = toKey(pair);
                if (constraints.find(key) == constraints.end()) {
                    if (hf == 0) {
                        constraints.insert({ key, Face<N,C>{ genLMatrix<N,C,Axis::z>(mesh, pair, false), genRMatrix<N,C,Axis::z>(mesh, pair, false) } });
                    }
                    else if (hf == 4) {
                        constraints.insert({ key, Face<N,C>{ genLMatrix<N,C,Axis::x>(mesh, pair, false), genRMatrix<N,C,Axis::x>(mesh, pair, false) } });
                    }
                    else {
                        constraints.insert({ key, Face<N,C>{ genLMatrix<N,C,Axis::y>(mesh, pair, false), genRMatrix<N,C,Axis::y>(mesh, pair, false) } });
                    }
                }
            }
        }
    }
};
