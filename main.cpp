#include "Mesh.hpp"
#include "QuantitiesOfInterest.hpp"
#include "SplineMesh.hpp"

#define samples 9

int CalcSplineDegree(const Mesh& mesh, int m, int alpha, int numConstraints) {
	assert(m >= (2 * alpha + 1));
	int C = mesh.getCuboids().size();
	QuantitiesOfInterest q(mesh);
	//auto edges = q.getAllEdges();
	//int E = std::count_if(edges.begin(), edges.end(), [&](auto edge) {return !q.isBorderEdge(edge); });
	int F = numConstraints;//q.interiorFaces();
	int V = 0;
	for (size_t v = 0; v < mesh.getVertices().size(); v++)
	{
		if (!q.isBorderVertex(v) && !q.isEVertex(v)) V++;
	}
	return (m + 1) * (m + 1) * (m + 1) + (C - 1) * (m + 1) * (m - alpha) * (m + alpha + 2) - F * (m + 1) * (alpha + 1) * (m - alpha) + V * (alpha + 1) * (alpha + 1) * (m - alpha);
 }

#define N 3

int main(int argc, char const *argv[])
{
	Mesh mesh(2,2,2);
	//uint32_t top = mesh.SplitAlongXY(0,0.25);
	//mesh.SplitAlongXZ(0, 0.25);
	//mesh.SplitAlongXY(1, 0.5);
	//mesh.SplitAlongYZ(top, 0.25);
	mesh.Save("SplineTest");
	SplineMesh<N, 1, 2, 0> splines(std::move(mesh));
	auto System = splines.generateGlobalMatrix();
	std::cout << System.cols() << ' ' << System.rows() << '\n';
	auto QR = Eigen::FullPivHouseholderQR<Eigen::MatrixXf>(System.transpose());
	auto Q = QR.matrixQ();
	auto kernel = Q.block(0, QR.rank(), Q.rows(), Q.cols() - QR.rank());
	std::cout << kernel.cols() << ' ' << kernel.rows() << '\n';
	//std::cout << kernel;
	std::cout << CalcSplineDegree(splines.get_mesh(), 3, 1, splines.constraints.size()) << '\n';
	splines.renderBasis("Spline.vtu", Eigen::VectorXf(kernel.col(0) + kernel.col(4) + kernel.col(17) + kernel.col(82) - kernel.col(43)), 10);
	QuantitiesOfInterest q(splines.get_mesh());
	for (uint32_t v = 0; v < splines.get_mesh().getVertices().size(); v++) {
		if (!q.isBorderVertex(v)) std::cout << splines.LocalNullspace<1>(v);
	}
	return 0;
}