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

#define N 5
#define C 2

int main(int argc, char const *argv[])
{
	Mesh mesh(1, 2, 2);
	//mesh.SplitAlongYZ(0, 0.3);
	/*uint32_t newelem  = mesh.SplitAlongXY(0, 0.25);
	uint32_t newelem1 = mesh.SplitAlongYZ(0, 0.25);
	uint32_t newelem2 = mesh.SplitAlongYZ(newelem, 0.25);
	mesh.SplitAlongXZ(0, 0.25);
	mesh.SplitAlongXZ(newelem, 0.25);
	mesh.SplitAlongXZ(newelem1, 0.25);
	mesh.SplitAlongXZ(newelem2, 0.25);*/
	//mesh.Save("dimtest");
	SplineMesh<N, C> splines(std::move(mesh));
	Eigen::VectorXf controlpoints(splines.numControlPoints());
	controlpoints.setZero();
	mesh.Save("localtest");
	//uint32_t top = splines.SplitAlongXY(0,0.5);
	//mesh.SplitAlongXZ(0, 0.5);
	//mesh.SplitAlongXY(1, 0.5);
	//mesh.SplitAlongYZ(top, 0.25);
	//mesh.Save("SplineTest");
	
	auto System = splines.generateGlobalMatrix();
	std::cout << System.cols() << ' ' << System.rows() << '\n';
	auto QR = Eigen::FullPivHouseholderQR<Eigen::MatrixXf>(System.transpose());
	auto Q = QR.matrixQ();
	auto kernel = Q.block(0, QR.rank(), Q.rows(), Q.cols() - QR.rank());
	std::cout << kernel.cols() << ' ' << kernel.rows() << '\n';
	//std::cout << kernel;
	std::cout << CalcSplineDegree(splines.get_mesh(), N, C, splines.constraints.size()) << '\n';
	//
	QuantitiesOfInterest q(splines.get_mesh());
	int sum{ 0 };
	for (uint32_t v = 0; v < splines.get_mesh().getVertices().size(); v++) {
		if (q.isPVertex(v)) {
			auto localNullspace = splines.LocalNullspace<3>(v);
			if (localNullspace.coefficients.size() == 0) continue;
			//std::cout << localNullspace.kernel << '\n';
			sum += localNullspace.kernel.cols();

			std::for_each(localNullspace.elements.begin(), localNullspace.elements.begin()+localNullspace.num, [&, element = 0](uint32_t elem) mutable {
				for (size_t i = 0; i < localNullspace.numControlPointsElement(); ++i)
				{
					auto coeff = elem * splines.numControlPointsElement() + localNullspace.coefficients[element * localNullspace.numControlPointsElement() + i];
					controlpoints[coeff] += localNullspace.kernel.col(0)[element * localNullspace.numControlPointsElement() + i];
				}
				element++;
				});
		}
	}
	splines.renderBasis("Localtest.vtu", controlpoints, 20);
	std::cout << sum << '\n';
	return 0;
}