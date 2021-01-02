#include "Mesh.hpp"
#include "Splines.hpp"

int main(int argc, char const *argv[])
{
	Mesh mesh;
	mesh.SplitAlongXY(0, 0.5); 
	mesh.SplitAlongXZ(0, 0.4); 
	mesh.SplitAlongXZ(1, 0.3);
	uint32_t foo = mesh.SplitAlongYZ(1, 0.7);
	mesh.SplitAlongXZ(foo, 0.1);
	mesh.SplitAlongYZ(0, 0.2);
	mesh.Save("SplineTest");
	SplineMesh<3, 2> splines(std::move(mesh));
	auto System = splines.generateGlobalMatrix();
	std::cout << System.cols() << ' ' << System.rows() << '\n';
	Eigen::SparseMatrix<float> Q;
	auto QR = Eigen::SparseQR<Eigen::SparseMatrix<float>, Eigen::COLAMDOrdering<int>>(System.transpose());
	Q = QR.matrixQ();
	auto kernel = Q.block(0, QR.rank(), Q.rows(), Q.cols() - QR.rank());
	std::cout << Q.cols() << ' ' << Q.rows() << ' ' << QR.rank() << '\n';
	return 0;
}