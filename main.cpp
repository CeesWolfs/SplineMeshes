#include "Mesh.hpp"
#include "Splines.hpp"

int main(int argc, char const *argv[])
{
	Mesh mesh;
	mesh.SplitAlongXY(0, 0.5);
	//auto top = mesh.SplitAlongXY(1, 0.75);
	mesh.SplitAlongXZ(0, 0.5);
	//mesh.SplitAlongXZ(1, 0.4);
	//mesh.SplitAlongYZ(0, 0.6);
	//mesh.SplitAlongYZ(top, 0.7);
	SplineMesh<2, 1> splines(std::move(mesh));
	auto System = splines.generateGlobalMatrix();
	std::cout << System.cols() << ' ' << System.rows() << '\n';
	std::cout << System << '\n';
	//std::cout << System.toDense() << '\n';
	auto QR = Eigen::FullPivHouseholderQR<Eigen::MatrixXf>(System.transpose());
	auto Q = QR.matrixQ();
	auto kernel = Q.block(0, QR.rank(), Q.rows(), Q.cols() - QR.rank());
	std::cout << kernel.cols() << ' ' << kernel.rows() << '\n';
	//std::cout << kernel;
	return 0;
}