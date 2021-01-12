#include "Mesh.hpp"
#include "lean_vtk.hpp"
#include "SplineMesh.hpp"

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
	auto QR = Eigen::FullPivHouseholderQR<Eigen::MatrixXf>(System.transpose());
	auto Q = QR.matrixQ();
	auto kernel = Q.block(0, QR.rank(), Q.rows(), Q.cols() - QR.rank());
	std::cout << kernel.cols() << ' ' << kernel.rows() << '\n';
	//std::cout << kernel << '\n';
	std::vector<double> points;
	std::vector<double> spline_val;
	// Render a spline basis
	for (int cube = 0; cube < splines.get_mesh().getCuboids().size(); cube++) {
		const auto bl_corner = mesh.getVertices()[mesh.getCuboids()[cube].v1];
		const auto tr_corner = mesh.getVertices()[mesh.getCuboids()[cube].v7];
		const auto depth = tr_corner - bl_corner;
		// Render 1000 points per cuboid
		for (size_t z_i = 0; z_i <= 10; z_i++)
		{
			float z = z_i * (depth.z / 10) + bl_corner.z;
			for (size_t y_i = 0; y_i <= 10; y_i++)
			{
				float y = y_i * (depth.y / 10) + bl_corner.y;
				for (size_t x_i = 0; x_i <= 10; x_i++)
				{
					float x = x_i * (depth.x / 10) + bl_corner.x;
					spline_val.emplace_back(VolumeSpline<2, float>(x_i/10, y_i/10, z_i/10, kernel.block(cube * 3 * 3 * 3, 0, 3 * 3 * 3, 1)));
					points.push_back(x);
					points.push_back(y);
					points.push_back(z);
				}
			}
		}
	}
	leanvtk::VTUWriter writer;
	writer.add_scalar_field("Spline values", spline_val);
	writer.write_point_cloud("Spline_test.vtu", 3, points);
	return 0;
}