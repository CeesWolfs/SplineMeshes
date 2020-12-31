#include "Mesh.hpp"
#include "Splines.hpp"

int main(int argc, char const *argv[])
{
	Mesh mesh(2, 2, 2);
	mesh.SplitAlongXY(0, 0.2);
	SplineMesh<3, 2> splines(std::move(mesh));
	auto System = splines.generateGlobalMatrix();
	return 0;
}