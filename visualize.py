import sys
import pyvista as pv
if len(sys.argv) < 2:
	print("Usage python visualize.py filename.ply")
	sys.exit(1)
mesh = pv.read(sys.argv[1])
p = pv.Plotter()
p.add_mesh(mesh, show_edges=True, color='white')
p.add_mesh(mesh.extract_cells(0),
           color='pink', edge_color='blue',
           line_width=5, show_edges=True)
p.add_mesh(pv.PolyData(mesh.points), color='red',
       point_size=10, render_points_as_spheres=True)
p.show()
