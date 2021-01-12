import sys
import pyvista as pv
if len(sys.argv) < 2:
	print("Usage: python visualize.py filename.ply")
	sys.exit(1)
mesh = pv.read(sys.argv[1])
p = pv.Plotter()
p.add_mesh(mesh, show_edges=True, color=None, opacity=0.1, line_width=5)
poly = pv.PolyData(mesh.points)
poly["My Labels"] = [f"{i}" for i in range(poly.n_points)]
p.add_mesh(poly, color='red',
       point_size=10, render_points_as_spheres=True)
#p.add_mesh(mesh.extract_cells([2*6, 28*6, 1*6, 0]),
#           color='pink', edge_color='blue',
#           line_width=5, show_edges=True)
p.add_point_labels(poly, "My Labels", font_size=30)
p.show_axes()
p.show()
