import open3d as o3d
import numpy as np
mesh=o3d.io.read_triangle_mesh("/home/finibo/Downloads/calib_cube.stl")

print(mesh)
print('Vertices:')
print(np.asarray(mesh.vertices))
print('Triangles:')
print(np.asarray(mesh.triangles))


mesh.compute_vertex_normals()
o3d.visualization.draw_geometries([mesh])
pcd = mesh.sample_points_uniformly(number_of_points=500)
o3d.visualization.draw_geometries([pcd])






# pcd = o3d.geometry.PointCloud()
# # mesh.compute_vertex_normals()
#
# # o3d.visualization.draw_geometries([mesh])
# print dir(mesh)
# pcd = mesh.sample_points_uniformly(number_of_points=500)
#
# o3d.visualization.draw_geometries([pcd])