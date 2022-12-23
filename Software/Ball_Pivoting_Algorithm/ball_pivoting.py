'''
* *****************************************************************************
* @author	ChangMin An
* @Mod		2022 - 12 - 20
* @brief	Ball Pivoting Algorithm using Point Cloud Data (PLY)
* @Version	Open3D 0.15.1 
* *****************************************************************************
'''

#===============================================#
#              Open Library Declare             #
#===============================================#
import open3d as o3d

print("setup")


#===============================================#
#                     Main                      #
#===============================================#

# Input Point Cloud Data Name
dataname="powerplant1205_010.ply"

# Output Point Cloud Data Name
output_name="powerplant1220_010_ball_pivoting.ply"

# Read Point Cloud Data
pcd = o3d.io.read_point_cloud(dataname)

# Normalize
print("Normalize")
pcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

# Ball Pivoting
print('Ball Pivoting')
radii = [0.005, 0.01, 0.02, 0.04]
rec_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
    pcd, o3d.utility.DoubleVector(radii))

# Write Result
print("Write Result")
o3d.io.write_triangle_mesh(output_name, rec_mesh)