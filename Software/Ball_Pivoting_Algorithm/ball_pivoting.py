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

# Input Point Cloud Data Path
input_path="D:\MIP\발표자료\Final_Image/"

# Output Point Cloud Data Path
output_path="D:\MIP\발표자료\Final_Image/"

# Input Point Cloud Data Name
dataname="powerplant1205_005.ply"

# Output Point Cloud Data Name
output_name="powerplant1220_005_ball_pivoting.ply"

# Read Point Cloud Data
pcd = o3d.io.read_point_cloud(input_path+dataname)

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
o3d.io.write_triangle_mesh(output_path+output_name, rec_mesh)