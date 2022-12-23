# Generation of LiDAR Dataset for Human Detection in Virtual Smart Factory Environment

<img src="images/blensor3.jpg" style="zoom:40%;" />

<img src="images/tie.gif" style="zoom:40%;" />



## I. Introduction

**Goal: Generation of LiDAR Dataset for Human Detection**

This research is LiDAR dataset generation through sensor simulation in a smart factory virtual environment. It is a program that generates LiDAR point cloud dataset using human detection model.

This project was carried out as a 2022-2 MIP study with advisor Ph.D Young-Keun Kim.



## II. Requirement

### Hardware

* Intel RealSense L515

<img src="images/L5152.jpg" style="zoom:20%;" />

### Software

* Python 3.9.12
* Open3D 0.15.1
* Visual Studio Code
* Dot3D Product
* MeshLab 2022.02
* Blensor (Blender Sensor Simulation Toolbox)
* MATLAB 2022a LiDAR Labeler



## III. Flow Chart

<img src="images/Flow_Chart.jpg" style="zoom:90%;" />



## IV. Tutorial

### 1. Installation

#### 1-1. Anaconda & Python 3.9.12

[Installation Guide](https://ykkim.gitbook.io/dlip/installation-guide/installation-guide-for-deep-learning)

Proceed to **number 4** in the above guidance, and then proceed with the following steps.



#### 1-2. install library to use Ball Pivoting Algorithm

<img src="images/anaconda.jpg" style="zoom:90%;" />

##### 1-1-1. Conda & Pip Update

```python
conda update -n base conda
conda activate py39
python -m pip install --upgrade pip
```

**1-1-2. Install Numpy & Open3D Library**

```python
conda activate py39
pip install numpy
pip install open3d=0.15.1
```



#### 1-3. install MATLAB R2022a & LiDAR Labeler

Add-On -> Computer Vision Toolbox, Image Processing Toolbox, LiDAR Toolbox Install

|                   Computer Vision Toolbox                    |                   Image Processing Toolbox                   |                      LiDAR Toolbox                       |
| :----------------------------------------------------------: | :----------------------------------------------------------: | :------------------------------------------------------: |
| <img src="images/Computer_Vision_Toolbox.jpg" style="zoom:90%;" /> | <img src="images/Image_Process_Toolbox.jpg" style="zoom:90%;" /> | <img src="images/LiDAR_Toolbox.jpg" style="zoom:90%;" /> |



### 2. Download

* Download MeshLab: [click here to download](https://www.meshlab.net/#download)
* Download Blensor:    [click here to download](https://drive.google.com/drive/folders/1sqgJ-0eLij1EOEoGhmoO7qEQzztfGDk6?usp=share_link)
* Download Dot3D:      [click here to download](https://www.dotproduct3d.com/dot3ddownload.html)

* Download Data:         [click here to download](https://drive.google.com/drive/folders/1sqgJ-0eLij1EOEoGhmoO7qEQzztfGDk6?usp=share_link)



### 3. Configuration

#### 3-1. MeshLab GPU Memory

It secures FPS through the memory use of GPU. To this end, the memory limit is set according to the specifications of each laptop or PC.

|                            (1)                             |                            (2)                             |
| :--------------------------------------------------------: | :--------------------------------------------------------: |
| <img src="images/Meshlab_Config1.jpg" style="zoom:90%;" /> | <img src="images/Meshlab_Config2.jpg" style="zoom:90%;" /> |



### 4. Factory 3D Modeling

#### 4-1. 3D Point Cloud Data Acquirement using Dot3D

**In order to obtain point cloud data, a Dot3D Pro version or a Trial version is required.**

[Dot3D Trial Webpage](https://dot3dapp.com/licensing/buyDot3d.php)

<img src="images/dot3d_trial.jpg" style="zoom:90%;" />

(0) Connect the LiDAR L515 to the computer

(1) Launch the Dot3D software and press 'New scan'

(2) Press the right button to obtain data

|                       (1)                        |                        (2)                        |
| :----------------------------------------------: | :-----------------------------------------------: |
| <img src="images/dot3d.jpg" style="zoom:90%;" /> | <img src="images/dot3d2.jpg" style="zoom:90%;" /> |



(3) If the data is acquired up to the desired part, press the right button again to end

(4)The optimization process provided by Dot3D

|                        (3)                        |                        (4)                        |
| :-----------------------------------------------: | :-----------------------------------------------: |
| <img src="images/dot3d3.jpg" style="zoom:90%;" /> | <img src="images/dot3d4.jpg" style="zoom:90%;" /> |



(5, 6) With less than 5 million points, point cloud data is stored with a PLY format

|                        (5)                        |
| :-----------------------------------------------: |
| <img src="images/dot3d5.jpg" style="zoom:90%;" /> |
|                      **(6)**                      |
| <img src="images/dot3d6.jpg" style="zoom:90%;" /> |



#### 4-2. 3D Modeling using Ball Pivoting Algorithm

**Run through the `ball_pivoting.py` file in Appendix.**



##### 4-2-1. Configure point cloud data path & read point cloud data

```python
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
```



##### 4-2-2. Normalize point cloud data

```python
# Normalize
print("Normalize")
pcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
```



##### 4-2-3. Run point to face using Ball pivoting algorithm

```python
# Ball Pivoting
print('Ball Pivoting')
radii = [0.005, 0.01, 0.02, 0.04]
rec_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
    pcd, o3d.utility.DoubleVector(radii))
```



##### 4-2-4. Export result (PLY format)

```python
# Write Result
print("Write Result")
o3d.io.write_triangle_mesh(output_path+output_name, rec_mesh)
```



#### 4-3. Laplacian Smoothing Filter using MeshLab

**Remove noise using Laplacian smoothing filter**

(0) Run MeshLab 2022.02

(1) Import the result of the point cloud data created in **4-2**

(2) Run Laplacian smoothing filter (Filters -> Smoothing, Fairing and Deformation -> Laplacian Smooth)

(3) Set the number of filter repetitions(Smoothing steps) and apply. The more the number of times, the more smooth effects, but distortion occurs.

|                         (1)                         |                         (2)                         |
| :-------------------------------------------------: | :-------------------------------------------------: |
| <img src="images/Meshlab1.jpg" style="zoom:90%;" /> | <img src="images/Meshlab2.jpg" style="zoom:90%;" /> |

|                         (3)                         |
| :-------------------------------------------------: |
| <img src="images/Meshlab3.jpg" style="zoom:90%;" /> |



#### 4-4. Export Factory 3D Modeling Data

(1, 2) Export factory modeling result (PLY format)

|                         (1)                         |                         (2)                         |
| :-------------------------------------------------: | :-------------------------------------------------: |
| <img src="images/Meshlab4.jpg" style="zoom:90%;" /> | <img src="images/Meshlab6.jpg" style="zoom:50%;" /> |



#### **Result**

|                       Raw Data                        |                 Ball Pivoting Algorithm                  |                       BPA + Smoothing                        |
| :---------------------------------------------------: | :------------------------------------------------------: | :----------------------------------------------------------: |
| <img src="images/pointcloud.jpg" style="zoom:90%;" /> | <img src="images/ball_pivoting.jpg" style="zoom:90%;" /> | <img src="images/Laplacian_Smoothing.jpg" style="zoom:90%;" /> |



### 5. Human 3D Modeling

#### 5-1. 3D Point Cloud Data Acquirement using Dot3D

It measures human data in the same way as the **4-1** process. After the experimenter fixes it in a specific position, it rotates 360 degrees to acquire data.

**[Demo Video](https://youtu.be/CIqr9B9jKh0)** 



#### 5-2. Normalize Point Cloud Data (MeshLab)

(0) Run MeshLab 2022.02

(1) Import the result of the point cloud data created in **5-1**

(2) Normalize point cloud data (Filters -> Normals, Curvatures and Orientation -> Compute normals for point sets)

(3) Normalize while adjusting the neighbor number. (If the color of the data becomes yellow, it is properly normalized)

|                         (1)                         |                        (2)                        |
| :-------------------------------------------------: | :-----------------------------------------------: |
| <img src="images/Meshlab1.jpg" style="zoom:90%;" /> | <img src="images/Human1.jpg" style="zoom:80%;" /> |

|                        (3)                        |
| :-----------------------------------------------: |
| <img src="images/Human2.jpg" style="zoom:90%;" /> |



#### 5-3. 3D Modeling using Poisson Surface Reconstruction (MeshLab)

(1) Make 3D human model using Poisson Surface Reconstruction. (Filters -> Remeshing, Simplication and Reconstruction -> Surface Reconstruction: Screened Poisson)

(2) As the reconstruction depth value increases, the depth is expressed detail.

|                           Depth 6                           |                           Depth 8                           |                          Depth 10                           |
| :---------------------------------------------------------: | :---------------------------------------------------------: | :---------------------------------------------------------: |
| <img src="images/Reconstruction1.jpg" style="zoom:100%;" /> | <img src="images/Reconstruction2.jpg" style="zoom:100%;" /> | <img src="images/Reconstruction3.jpg" style="zoom:100%;" /> |

|                        (1)                         |                        (2)                        |
| :------------------------------------------------: | :-----------------------------------------------: |
| <img src="images/Human3.jpg" style="zoom:110%;" /> | <img src="images/Human4.jpg" style="zoom:90%;" /> |



#### 5-4. Laplacian Smoothing Filter using MeshLab

(1) Run Laplacian smoothing filter (Filters -> Smoothing, Fairing and Deformation -> Laplacian Smooth)

(2) Set the number of filter repetitions(Smoothing steps) and apply. The more the number of times, the more smooth effects, but distortion occurs.

|                         (1)                          |                         (2)                         |
| :--------------------------------------------------: | :-------------------------------------------------: |
| <img src="images/Meshlab2.jpg" style="zoom:100%;" /> | <img src="images/Meshlab3.jpg" style="zoom:90%;" /> |



#### 5-5. Export Human 3D Modeling Data

(1, 2) Export factory modeling result (PLY format)

|                         (1)                         |                         (2)                         |
| :-------------------------------------------------: | :-------------------------------------------------: |
| <img src="images/Meshlab4.jpg" style="zoom:90%;" /> | <img src="images/Meshlab6.jpg" style="zoom:50%;" /> |

#### **Result**

|                       Raw Data                       |                     Normalized Data                     |
| :--------------------------------------------------: | :-----------------------------------------------------: |
| <img src="images/Human_Raw.jpg" style="zoom:50%;" /> | <img src="images/Human_Normal.jpg" style="zoom:50%;" /> |

|                Poisson Surface Reconstruction                |          Poisson Surface Reconstruction + Filter           |
| :----------------------------------------------------------: | :--------------------------------------------------------: |
| <img src="images/Poisson_Reconstruction.jpg" style="zoom:50%;" /> | <img src="images/Human_Smoothing.jpg" style="zoom:50%;" /> |



### 6. Virtual Environment LiDAR Simulation (Blensor)

#### 6-1. Configuration about LiDAR Sensor

We can set the LiDAR specification in the Blensor. Currently, it is set as a LiDAR that matches `HESAI's Pandar XT32`.

<img src="images/Blensor_lidar1.jpg" style="zoom:90%;" />

<img src="images/Blensor_lidar2.jpg" style="zoom:90%;" />



#### 6-2. Open Blensor & Project File

(1) Run the Blensor software (Blensor -> Blensor-1.0.18-Blender-2.79-Winx64 -> blender.exe)

(2, 3) Open `Final_Simulation.blend` file.

|                          (1)                           |                          (2)                           |
| :----------------------------------------------------: | :----------------------------------------------------: |
| <img src="images/simulation1.jpg" style="zoom:50%;" /> | <img src="images/simulation2.jpg" style="zoom:90%;" /> |

|                          (3)                           |
| :----------------------------------------------------: |
| <img src="images/simulation3.jpg" style="zoom:90%;" /> |



#### 6-3. Import Factory 3D Model & Human 3D Model

(1) Import the previously created human model and factory-like model. (File->Import->.ply)

|                          (1)                           |
| :----------------------------------------------------: |
| <img src="images/simulation4.jpg" style="zoom:90%;" /> |



#### 6-4. Place 3D Models like factory

(1) Place 3D facroty-like model and human model. (Location, Rotation, Scale) **Scale must be 1.0.**

|                           (1)                           |                         (2)                         |
| :-----------------------------------------------------: | :-------------------------------------------------: |
| <img src="images/simulation8.jpg" style="zoom:200%;" /> | <img src="images/blensor1.jpg" style="zoom:90%;" /> |



#### 6-5. Place LiDAR Sensor

(1) Camera in Blensor is a virtual LiDAR sensor. Click Camera and place the sensor in the desired location. 

|                          (1)                           |
| :----------------------------------------------------: |
| <img src="images/simulation5.jpg" style="zoom:80%;" /> |



#### 6-6. LiDAR Simulation

(1) Click on the camera and set the sensor simulation as follows. You must check `Save to File` in order to export LiDAR data. Click `Single scan`.

|                           (1)                           |
| :-----------------------------------------------------: |
| <img src="images/simulation6.jpg" style="zoom:120%;" /> |



#### 6-7. Export Point Cloud Data(PCD format)

(1) Make sure to set the file `.pcd` before running.

|                          (1)                           |
| :----------------------------------------------------: |
| <img src="images/simulation7.jpg" style="zoom:80%;" /> |

#### **result**

<img src="images/simulation9.jpg" style="zoom:80%;" />

<img src="images/blensor2.jpg" style="zoom:100%;" />



### 7. Point Cloud Data Labeling using MATLAB LiDAR Labeler

**The labeling work is carried out using the data obtained through the Blensor.**

#### 7-1. Run LiDAR Labeler

(1) MATLAB 2022a -> APPs -> Lidar Labeler

(2, 3, 4) Add point cloud `PCD format`

|                           (1)                            |                         (2)                          |
| :------------------------------------------------------: | :--------------------------------------------------: |
| <img src="images/LiDAR_Labeler.jpg" style="zoom:80%;" /> | <img src="images/Labeler1.jpg" style="zoom:150%;" /> |

|                         (3)                          |                         (4)                          |
| :--------------------------------------------------: | :--------------------------------------------------: |
| <img src="images/Labeler2.jpg" style="zoom:150%;" /> | <img src="images/Labeler3.jpg" style="zoom:100%;" /> |



#### 7-2. Configuration Label name

(1) ROI label -> Click Label

(2) Label Name is `Human` forms of `Cuboid`

|                         (1)                          |                         (2)                          |
| :--------------------------------------------------: | :--------------------------------------------------: |
| <img src="images/Labeler4.jpg" style="zoom:150%;" /> | <img src="images/Labeler5.jpg" style="zoom:150%;" /> |



#### 7-3. Snap to Cluster about point cloud data

(1) Click `Snap to Cluster`

(2, 3) Click `Cluster Settings` -> Distance-based clustering(=거리 기반 군집화) -> Configuration `Min Distance` is **0.1~0.4**

|                         (1)                          |                         (2)                          |                         (3)                          |
| :--------------------------------------------------: | :--------------------------------------------------: | :--------------------------------------------------: |
| <img src="images/Labeler6.jpg" style="zoom:150%;" /> | <img src="images/Labeler7.jpg" style="zoom:150%;" /> | <img src="images/Labeler8.jpg" style="zoom:150%;" /> |



#### 7-3. Labeling point cloud data & Export result

(1) Labeling through clicked data & using Projected View

(2) If you have finished labeling the current frame, click next frame

(3) If you want to save during labeling, click `Save Session` to save

(4) Export labeling result (Export -> To file)

|                         (1)                          |
| :--------------------------------------------------: |
| <img src="images/Labeler9.jpg" style="zoom:150%;" /> |

|                          (2)                          |                          (3)                          |                          (4)                          |
| :---------------------------------------------------: | :---------------------------------------------------: | :---------------------------------------------------: |
| <img src="images/Labeler10.jpg" style="zoom:100%;" /> | <img src="images/Labeler11.jpg" style="zoom:100%;" /> | <img src="images/Labeler12.jpg" style="zoom:100%;" /> |

**[Demo Video](https://youtu.be/Hdqk4p8Chow)**





## V. Appendix

### ball_pivoting.py

```python
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

```



## VI. Reference

* DotProduct LLC. Dot3D platform. Nov. 2, 2022. https://www.dotproduct3d.com/subscribe.html.

* Bernardini, F., Mittleman, J., Rushmeler, H., Silva, C. and Taubin, G. The ball-pivoting algorithm for surface reconstruction. IEEE. *TVCG*. 1999.

* Olga, S. Laplacian Mesh Processing. EUROGRAPHICS. 2005.

* Michael, K. Matthew, B. and Hugues, H. Poisson Surface Reconstruction. EUROGRAPHICS. 2006.

* Michael, G. Roland, K. Andreas, U. and Wolfgang, P. Blensor: Blender Sensor Simulation Toolbox. *ISVC*. 2011. https://www.blensor.org/.

* Lang, A. H. Vora, S. Caesar, H. Zhou, L. Yang, J. and Beijbom, O. Pointpillars: Fast encoders for object detection from point clouds. In *CVPR*., 2019.



## VII. Info

If you haver a question about the code or setting the environment, contact to me via e-mail

[ckdals915@handong.ac.kr](mailto:hey.ckdals915@handong.ac.kr)
