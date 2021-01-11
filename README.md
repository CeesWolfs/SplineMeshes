# SplineMeshes
Final project TW3725TU, An implementation of splines on a 3D-hierarchical cuboidal mesh

# How to install
First we need cmake and conan installed on the system, download and install cmake and run
```
pip install conan
```
Then to install the project and dependencies issue the following commands
```
mkdir build
cd build
conan install ..
```
Then finally 
run ```cmake -G "Visual Studio 16" ..``` for Windows <br>
and ```cmake -G "Unix Makefiles" .. ``` Linux

For consistency, this is how the cuboids are always structured:
![alt text](https://github.com/CeesWolfs/SplineMeshes/blob/3D/images/image0.png?raw=true)  
  
This is the orientation that we adhere to for consistency and an example of split along XY plane:
![alt text](https://github.com/CeesWolfs/SplineMeshes/blob/3D/images/image1.png?raw=true)  

Example split along YZ plane:
![alt text](https://github.com/CeesWolfs/SplineMeshes/blob/3D/images/image2.png?raw=true)  

Example split along XY plane:
![alt text](https://github.com/CeesWolfs/SplineMeshes/blob/3D/images/image3.png?raw=true)

The 3D mesh domain has its following boundaries set at:
0 <= x <= 1, 0 <= y <= 1, 0 <= z <= 1

# Mesh division usage

Mesh refinement can be obtained by constructing an initial mesh
```
Mesh mesh;
```
and calling at least one or more of the split methods, depending on which plane and axis coordinate the splits should happen. 
For example:
```
mesh.SplitAlongYZ(0, 0.5);
```
splits the initial cuboid (which can be identified with id 0) along the YZ plane at x = 0.5. This will result in 2 cuboids from which the new cuboid gets id 1 (the old cuboid id incremented by 1).
