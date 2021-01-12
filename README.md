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

The 3D mesh has the following domain: (0, 1) X (0, 1) X (0, 1)

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

# Mesh visualization usage

After having the mesh refinement phase, the user can save the mesh to a file as follows:
```
mesh.Save("<filename>");
```
The python script 'visualize.py' reads the mesh data from the generated '.ply' file.
Running the following command will display the mesh visualization with help of the pyvista module:
```
python visualize.py filename.ply
```
Make sure to have the pyvista module installed: https://docs.pyvista.org/getting-started/installation.html

A basic example for saving a mesh which is divided in 4 cuboids:
```
Mesh mesh;
mesh.SplitAlongXZ(0, 0.5);
mesh.SplitAlongXY(1, 0.5);
mesh.SplitAlongXY(0, 0.5);
mesh.Save("Fourths");
```
and running the command ```python visualize.py Fourths.ply``` will render the following mesh:
![alt text](https://github.com/CeesWolfs/SplineMeshes/blob/3D/images/fourths2.JPG?raw=true)
