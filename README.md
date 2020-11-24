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

Example split along ZY plane:
![alt text](https://github.com/CeesWolfs/SplineMeshes/blob/3D/images/image2.png?raw=true)  

Example split along ZX plane:
![alt text](https://github.com/CeesWolfs/SplineMeshes/blob/3D/images/image3.png?raw=true)
