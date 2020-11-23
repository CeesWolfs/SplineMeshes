# SplineMeshes
Final project TW3725TU, An implementation of splines on a 3D-hierarchical cuboidal mesh

# How to install
First we need cmake and conan installed on the system, download and install cmake and run
```
pip install conan
```
Then to install the project and dependencies issue the following commands
```
mkdir build && cd build
conan install ..
```
Then finally 
```cmake -G "Visual Studio 16" ..``` Windows
```cmake -G "Unix Makefiles" .. ``` Linux
