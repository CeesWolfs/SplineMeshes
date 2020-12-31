cmake_minimum_required(VERSION 3.15)
project(CuboidalSplines CXX)

# Link this 'library' to use the warnings specified in CompilerWarnings.cmake
add_library(project_warnings INTERFACE)

# standard compiler warnings
include(cmake/CompilerWarnings.cmake)
set_project_warnings(project_warnings)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(ENABLE_TESTING "Enable Test Builds" ON)

include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup()

include(CheckIPOSupported)
check_ipo_supported(RESULT supported OUTPUT error)
if( supported )
    message(STATUS "IPO / LTO enabled")
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
else()
    message(STATUS "IPO / LTO not supported: <${error}>")
endif()

add_executable(CuboidalSplines main.cpp Mesh.cpp QuantitiesOfInterest.cpp Splines.hpp SubFaceTree.cpp Types.hpp QuantitiesOfInterest.hpp SubFaceTree.hpp Mesh.hpp)
target_link_libraries(CuboidalSplines ${CONAN_LIBS})

if(ENABLE_TESTING)
  enable_testing()
  message("Building Tests.")
  add_subdirectory(tests)
endif()
