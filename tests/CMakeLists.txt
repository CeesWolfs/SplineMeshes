# automatically enable catch2 to generate ctest targets
if(CONAN_CATCH2_ROOT_DEBUG)
  include(${CONAN_CATCH2_ROOT_DEBUG}/lib/cmake/Catch2/Catch.cmake)
else()
  include(${CONAN_CATCH2_ROOT}/lib/cmake/Catch2/Catch.cmake)
endif()

add_library(catch_main STATIC catch_main.cpp)
target_link_libraries(catch_main ${CONAN_LIBS})

add_executable(tests tests.cpp ../SubFaceTree.cpp ../SubFaceTree.hpp ../Mesh.cpp ../Mesh.hpp ../QuantitiesOfInterest.cpp ../QuantitiesOfInterest.hpp)
target_link_libraries(tests PRIVATE project_warnings catch_main)

# automatically discover tests that are defined in catch based test files you can modify the unittests.
catch_discover_tests(tests)
