cmake_minimum_required(VERSION 3.10)
project(RayTracing)

set(CMAKE_CXX_STANDARD 17)

set(OpenMP_CXX_FLAGS "-Xpreprocessor -fopenmp -I/opt/homebrew/Cellar/libomp/16.0.6/include")
set(OpenMP_CXX_LIB_NAMES "omp")
set(OpenMP_omp_LIBRARY "/opt/homebrew/Cellar/libomp/16.0.6/lib/libomp.dylib")
find_package(OpenMP REQUIRED COMPONENTS CXX)


add_executable(RayTracing main.cpp Object.hpp Vector.cpp Vector.hpp Sphere.hpp global.hpp Triangle.hpp Scene.cpp
        Scene.hpp Light.hpp AreaLight.hpp BVH.cpp BVH.hpp Bounds3.hpp Ray.hpp Material.hpp Intersection.hpp
        Renderer.cpp Renderer.hpp)

#
#if(OPENMP_FOUND)
    target_link_libraries(RayTracing PRIVATE OpenMP::OpenMP_CXX)
#endif()

#set(CMAKE_CXX_FLAGS "${CAMKE_CXX_FLAGS} -O3 -fopenmp")