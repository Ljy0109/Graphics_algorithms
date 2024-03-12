cmake_minimum_required(VERSION 3.10)
project(RayTracing)
 
set(CMAKE_CXX_STANDARD 17)
# 解决：多线程编译会出现“undefined reference to pthread_create”
find_package(Threads) #引入外部依赖包
# 解决：undefined reference to `omp_set_num_threads’
# 这涉及到OpenMP库的链接 https://blog.csdn.net/LimitOut/article/details/103550868 
FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

add_executable(RayTracing main.cpp Object.hpp Vector.cpp Vector.hpp Sphere.hpp global.hpp Triangle.hpp Scene.cpp
        Scene.hpp Light.hpp AreaLight.hpp BVH.cpp BVH.hpp Bounds3.hpp Ray.hpp Material.hpp Intersection.hpp
        Renderer.cpp Renderer.hpp)
 
target_link_libraries (${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT}) #链接 Thread 库