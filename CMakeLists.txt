cmake_minimum_required(VERSION 3.1)
project(cloth-sim)
set (CMAKE_CXX_STANDARD 11)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Set build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()


# Add your project files
file(GLOB_RECURSE SRCFILES "src/*.cpp")

if (MSVC)
  # For Microsoft compiler
  # Enable OpenMP
  # Enable Warning
  set_source_files_properties(
    ${SRCFILES}
    PROPERTIES
    COMPILE_FLAGS "/W4"
  )
else()
  # For other compilers
  # Enable OpenMP
  # Enable Warning
  set_source_files_properties(
    ${SRCFILES}
    PROPERTIES
    COMPILE_FLAGS "-Wall -Wextra -pedantic"
  )
  set(CMAKE_CXX_FLAGS_RELEASE "-O3")
endif()

# Link libraries
set(LINK_LIBS igl::core igl::opengl_glfw igl::opengl_glfw_imgui igl::embree embree)
if (NOT MSVC)
    list(APPEND LINK_LIBS stdc++fs)
endif()

# Link OpenMP
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# Link libigl
option(LIBIGL_WITH_OPENGL            "Use OpenGL"         ON)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"           ON)
option(LIBIGL_WITH_OPENGL_GLFW_IMGUI "Use IMGUI"          ON)
option(LIBIGL_WITH_EMBREE            "Use Embree"         ON)

find_package(LIBIGL REQUIRED QUIET)

# Link Embree
find_package(embree 3 CONFIG REQUIRED)

# Linking
add_executable(${PROJECT_NAME} ${SRCFILES})
target_link_libraries(${PROJECT_NAME} ${LINK_LIBS})
