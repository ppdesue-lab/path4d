cmake_minimum_required(VERSION 3.11)

set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

set(projname path4d)

project(${projname})
set(CMAKE_CXX_STANDARD 17)

include(FetchContent)

FetchContent_Declare(
  assimp
  GIT_REPOSITORY https://github.com/assimp/assimp.git
  GIT_TAG v5.3.1
)

set(ASSIMP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(ASSIMP_BUILD_SAMPLES OFF CACHE BOOL "" FORCE)
set(ASSIMP_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(ASSIMP_NO_EXPORT ON CACHE BOOL "" FORCE)
set(ASSIMP_BUILD_ALL_IMPORTERS_BY_DEFAULT OFF CACHE BOOL "" FORCE)
set(ASSIMP_BUILD_ALL_EXPORTERS_BY_DEFAULT OFF CACHE BOOL "" FORCE)

set(ASSIMP_BUILD_OBJ_IMPORTER ON CACHE BOOL "" FORCE)
set(ASSIMP_BUILD_PLY_IMPORTER ON CACHE BOOL "" FORCE)
set(ASSIMP_INSTALL_PDBS OFF CACHE BOOL "" FORCE)


FetchContent_MakeAvailable(assimp)

FetchContent_Declare(
  glfw
  GIT_REPOSITORY https://github.com/glfw/glfw.git
  GIT_TAG 3.3.8
)

set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(glfw)

FetchContent_Declare(
  glad
  GIT_REPOSITORY https://github.com/Dav1dde/glad.git
  GIT_TAG v0.1.36
)

set(GLAD_API gl=3.3 CACHE STRING "" FORCE)

FetchContent_MakeAvailable(glad)
FetchContent_Declare(
  canvas_ity
  GIT_REPOSITORY https://github.com/a-e-k/canvas_ity.git
  GIT_TAG main
)

FetchContent_MakeAvailable(canvas_ity)

FetchContent_Declare(
  stb
  GIT_REPOSITORY https://github.com/nothings/stb.git
  GIT_TAG master
)

FetchContent_MakeAvailable(stb)



file(GLOB src_files *.cpp)

add_executable(${projname} ${src_files} "glm-aabb/aabb.cpp")
target_compile_definitions(${projname} PUBLIC APP_ROOT_PATH=\"${CMAKE_CURRENT_SOURCE_DIR}\")

target_link_libraries(${projname} assimp::assimp glfw glad)

target_include_directories(${projname} PRIVATE D:/gitclones/Hazel/Hazel/vendor/glm ${canvas_ity_SOURCE_DIR} ${stb_SOURCE_DIR})
