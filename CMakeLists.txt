cmake_minimum_required(VERSION 3.11)

set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

set(projname path4d)

project(${projname})

include(FetchContent)

FetchContent_Declare(
  assimp
  GIT_REPOSITORY https://github.com/assimp/assimp.git
  GIT_TAG v5.3.1
)

FetchContent_MakeAvailable(assimp)

FetchContent_Declare(
  glfw
  GIT_REPOSITORY https://github.com/glfw/glfw.git
  GIT_TAG 3.3.8
)

FetchContent_MakeAvailable(glfw)

FetchContent_Declare(
  glad
  GIT_REPOSITORY https://github.com/Dav1dde/glad.git
  GIT_TAG v0.1.36
)

set(GLAD_API gl=3.3 CACHE STRING "" FORCE)

FetchContent_MakeAvailable(glad)

file(GLOB src_files *.cpp)

add_executable(${projname} ${src_files} "glm-aabb/aabb.cpp")
target_compile_definitions(${projname} PUBLIC APP_ROOT_PATH=\"${CMAKE_CURRENT_SOURCE_DIR}\")

target_link_libraries(${projname} assimp::assimp glfw glad)

target_include_directories(${projname} PRIVATE D:/gitclones/Hazel/Hazel/vendor/glm)
