cmake_minimum_required(VERSION 3.11)

set(projname path4d)

project(${projname})

include(FetchContent)

FetchContent_Declare(
  assimp
  GIT_REPOSITORY https://github.com/assimp/assimp.git
  GIT_TAG v5.3.1
)

FetchContent_MakeAvailable(assimp)

file(GLOB src_files *.cpp)

add_executable(${projname} ${src_files} "glm-aabb/aabb.cpp")
target_compile_definitions(${projname} PUBLIC APP_ROOT_PATH=\"${CMAKE_CURRENT_SOURCE_DIR}\")

target_link_libraries(${projname} assimp::assimp)

target_include_directories(${projname} PRIVATE D:/gitclones/Hazel/Hazel/vendor/glm)
