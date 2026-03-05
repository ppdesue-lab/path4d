cmake_minimum_required(VERSION 3.10)

project(Clipper2)

#set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_CXX_EXTENSIONS OFF)
#set_property(GLOBAL PROPERTY USE_FOLDERS ON)

set(CLIPPER2_INC
    include/clipper2/clipper.h
    include/clipper2/clipper.core.h
    include/clipper2/clipper.engine.h
    include/clipper2/clipper.export.h
    include/clipper2/clipper.minkowski.h
    include/clipper2/clipper.offset.h
    include/clipper2/clipper.rectclip.h
)

set(CLIPPER2_SRC
  src/clipper.engine.cpp
  src/clipper.offset.cpp
  src/clipper.rectclip.cpp
)

add_library(Clipper2 STATIC ${CLIPPER2_INC} ${CLIPPER2_SRC})
target_include_directories(Clipper2 PRIVATE include)