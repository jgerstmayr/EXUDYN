
cmake_minimum_required(VERSION 3.0)

project(ngs_core)

set(NGS_LIB_TYPE STATIC CACHE STRING "ngs-core library type, default=STATIC")

set(NGS_CORE_CPP_FILES exception.cpp localheap.cpp paje_interface.cpp profiler.cpp
  table.cpp taskmanager.cpp bitarray.cpp)

set(CMAKE_CXX_STANDARD 14)

add_library(ngs_core ${NGS_LIB_TYPE} ${NGS_CORE_CPP_FILES})
set_target_properties(ngs_core PROPERTIES POSITION_INDEPENDENT_CODE ON )
target_include_directories(ngs_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
