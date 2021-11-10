cmake_minimum_required(VERSION 3.11 FATAL_ERROR)
project(ComputerGraphics CXX)

if (EXISTS "${CMAKE_CURRENT_LIST_DIR}/framework")
	# Create framework library and include CMake scripts (compiler warnings, sanitizers and static analyzers).
	add_subdirectory("framework")
else()
	# During development the framework lives in parent folder.
	add_subdirectory("../../../framework/" "${CMAKE_BINARY_DIR}/framework/")
endif()

find_package(assimp CONFIG REQUIRED)
find_package(OpenGL REQUIRED)

add_library(FinalProject1Lib STATIC "src/ray_tracing.cpp" "src/scene.cpp" )
target_link_libraries(FinalProject1Lib PUBLIC CGFramework assimp::assimp)
target_compile_features(FinalProject1Lib PUBLIC cxx_std_20)
set_project_warnings(FinalProject1Lib)
enable_sanitizers(FinalProject1Lib)

add_executable(FinalProject1 "src/main.cpp" "src/draw.cpp")
target_link_libraries(FinalProject1 PRIVATE FinalProject1Lib CGFramework OpenGL::GLU)
target_compile_features(FinalProject1 PRIVATE cxx_std_20)
enable_sanitizers(FinalProject1)
set_project_warnings(FinalProject1)

target_compile_definitions(FinalProject1 PRIVATE "-DDATA_DIR=\"${CMAKE_CURRENT_LIST_DIR}/data/\"" "-DOUTPUT_DIR=\"${CMAKE_CURRENT_LIST_DIR}/\"")

if (EXISTS "${CMAKE_CURRENT_LIST_DIR}/grading_tests/")
	add_subdirectory("grading_tests")
endif()
