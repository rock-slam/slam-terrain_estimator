cmake_minimum_required(VERSION 2.6)
include($ENV{ROCK_CMAKE_MACROS}/Rock.cmake)
rock_init(terrain_estimator 1.0)

#rock_find_pkgconfig(BASE_TYPES base-types REQUIRED)
#rock_find_cmake(Boost REQUIRED)
pkg_check_modules(FFTW REQUIRED "fftw3")
include_directories(${FFTW_INCLUDE_DIRS})
link_directories(${FFTW_LIBRARY_DIRS})

rock_export_includedir(${PROJECT_SOURCE_DIR}/src terrain_estimator)

rock_standard_layout()

# project(terrain_estimator)
# set(PROJECT_VERSION 1.0)
# 
# set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
# 
# cmake_minimum_required(VERSION 2.6)
# include(FindPkgConfig)
# 
# add_definitions(-Wall)
# 
# find_package(Eigen2 REQUIRED)
# include_directories(${EIGEN2_INCLUDE_DIR})
# 
# pkg_check_modules(FFTW REQUIRED "fftw3")
# include_directories(${FFTW_INCLUDE_DIRS})
# link_directories(${FFTW_LIBRARY_DIRS})
# 
# 
# include_directories(./src)
# 
# set(TERRAIN_HDRS
#     src/VibrationAnalysis.hpp
#     src/TerrainConfiguration.hpp
#     )
# 
# set(TERRAIN_SRCS
#     src/VibrationAnalysis.cpp
#     )
# 
# add_library(terrain_estimator SHARED ${TERRAIN_SRCS})
# 
# target_link_libraries(terrain_estimator fftw3)
# 
# 
# install(TARGETS terrain_estimator
#     RUNTIME DESTINATION bin
#     LIBRARY DESTINATION lib)
# install(FILES ${TERRAIN_HDRS} DESTINATION include/terrain_estimator)
# 
# configure_file(Doxyfile.in Doxyfile @ONLY)
# add_custom_target(doc doxygen Doxyfile)
# 
# configure_file(terrain_estimator.pc.in pose_ekf.pc @ONLY)
# install(FILES ${CMAKE_BINARY_DIR}/terrain_estimator.pc DESTINATION lib/pkgconfig)
# 
# include_directories(${PROJECT_SOURCE_DIR})
# 
# #add_subdirectory(test)
# 
# 
