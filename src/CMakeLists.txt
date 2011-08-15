set(TERRAIN_HDRS
    #VibrationAnalysis.hpp
    TerrainConfiguration.hpp
    ModelBaseAnalysis.hpp
    )

set(TERRAIN_SRCS
    #VibrationAnalysis.cpp
    ModelBaseAnalysis.cpp
    )
rock_library(terrain_estimator 
    SOURCES ${TERRAIN_SRCS} 
    HEADERS ${TERRAIN_HDRS})
