set(TERRAIN_HDRS
    VibrationAnalysis.hpp
    TerrainConfiguration.hpp
    )

set(TERRAIN_SRCS
    VibrationAnalysis.cpp
    )
rock_library(terrain_estimator 
    SOURCES ${TERRAIN_SRCS} 
    HEADERS ${TERRAIN_HDRS})
