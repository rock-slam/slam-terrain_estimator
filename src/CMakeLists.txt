set(TERRAIN_HDRS
    TerrainConfiguration.hpp
    ModelBaseAnalysis.hpp
    )

set(TERRAIN_SRCS
    ModelBaseAnalysis.cpp
    )
rock_library(terrain_estimator 
    SOURCES ${TERRAIN_SRCS} 
    HEADERS ${TERRAIN_HDRS}
    DEPS_PKGCONFIG base-types
    )

