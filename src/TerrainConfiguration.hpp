#ifndef __TERRAIN_CONFIGURATION_TYPES__
#define __TERRAIN_CONFIGURATION_TYPES__

#include <vector>
#include <base/time.h>

namespace terrain_estimator {

    
    enum TerrainType{
	UNKNOWN, 
	GRASS, 
	PATH,
	PEBBLES
    }; 

    /**
     * @param terrain - the terrain type
     * @param probability - the probability 
     */
    struct TerrainProbability{
	TerrainType type; 
	double probability; 
	TerrainProbability()
	    : type(UNKNOWN){}
    }; 
    
    /**
    * @param time - the time at the end of the step where the slip was detected
     * @param wheel_idx - the wheel index where the terrain was classified
     * @param terrain - the terrain type with the probability 
     */
    struct TerrainClassification{
	base::Time time; 
	int wheel_idx; 
	std::vector<TerrainProbability> terrain;
    }; 
    
  
    /**
     * inputs a svm configuration for classifying a terrain type  
    * @param svm_function - the svm classification function 
    * @param offset - the offset of the classification function, so that the planes are separated around the 0 
    * @param lower_svm_threshold - all svm values lower than the lower_svm_threshold will be classified as the lower_type
    * @param lower_type - the classification for all svm values lower than lower_svm_threshold 
    * @param upper_svm_threshold - all svm values bigger than the upper_svm_threshold will be classified as the upper_type
    * @param upper_type - the classification for all svm values bigger than upper_svm_threshold 

     */ 
    struct SVMConfiguration{
	std::vector<double> function; 
	double offset; 
	double lower_threshold;
	TerrainType lower_type;
	double upper_threshold; 
	TerrainType upper_type; 
	#ifndef __orogen
	SVMConfiguration()
	    : lower_type(UNKNOWN),
	    upper_type(UNKNOWN){}
	#endif
    };
}
#endif


