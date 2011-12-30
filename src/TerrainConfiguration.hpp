#ifndef __TERRAIN_CONFIGURATION_TYPES__
#define __TERRAIN_CONFIGURATION_TYPES__
#include <vector>

namespace terrain_estimator {

    
    enum TerrainType{
	UNKNOWN, 
	GRASS, 
	PATH,
	PEBBLES
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


