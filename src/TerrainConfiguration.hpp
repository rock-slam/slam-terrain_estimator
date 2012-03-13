#ifndef __TERRAIN_CONFIGURATION_TYPES__
#define __TERRAIN_CONFIGURATION_TYPES__

#include <vector>
#include <base/time.h>
#include <base/eigen.h>

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

	/** 
	 * @brief convert an RGB color value into a terrain classification object
	 *
	 * This is simple for the currently available three classes.
	 * Once we extend these classes, more elaborate encoding 
	 * can be used. 
	 *
	 * @param color rgb color information that represents the likelihood of
	 *        terrain
	 */
	static TerrainClassification fromRGB( const base::Vector3d& color )
	{
	    terrain_estimator::TerrainProbability tp;
	    tp.probability = 0.9;
	    // TODO perturb the output and also add other classes

	    // HACK converting rgb values of the MLS into a 
	    // classification
	    // red - path
	    // green - grass
	    // blue - pebbles
	    int max_idx;
	    if( color.maxCoeff( &max_idx ) > 0.0 )
	    {
		switch( max_idx )
		{
		    case(0): tp.type = terrain_estimator::PATH; break;
		    case(1): tp.type = terrain_estimator::GRASS; break;
		    case(2): tp.type = terrain_estimator::PEBBLES; break;
		}
	    }
	    else
		tp.type = terrain_estimator::UNKNOWN;

	    terrain_estimator::TerrainClassification tc;
	    tc.terrain.push_back( tp );

	    return tc;
	}

	/** 
	 * @brief convert TerrainProbability to rgb value
	 *
	 * This function will encode a terrain probability as an rgb value.
	 * This is mainly so that it can be processed in maps more easiliy.
	 */
	const base::Vector3d toRGB() const
	{
	    // since currently the map patches provide a color
	    // information we also convert the
	    // terrain_classification information into a color
	    // information based on the terrain classes.

	    base::Vector3d prop_terrain = base::Vector3d::Zero();

	    for( std::vector<terrain_estimator::TerrainProbability>::const_iterator it = terrain.begin();
		    it != terrain.end(); it++)
	    {
		const terrain_estimator::TerrainProbability &tp( *it );
		switch( tp.type )
		{
		    case terrain_estimator::PATH: prop_terrain[0] = tp.probability; break;
		    case terrain_estimator::GRASS: prop_terrain[1] = tp.probability; break;
		    case terrain_estimator::PEBBLES: prop_terrain[2] = tp.probability; break;
		    default: break;
		}
	    }

	    return prop_terrain;
	}

	/**
	 * @brief calculate joint Probability of visual and proprioceptive
	 * terrain estimation
	 *
	 * This function will calculate the joint probability of visual and
	 * proprioceptive terrain estimation
	 */
	double jointProbability( const TerrainClassification& tc )
	{
	    // for now, do this in RGB space
	    // TODO this only works when there are only 3 classes
	    // will need to update for more classes later
	    base::Vector3d prop_terrain( toRGB() );
	    base::Vector3d visual_terrain( tc.toRGB() );

	    // for now use the crudest way possible to get the probability
	    // for the measurement
	    //double prob = 1.0 - (visual_terrain - prop_terrain).norm() / sqrt( 3.0 );

	    int idx1, idx2;
	    prop_terrain.maxCoeff( &idx1 );
	    visual_terrain.maxCoeff( &idx2 );

	    double prob = (idx1 == idx2) ? 0.9 : 0.1;

	    return prob;
	}
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


