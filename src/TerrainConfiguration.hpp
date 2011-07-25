#ifndef __TERRAIN_CONFIGURATION_TYPES__
#define __TERRAIN_CONFIGURATION_TYPES__


namespace terrain_estimator {
    /** 
     * Configuration on how to construct a point cloud 
     */ 
    struct VibrationConfiguration{
      	/**'minimal number of acc samples for creating a sample for estimating terrain*/
	unsigned int lines_per_terrain_sample;
	/** minimum number of lines that dont overlap with the sample */
	unsigned int min_line_advance; 
	/** number of points in the fft */ 
	unsigned int n_fft_points; 
	/** maximal gait standart deviation to accept the sample */
	double max_gait_STD;
	/** maximal angular velocity standart deviation to accept the sample */
	double max_angular_velocity_STD; 
	
    };
    
}
#endif


