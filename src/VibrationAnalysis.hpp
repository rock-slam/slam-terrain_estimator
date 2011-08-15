/*
 * =====================================================================================
 *
 *       Filename:  VibrationAnalysis.hpp
 *
 *    Description:  Vibration Based Terrain Estimation
 *
 *        Version:  1.0
 *        Created:  22/07/11 11:22:25
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Patrick Merz Paranhos (), 
 *        Company:  DFKI
 *
 * =====================================================================================
 */

#ifndef VIBRATION_ANALYSIS_H
#define VIBRATION_ANALYSIS_H



#include <Eigen/Core>

#include "TerrainConfiguration.hpp" 
#include <fftw3.h>

#include <deque>

namespace terrain_estimator
{

    template <unsigned int fft_points>
    struct TerrainSample {
	    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	    double mean_agular_velocity; 
	    double std_angular_vel; 
	    double mean_gait; 
	    double std_gait; 
	    Eigen::Matrix<double,fft_points / 2 + 1,1> acc_frequency; 
    };
    
    struct InputSample { 
	double acc;
	double gait; 
	double angular_velocity; 
    };
    
    class VibrationAnalysis
    {
	public:
	    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	    
	    /** number of points in the fft */ 
	    static const unsigned int FFT_POINTS = 128;
  
   	    VibrationAnalysis(VibrationConfiguration conf);

	    ~VibrationAnalysis();
	    
	    /** If there are enough information to make a new sample   */ 
	    bool hasNewTerrainSample(); 
	    
	    /** adds the input information for the sample */ 
	    void addSample(double acc, double gait, double angular_velocity); 
	    
	    /** creates and return the current terrain sample */ 
	    TerrainSample<FFT_POINTS> getTerrainSample();

	    
	public: 
	    /**the number of acceleration added */
	    uint accCount;
	    
	    /**the last acc idx added */
	    uint lastAccIndex; 
	
	private: 
	    /** Calculates the fft for the input sample */
	    void calculateFFT(); 
	    
	    /** configuration for the vibration based terrain analysis */ 
	    VibrationConfiguration conf; 

	    /** fft  output vector */ 
	    fftw_complex  *out;
	    /** fft  input vector */ 
	    double *in; 
	    
	    /** fft  plan */ 
	    fftw_plan p;
	    
	    std::deque<InputSample> inputSamples;
	    
	    double sum_gait; 
	    double sum_angular_vel; 
	    double square_sum_gait; 
	    double square_sum_angular_vel; 
  
    };
}
#endif
