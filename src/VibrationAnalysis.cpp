
#include "VibrationAnalysis.hpp"
#include <boost/concept_check.hpp>


using namespace Eigen;
using namespace terrain_estimator;


VibrationAnalysis::VibrationAnalysis(VibrationConfiguration conf)
{
    this->conf = conf; 

    in = (double*) fftw_malloc(sizeof(double) * FFT_POINTS);
    //since the data is real for the fft out[i] is the conjugate of out[n-i]
    out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * (FFT_POINTS/2 + 1) );
    p = fftw_plan_dft_r2c_1d(FFT_POINTS, in, out, FFTW_MEASURE);
    
    sum_gait = 0; 
    sum_angular_vel = 0; 
    square_sum_gait = 0; 
    square_sum_angular_vel = 0;

    accCount = 0; 
    lastAccIndex = 0; 
}

TerrainSample<VibrationAnalysis::FFT_POINTS> VibrationAnalysis::getTerrainSample()
{

    calculateFFT(); 

    TerrainSample<FFT_POINTS> terrain_sample; 	
    terrain_sample.mean_agular_velocity = sum_angular_vel / inputSamples.size(); 
    terrain_sample.mean_gait = sum_gait / inputSamples.size(); 
    terrain_sample.std_gait = sqrt( (square_sum_gait - sum_gait*(sum_gait/inputSamples.size()))/(inputSamples.size() - 1) );
    terrain_sample.std_angular_vel = sqrt( (square_sum_angular_vel - sum_angular_vel*(sum_angular_vel/inputSamples.size()))/(inputSamples.size() - 1) );

    for( int i = 0; i < FFT_POINTS/2 + 1; i++) 
	terrain_sample.acc_frequency(i,0) = sqrt( pow(out[i][0], 2) + pow(out[i][1], 2) );
    	

    lastAccIndex = accCount; 

    return terrain_sample;

} 
VibrationAnalysis::~VibrationAnalysis()
{
    fftw_destroy_plan(p);
    fftw_free(in); fftw_free(out);
}

void VibrationAnalysis::calculateFFT(){
  
    for( uint i = 0; i < inputSamples.size(); i++) 
	in[i] = inputSamples.at(i).acc; 
    // the number of points in the fft is bigger than the sample vector. 
    // The input vector should be the sample vector with 0 at the end. 
    for( uint i =  inputSamples.size(); i < FFT_POINTS; i++ ) 
	in[i] = 0; 
    
    fftw_execute(p); 
    
}


void VibrationAnalysis::addSample(double acc, double gait, double angular_velocity)
{
    InputSample sample;
    sample.acc = acc; 
    sample.gait = gait; 
    sample.angular_velocity = angular_velocity; 
    
    sum_gait = sum_gait + gait; 
    sum_angular_vel = sum_angular_vel + angular_velocity; 
    square_sum_gait = square_sum_gait + pow(gait,2); 
    square_sum_angular_vel = square_sum_angular_vel + pow( angular_velocity, 2); 
    
    inputSamples.push_back(sample);
    while( inputSamples.size() > conf.lines_per_terrain_sample ){
	sum_gait = sum_gait - inputSamples.front().gait; 
	sum_angular_vel = sum_angular_vel - inputSamples.front().angular_velocity; 
	square_sum_gait = square_sum_gait - pow(inputSamples.front().gait,2); 
	square_sum_angular_vel = square_sum_angular_vel - pow( inputSamples.front().angular_velocity, 2); 
	inputSamples.pop_front();
    }
    
    accCount++;
    
}

bool VibrationAnalysis::hasNewTerrainSample()
{
    //checks for the minimal amount of lines 
    
    if( accCount > conf.lines_per_terrain_sample && (accCount - lastAccIndex) > conf.min_line_advance )
    { 	
	
	//if there is enough lines, check if the sample is stable 
	double std_gait = sqrt( (square_sum_gait - sum_gait*(sum_gait/conf.lines_per_terrain_sample))/(conf.lines_per_terrain_sample - 1) ) ;
	double std_angular_vel = sqrt( (square_sum_angular_vel - sum_angular_vel*(sum_angular_vel/conf.lines_per_terrain_sample))/(conf.lines_per_terrain_sample - 1) ) ;
	
	if( std_gait > conf.max_gait_STD || std_angular_vel > conf.max_angular_velocity_STD)
	    return false; 
	else
	    return true;
    } 
    
    return false; 
}