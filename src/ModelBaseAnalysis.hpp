#ifndef MODEL_BASE_ANALYSIS_H
#define MODEL_BASE_ANALYSIS_H



#include <Eigen/Core>
#include <Eigen/Geometry>

#include "TerrainConfiguration.hpp" 
#include <iostream>



/**
 * Use Vehicle model based methods to estimate terrain 
 */
namespace terrain_estimator
{

    /** 
     * Given 2 wheels that should mantain a constant geometric distance between themself
     * This class calculates the aparent distance change between the axis. 
     * This class for the time assume the contact point to be the lowest feet (Flat ground assumtion) 
     */ 
    class SlipDetectionBasedOnDistanceBetweenAxis{
	public: 
	    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	    SlipDetectionBasedOnDistanceBetweenAxis(double front_encoder, double rear_encoder, double angle_between_legs); 
	  
	    /** 
	     * Returns how much the distance between the 2 axis changed 
	     */ 
	    double distanceVariation(double front_wheel_encoder, double back_wheel_encoder);
	    
	    /**
	     * Returns which wheel is sliping 
	     * If the distance variation between the axis exceed a certain value there is a slip
	     * But it can be a slip do to a wheel being drag or the wheel that moved sliped. 
	     * So based on the slip should also be caracterized by a sudden traction force droped. 
	     * So the wheel that sliped will be the one with the bigger force droped 
	     */
	    bool slipDetection(double front_encoder, double rear_encoder, double distance_threshold);
	    
	    double distance; 
	    	  
	    Eigen::Vector2d pos_front;
	    Eigen::Vector2d pos_back;

	private:

	    Eigen::Vector2d positionInAxisFrame(double encoder, double initial_encoder); 
	    /**
	     * The number of steps cycle since the initial encoder position 
	     * A cycle starts and ends when both feet are in contact with the ground 
	     */
	    double numberOfCycles(double encoder);
	    
	    /** angle of the leg with the vertical
	    0 means leg in vertical stance
	    -PI/5 or +PI/5 means double contact stance **/
	    double getLegPos(double external_encoder_value);
	    
	    /** The value of the encoder in the initial*/ 
	    Eigen::Vector2d transform;  
	    double init_front_encoder; 
	    double init_rear_encoder;
	    double angle_between_legs;
// 	    double axis_distance;
// 	    double radius;
	    double reference_dist; 
	    

	    
    }; 
    


}

#endif