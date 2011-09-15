#ifndef MODEL_BASE_ANALYSIS_H
#define MODEL_BASE_ANALYSIS_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "TerrainConfiguration.hpp" 
#include <iostream>
using namespace Eigen; 


namespace terrain_estimator
{
  
    /**
     * \class SlipDetectionModelBased
     * 
     * \brief
     * Estimates slip using the vehicle model constrains. 
     * 
     * Do to the Vehicle mechanical constrain. The distance between axis should be constant. 
     * Using the translation given by the encouder and a heading given by a measurement unit this class tryes to detect a wheel slip.  
     * The problem is underderterminated. So the slip detection works with hypotesis. 
     * 
     * 
     * \author $Author: Patrick Merz Paranhos $
     * \date $Date: 15/09/2011 $
     * 
     * Contact: patrick.merz_paranhos@dfki.de
     */ 
    class SlipDetectionModelBased{
	public: 
	    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	    /**
	     * @param axis_rotation - the axis of rotation for the vehicle 
	     * @param fl - the front left wheel index 
	     * @param fr - the front right wheel index 
	     * @param rl - the rear left wheel index 
	     * @param rr - the rear right wheel index 
	    */
	    SlipDetectionModelBased(double axis_rotation, uint fl, uint fr, uint rl, uint rr); 
	    
	    ~SlipDetectionModelBased(); 
	    
	    /**
	     * If the distance variation between the axis exceed a certain value there is a slip
	     * Since the problem is underterminated this work with multiple hypotesis that each wheel didn't slip 
	     * @param translation - each of the robot axis translation 
	     * @param delta_heading_measured - the measured heading CHANGE during the measured translation 
	     * @param slip_threashold - if the distance variation between axis exceed a certain value a slip is detected 
	     * @return slip? - if there was a slip or not 
	     */
	    bool slipDetection(Vector4d translation, double delta_heading_measured, double slip_threashold);
	    
	    /**
	     * @return The wheel that sliped or -1 if no wheel sliped 
	     */
	    double getWheelSlipSingleCase(); 
	    
	    /**
	    * @return true if it is likely that his wheel sliped in the single sliped scenario 
	    */
	    bool hasThisWheelSingleSliped(int wheel); 
	    
	    
	    double delta_theta_model;
	    double delta_theta_measured;
	    Eigen::Vector4d total_slip;
	    Vector4d slip_votes; 
	    
	private:
	    /**
	     * Return if it is probable to have a single wheel slip
	     */
	    bool hasSingleWheelSliped(); 
	    
	    /**
	     * Calculates how much each will sliped given the truth right and left translation
	     */ 
	    Eigen::Vector4d slipValue(Vector4d translation, double right_translation, double left_translation);

	    double axis_rotation;
	    uint fl;
	    uint fr;
	    uint rl;
	    uint rr; 

	    
    }; 
    
     /**
     * \class AsguardOdometry
     * 
     * \brief
     * Using asguard model it calculates the translation of each axis in the plane paralel to the ground
     * This model assumes flat ground. With lowest feet contact point. 
     * This code should later be integrated into asguard odometry. 
     * 
     * \author $Author: Patrick Merz Paranhos $
     * \date $Date: 15/09/2011 $
     * 
     * Contact: patrick.merz_paranhos@dfki.de
     */ 
    class AsguardOdometry{
	    
    public: 
	    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	    
	    AsguardOdometry(double angle_between_legs, double radius); 
	    

	    void setInitialEncoder(Vector4d encoder){ this->init_encoder = encoder; }  
	  
	    /**
	     * @Returns the translation in the paralel plane to soil the axis tranversed beetween 2 encouder measurement. 
	     */
	    Vector4d translationAxes(Vector4d encoder);
	    
	    /** 
	     * @Returns the translation in the paralel plane to soil the axis tranversed beetween 2 encouder measurement. 
	     */ 
	    double translationAxis(double encoder, double initial_encoder);

    private:
	    /**
	     * The number of steps cycle since the initial encoder position 
	     * A cycle starts and ends when both feet are in contact with the ground 
	     */
	    double numberOfCycles(double encoder);
	    
	    /** angle of the leg with the vertical
	    0 means leg in vertical stance
	    -PI/5 or +PI/5 means double contact stance **/
	    double getLegPos(double external_encoder_value);
  
	    Vector4d init_encoder; 
    
    private:    
	    //Asguard Model Parameters 
	    double angle_between_legs;
	    double radius; 
	    
    }; 
    


}

#endif