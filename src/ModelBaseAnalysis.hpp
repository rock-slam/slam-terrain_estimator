#ifndef MODEL_BASE_ANALYSIS_H
#define MODEL_BASE_ANALYSIS_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "TerrainConfiguration.hpp" 
#include <iostream>
#include <vector>
using namespace Eigen; 


namespace terrain_estimator
{


    /** A step information */ 
    struct step{
	/** the tractions force of the step  */
	std::vector<double> traction; 
	/** the maximal traction force in the step*/ 
	double max_traction; 
	/** the minimal traction force in the step*/ 
	double min_traction; 
	/** the minimal leg angle reading */ 
	double min_angle; 
	/** the maximal leg angle reading */ 
	double max_angle; 	    
	/** the step indentifier */ 
	double id; 
	/** last encoder reading */
	double last_encoder; 
    };
    
    /**
     * \class TractionForceGroupedIntoStep
     * 
     * \brief
     * Groups traction force measurement into a vector corresponding to a step
     * A step is caracterize by the transition, double contact to single contact to double contact
     * This class stores only the information of the previous completed step and a undergoing uncompleted Step. 
     * 
     * \author $Author: Patrick Merz Paranhos $
     * \date $Date: 26/10/2011 $
     * 
     * Contact: patrick.merz_paranhos@dfki.de
     */ 
    class TractionForceGroupedIntoStep{
	
	public: 
	    TractionForceGroupedIntoStep(); 
	    
	    /**
	     * adds a traction force to the corresponding step 
	     * @param wheel_idx - the index for the wheel 
	     * @param traction - the measured traction force value 
	     * @param encoder - the encoder position 
	     */ 
	    void addTraction(uint wheel_idx, double traction, double encoder); 
	    
	    /** 
	     * @param wheel_idx - the index for the wheel 
	     * @return the indentifier of the current completed step 
	     */ 
	    double getCompletedStepId(uint wheel_idx); 
	    
	    /**
	     * @param wheel_idx - the index for the wheel 
	     * @return the current completed step 
	     */
	    step getCompletedStep(uint wheel_idx); 
	    
	    
	private:
	    
	    /** the vector of completed steps */ 
	    step completed_step[4]; 
	    
	    /** the current step */ 
	    step current_step[4];
	    
	    /**
	    * Checks if the currents step is completed, which means transitioned from a double contact point to a DIFERENT double contact point
	    * A wheel that goes back to the same double contact point (rotate back and foward) is not considered as having completed a step 
	    * 
	    * @param wheel_idx - the index for the wheel 
	    * @return if the current step is complete or not 
	    */
	    bool isCurrentStepCompleted(uint wheel_idx);
	    
	    /** 
	     * Uses the encoder value to calculate the step 
	     * @return a unique value related to a wheel step 
	     */ 
	    double getStepId(double encoder); 
	    
	    /**
	     * Initializes the current step
	     * @param wheel_idx - the index for the wheel 
	     */ 
	    void initCurrentStep(uint wheel_idx); 
	    
	    /**
	     * @param encoder the encoder reading 
	     * @return the leg angle from 0 to 72 degree 
	     */ 
	    double getLegAngle( double encoder ); 
	    
    };
    
    /**
     * \class HistogramTerrainClassification
     * 
     * \brief
     * Creates a Histogram of the measured traction forces value.
     * \author $Author: Patrick Merz Paranhos $
     * \date $Date: 21/10/2011 $
     * 
     * Contact: patrick.merz_paranhos@dfki.de
     */ 
    class HistogramTerrainClassification {
	public: 
	    /** 
	    * The histogram goes from 0 to max_torque  
	    * all values above max_torque are grouped in a single bin
	    * The bin size is max_torque / num_bins-1 
	    * @param numb_bins -  number of bins in the histogram 
	    * @param max_torque - The histogram goes from 0 to max_torque 

	    */ 
	    HistogramTerrainClassification(int numb_bins, double max_torque); 
	    
	    /**
	     * Adds a measured traction force to the histogram 
	     * The absolute value of the torque is added to the histogram
	     * @param traction - the measured traction 
	    */
	    void addTraction( double traction ); 
	    
	    /**
	     * Clears the histogram
	     */ 
	    void clearHistogram();
	    
	    /**
	     * @returns a normalized copy of the histogram 
	     */ 
	    std::vector<double> getHistogram(); 
	    
	    /** 
	     * @return the number of points added to the histogram 
	     */ 
	    double getNumberPoints(){ return number_points; }
	    
	private:
	    /** The histogram goes from 0 to max_torque
	     * all values above max_torque are grouped in a single bin*/ 
	    double max_torque; 
	    
	    /** number of bins in  the histogram */ 
	    int numb_bins; 
	    
	    /** the histogram for the traction */
	    std::vector<double> histogram;
	    
	    /** the number of traction measurements added to the histogram */
	    double number_points; 
	    
	    /** the size of a bin */ 
	    double bin_size; 
    }; 
    
    /**
     * \class SlipDetectionModelBased
     * 
     * \brief
     * Estimates slip using the vehicle model constrains. 
     * 
     * Do to the Vehicle mechanical constrain the distance between axis should be constant. 
     * Using the translation given by the encouder and a heading given by a measurement unit one can estimate the position of each of the wheel axis.  
     *But do to the fact that each wheel can slip the problem is underderterminated. 
     * 
     * 4 hypotesis are created: each hypotesis assumes that one of the wheels didn't slip
     * Once you consider that 1 wheel has 0 meter slip (hypotesis) the problem becomes determined. 
     * Solving the problem will yeal 3 slip values, one for each wheel. 
     * If this slip value is above a certain threashold, that wheel is considered to be slipping under that hypotesis. 
     * 
     * Once all 4 hypotesis have been tested, each will have casted a vote if the other wheels are considered to be slipping or not. 
     * If a wheel get's 3 votes, while all other wheels have under 2 votes this wheel is considered to be very likely slipping.  
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
	    
	    /** The change in the heading between times step in the model */ 
	    double delta_theta_model;
	    /** The change in the heading between time steps measured by the imu */ 
	    double delta_theta_measured;
	    /** The distance sliped in the defined time step (this value is calulate by averaging the distance slip in each hypotesis) */ 
	    Eigen::Vector4d total_slip;
	    /** The number of votes casted by each of the hypotesis if this wheel is slipping or not */ 
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
	    
	    AsguardOdometry(double angle_between_legs, double radius){this->angle_between_legs = angle_between_legs; this->radius = radius; } 
	    

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