#ifndef MODEL_BASE_ANALYSIS_H
#define MODEL_BASE_ANALYSIS_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "TerrainConfiguration.hpp" 
#include <iostream>
#include <vector>
#include <deque>
using namespace Eigen; 


namespace terrain_estimator
{

    /**
     * \class SVMTerrainClassification
     * 
     * \brief
     * A batch of SVM function is used to classify the histogram 
     * \author $Author: Patrick Merz Paranhos $
     * \date $Date: 26/10/2011 $
     * 
     * Contact: patrick.merz_paranhos@dfki.de
     */ 
    class SVMTerrainClassification{
	public: 
	    
	    /**
	     * The type that will be detected is the first type to acquire the minimal number of votes 
	     * Each vote is casted by a svm function 
	     * @param terrain_types - the list of terrains types in the classification
	     * @param min_number_of_votes - minimal number of votes needed for a terrain to be detected (each svm function counts as one vote) -
	     */
	    SVMTerrainClassification(std::vector<TerrainType> terrain_types, int min_number_of_votes); 
	    
	    /**
	     * @param SVMConfiguration - the svm classification function 
	    */
	    void addSVMClassifier(SVMConfiguration svm_function);
	    
	    /**
	     * @param SVMConfiguration - the svm classification function 
	    */
	    void addSVMClassifier(std::vector<SVMConfiguration> svm_functions){ this->svm_functions = svm_functions; }
	    
	    /**
	     * Calculates the svm value from the combined histogram 
	     * The combined histogram needs to exist. 
	     * @return the terrain type as classified by the svm
	     */
	    TerrainType getTerrainClassification(std::vector<double> histogram); 
	
	    /**
	     * @param TerrainType - the terrain type 
	     * @return the probability distribution of a type based on the last terrain classification 
	     */ 
	    double getProbability(TerrainType type); 
	    
	private: 
	    
	    /**
	     * calculate the SVM value
	     */
	    void calculateSVMValue(); 
	    
	    int getIndexType(TerrainType type);
	    
	    std::vector < SVMConfiguration > svm_functions; 
	    
	    std::vector<TerrainType> terrain_types; 
	    
	    int min_number_of_votes;
	    
 	    std::vector<double> probability; 
	    
	    double svm_value;
	    
    }; 
    
    /**
     * \class HistogramTerrainClassification
     * 
     * \brief
     * A list of the last N histograms of traction values are combined into a single histogram.  
     * \author $Author: Patrick Merz Paranhos $
     * \date $Date: 26/10/2011 $
     * 
     * Contact: patrick.merz_paranhos@dfki.de
     */ 
    class HistogramTerrainClassification{
	public: 
	    
	    /**
	     * @param number_histograms - the number of histograms needed for a combined solution
	    */
	    HistogramTerrainClassification(uint number_histograms);
	    
	    /** 
	    * adds a histogram to the list of histograms 
	    * @param histogram_traction - a histogram of traction forces 
	    * @param histogram_angular_velocity - a histogram of angular velocities 
	    * @param histogram_linear_velocity - a histogram of linear velocities
	    * @return if there are enougth histograms for calculating a terrain classification solution (at least N histograms) 
	    */
	    bool addHistogram(std::vector<double> histogram, std::vector<double> histogram_angular_velocity,  std::vector<double> histogram_linear_velocity );
	    
	    /**
	     * @return the combined histogram 
	     */
	    std::vector<double> getCombinedHistogram(); 
	    
	
	private: 
	    std::deque < std::vector<double> > histogram_list; 
	    
	    std::vector<double> combined_histogram; 
	    
	    uint number_histograms;
    }; 
    
    /** A step information */ 
    struct Step{
	/** the tractions force of the step  */
	std::vector<double> traction; 
	/** the angular velocity of the robot during the step  */
	std::vector<double> angular_velocity; 
	/** the linear velocity of the robot  */
	std::vector<double> linear_velocity; 
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
	    TractionForceGroupedIntoStep(double angle_between_legs);
	    
	    /**
	     * adds the data to be grouped into a step 
	     * @param traction - the measured traction force value 
	     * @param encoder - the encoder position 
	     * @param angular_velocity - the robot angular velocity 
	     * @param linear_velocity - the robot linear velocity 
	     */ 
	    void addTraction(double traction, double encoder); 
	    
	    /**
	     * adds the velocity data to the current step 
	     * @param angular_velocity - the robot angular velocity 
	     * @param linear_velocity - the robot linear velocity 
	     */ 	    
	    void addRobotVelocities(double angular_velocity, double linear_velocity); 
	    
	    /** 
	     * @return the indentifier of the current completed step 
	     */ 
	    double getCompletedStepId(); 
	    
	    /**
	     * @return the current completed step 
	     */
	    Step getCompletedStep(); 
	    
	    /**
	     * @return the maximal value either step (completed or current) 
	     */
	    double getMaximalTractionEitherStep(); 
	    
	    /**
	     * @return the maximal value either step (completed or current) 
	     */	    
	     double getMinimalTractionEitherStep(); 
	     
	private:
	    
	    /** the vector of completed steps */ 
	    Step completed_step; 
	    
	    /** the current step */ 
	    Step current_step;
	    
	    /**
	    * Checks if the currents step is completed, which means transitioned from a double contact point to a DIFERENT double contact point
	    * A wheel that goes back to the same double contact point (rotate back and foward) is not considered as having completed a step 
	    * 
	    * @return if the current step is complete or not 
	    */
	    bool isCurrentStepCompleted();
	    
	    /** 
	     * Uses the encoder value to calculate the step 
	     * @return a unique value related to a wheel step 
	     */ 
	    double getStepId(double encoder); 
	    
	    /**
	     * Initializes the current step
	     */ 
	    void initCurrentStep(); 
	    
	    /**
	     * @param encoder the encoder reading 
	     * @return the leg angle from -PI/5 or +PI/5 
	     */ 
	    double getLegAngle( double encoder ); 
	    
	    double angle_between_legs;
	    
    };
    
    /**
     * \class Histogram
     * 
     * \brief
     * Creates a Histogram for measured values 
     * All value beneath min value are grouped in the first bin 
     * All values above max_value are grouped in the last bin 
     * \author $Author: Patrick Merz Paranhos $
     * \date $Date: 21/10/2011 $
     * 
     * Contact: patrick.merz_paranhos@dfki.de
     */ 
    class Histogram {
	public: 
	    /** 
	    * The histogram goes from min_value to max_value  
	    * all values above max_value are grouped in a single bin
	    * all values beneath min_value are grouped in a single bin
	    * So there are (numb_bins - 2) bins between max_value and min_value
	    * The bin size is (max_value - min_value) / num_bins-2 
	    * @param numb_bins -  number of bins in the histogram 
	    * @param min_value - the minimal value of the histogram 
	    * @param max_value - the minimal value of the histogram 
	    */ 
	    Histogram(int numb_bins, double min_value, double max_value); 
	    
	    /**
	     * @param value - a value to be added to the histogram
	    */
	    void addValue( double value ); 
	    
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
	    
	    /** the maximum histogram value */ 
	    double max_value; 
	    
	    /** the minimum histogram value */ 
	    double min_value; 
	    
	    /** number of bins in  the histogram */ 
	    int numb_bins; 
	    
	    /** the histogram*/
	    std::vector<double> histogram;
	    
	    /** the number of measurements added to the histogram */
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
	    * @return true if it is likely that this wheel sliped in the single sliped scenario 
	    */
	    bool hasWheelSingleSliped(int wheel); 
	    
	    /**
	    * @return true if it is likely that this wheel sliped
	    */
	    bool hasWheelSliped(int wheel); 

	    /**
	    * @return true if it is likely that this wheel sliped consecutiv time, which caracterize a strong slip chance. 
	    */
	    bool hasWheelConsecutivelySliped(int wheel); 
	    
	    /**
	     * accumulate the slip votes if they are consecutive 
	     */
	    void analyzeConsecutiveSlips();
	    
	    /** The change in the heading between times step in the model */ 
	    double delta_theta_model;
	    
	    /** The change in the heading between time steps measured by the imu */ 
	    double delta_theta_measured;
	    
	    /** The distance sliped in the defined time step (this value is calulate by averaging the distance slip in each hypotesis) */ 
	    Eigen::Vector4d total_slip;
	    
	    /** The number of votes casted by each of the hypotesis if this wheel is slipping or not */ 
	    Vector4d slip_votes; 
	    
	    /** accumulates consecutive slip votes over a period of time (it reduces the speed of the slip detection, but increases the certantie */ 
	    Vector4d consectuive_slip_votes; 
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
     * \class LegWheelOdometry
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
    class LegWheelOdometry{
	    
    public: 
	    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	    
	    LegWheelOdometry(double angle_between_legs, double radius){this->angle_between_legs = angle_between_legs; this->radius = radius; } 
	    

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
