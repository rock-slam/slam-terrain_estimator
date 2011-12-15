#include "ModelBaseAnalysis.hpp"

using namespace Eigen;
using namespace terrain_estimator;
using namespace std;


SVMTerrainClassification::SVMTerrainClassification(std::vector<TerrainType> terrain_types, int min_number_of_votes)
{
    this->terrain_types = terrain_types; 
    this->min_number_of_votes = min_number_of_votes; 
    svm_functions.clear(); 
}

void SVMTerrainClassification::addSVMClassifier(SVMConfiguration svm_function)
{
    svm_functions.push_back( svm_function); 
}


int SVMTerrainClassification::getIndexType(TerrainType type) 
{
    for(uint type_idx = 0; type_idx < terrain_types.size(); type_idx++)
	if( type == terrain_types.at(type_idx)) 
	    return type_idx; 
    return 0; 
}

TerrainType SVMTerrainClassification::getTerrainClassification(std::vector<double> histogram)
{
    int votes[terrain_types.size()]; 
    
    for( uint type = 0; type < terrain_types.size(); type++) 
	votes[type] = 0; 
    
    for(uint i = 0; i < svm_functions.size(); i++) 
    {
	SVMConfiguration svm = svm_functions.at(i); 

	double svm_value = svm.offset; 
	for(uint i = 0; i < svm.function.size() - 1 ; i ++)
	    svm_value = svm_value + histogram.at(i) * svm.function.at(i); 
	
	//std::cout << "SVM " << svm_value <<  " " << svm.upper_threshold << " " <<svm.lower_threshold<< std::endl; 
	
	if(svm_value >= svm.upper_threshold)
	    votes[getIndexType(svm.upper_type)]++; 
	else if(svm_value <= svm.lower_threshold) 
	    votes[getIndexType(svm.lower_type)]++; 
	else 
	    votes[getIndexType(UNKNOWN)]++; 
	
    } 
/*    
    //debug 
    for( int i = 0; i < 4; i++) 
    {
	std::cout << " Class " << i << " votes " <<  votes[i] << std::endl; 
    }
    std::cout << " *********************** " << std::endl; */

    for( uint type_idx = 0; type_idx < terrain_types.size(); type_idx++) 
    {
	if( votes[type_idx] >= min_number_of_votes ) 
	    return terrain_types.at(type_idx); 
    }

    return UNKNOWN; 

}



/** ********* HistogramTerrainClassification **************** */ 
/** ********* HistogramTerrainClassification **************** */ 
/** ********* HistogramTerrainClassification **************** */ 

HistogramTerrainClassification::HistogramTerrainClassification(uint number_histograms )
{
    this->number_histograms = number_histograms; 
}

bool HistogramTerrainClassification::addHistogram(std::vector<double> histogram_traction, std::vector<double> histogram_angular_velocity,  std::vector<double> histogram_linear_velocity )
{
    //groups the histogram of angular velocity, linear velocity and traction into a single histogram 
    std::vector<double> grouped_histograms; 
    for(uint i = 0; i < histogram_traction.size(); i++) 
	grouped_histograms.push_back(histogram_traction.at(i));
    
//     for(uint i = 0; i < histogram_angular_velocity.size(); i++) 
// 	grouped_histograms.push_back(histogram_angular_velocity.at(i));
//     
//     for(uint i = 0; i < histogram_linear_velocity.size(); i++) 
// 	grouped_histograms.push_back(histogram_linear_velocity.at(i));
    
    //creates the list of histograms
    histogram_list.push_back(grouped_histograms);
    
    if(histogram_list.size() < number_histograms)
	return false; 
    
    if(histogram_list.size() > number_histograms)
	histogram_list.pop_front(); 
    
    combined_histogram.clear(); 
    for(uint i = 0; i < grouped_histograms.size(); i++) 
	combined_histogram.push_back(histogram_list.at(0).at(i) / number_histograms );
    
    for(uint list = 1; list < number_histograms; list++)
	for(uint i = 0; i < grouped_histograms.size(); i++) 
	    combined_histogram.at(i) = combined_histogram.at(i) + histogram_list.at(list).at(i) / number_histograms;
	
    return true; 
}

std::vector<double> HistogramTerrainClassification::getCombinedHistogram()
{
    return combined_histogram; 
}





/** ********* TractionForceGroupedIntoStep **************** */ 
/** ********* TractionForceGroupedIntoStep **************** */ 
/** ********* TractionForceGroupedIntoStep **************** */ 

TractionForceGroupedIntoStep::TractionForceGroupedIntoStep(double angle_between_legs)
{
    this->angle_between_legs = angle_between_legs;
    
    initCurrentStep();
}

void TractionForceGroupedIntoStep::addTraction( double traction, double encoder)
{
    double id = getStepId(encoder);
    
    if( id != current_step.id) 
    {
	if( isCurrentStepCompleted() )  
	{
	    completed_step = current_step; 
	    initCurrentStep();
	}
    }
    
    current_step.id = id; 
    
    if( traction > current_step.max_traction )
	current_step.max_traction = traction; 
    else if( traction < current_step.min_traction )
	current_step.min_traction = traction;
    
    double legAngle = getLegAngle(encoder); 
    
    if( legAngle > current_step.max_angle )
	current_step.max_angle = legAngle; 
    else if( legAngle < current_step.min_angle )
	current_step.min_angle = legAngle;
    
    //this is to avoid filling the traction vector when there are stationary reading 
    if(encoder == current_step.last_encoder && current_step.traction.size() > 0) 
    {
	current_step.traction.back() = (current_step.traction.back() + traction ) / 2;
    }
    else 
    {
	current_step.traction.push_back(traction);
	current_step.last_encoder = encoder; 
    }
    
}
void TractionForceGroupedIntoStep::addRobotVelocities( double angular_velocity, double linear_velocity)
{
    current_step.angular_velocity.push_back(angular_velocity);
    current_step.linear_velocity.push_back(linear_velocity);
    
}

double TractionForceGroupedIntoStep::getCompletedStepId()
{
    return current_step.id; 
}

Step TractionForceGroupedIntoStep::getCompletedStep()
{
    return completed_step; 
}

bool TractionForceGroupedIntoStep::isCurrentStepCompleted()
{
    // a step should go from -pi/5 to pi/5 
    if( current_step.min_angle < -0.5 && current_step.max_angle > 0.5 )
	return true; 
    else
	return false; 
}

double TractionForceGroupedIntoStep::getStepId(double encoder)
{
    return floor( encoder / angle_between_legs);
}

void TractionForceGroupedIntoStep::initCurrentStep()
{
    current_step.id = 0; 
    current_step.last_encoder = 0; 
    current_step.max_angle = 3.14;
    current_step.min_angle = -3.14;
    current_step.max_traction = -999; 
    current_step.min_traction = 999; 
    current_step.traction.clear(); 
    current_step.angular_velocity.clear();
    current_step.linear_velocity.clear(); 
}

double TractionForceGroupedIntoStep::getLegAngle( double encoder )
{
    double legPos = encoder - (round(encoder / angle_between_legs) * angle_between_legs);
    
    if(legPos >= 0) 
	legPos = M_PI / 5 - legPos; 
    else
	legPos = - M_PI / 5 - legPos; 
    return legPos;
}


double TractionForceGroupedIntoStep::getMaximalTractionEitherStep()
{
    if(current_step.max_traction > completed_step.max_traction)
	return current_step.max_traction;
    else
	return completed_step.max_traction;
}

double TractionForceGroupedIntoStep::getMinimalTractionEitherStep()
{
    if(current_step.min_traction < completed_step.min_traction)
	return current_step.min_traction;
    else
	return completed_step.min_traction;
}

/** ********* HISTOGRAM **************** */ 
/** ********* HISTOGRAM **************** */ 
/** ********* HISTOGRAM **************** */ 

Histogram::Histogram(int numb_bins, double min_value, double max_value)
{
  
    for(int i =0; i < numb_bins ; i++) 
	histogram.push_back(0); 
    
    this->numb_bins = numb_bins; 
    this->min_value = min_value;
    this->max_value = max_value;
    number_points = 0; 
    bin_size = fabs(max_value - min_value) / (numb_bins-2); 
}


void Histogram::addValue(double value)
{
    number_points = number_points + 1;
   
    for(int i = 0; i < numb_bins - 1 ; i++) 
    {
	if (value <= min_value + i*bin_size ) 
	{
	    histogram.at(i) = histogram.at(i) + 1; 
	    return; 
	}
    }
    histogram.back() = histogram.back()  + 1;
    

}

void Histogram::clearHistogram()
{
    histogram.clear(); 
    
    for(int i =0; i < numb_bins ; i++) 
	histogram.push_back(0); 
    
    number_points = 0; 
}

std::vector<double> Histogram::getHistogram()
{
    if( number_points == 0 ) 
	return histogram; 
    
    std::vector<double> normalized_histogram; 
    normalized_histogram = histogram; 
    
    for(int i = 0; i < numb_bins; i++) 
	normalized_histogram.at(i) = normalized_histogram.at(i) / number_points; 
    
    return normalized_histogram; 
}

/** ********* SlipDetectionModelBased **************** */ 
/** ********* SlipDetectionModelBased **************** */ 
/** ********* SlipDetectionModelBased **************** */ 

SlipDetectionModelBased::~SlipDetectionModelBased()
{
    
}

SlipDetectionModelBased::SlipDetectionModelBased(double axis_rotation, uint fl, uint fr, uint rl, uint rr)
{ 
      this->axis_rotation = axis_rotation; 
     
      this->fl = fl; 
      this->fr = fr; 
      this->rl = rl; 
      this->rr = rr; 
      
      consectuive_slip_votes.setZero(); 
} 


Eigen::Vector4d SlipDetectionModelBased::slipValue(Vector4d translation, double right_translation, double left_translation)
{
    Vector4d slip; 
    slip[fr] = translation[fr] - right_translation; 
    slip[rr] = translation[rr] - right_translation; 
    slip[fl] = translation[fl] - left_translation; 
    slip[rl] = translation[rl] - left_translation; 
    return slip; 
}
bool SlipDetectionModelBased::slipDetection(Vector4d translation, double delta_heading_measured, double slip_threashold)
{
    
    delta_theta_model = ( (translation[fr] + translation[rr])/2 - (translation[fl] + translation[rl])/2 ) / axis_rotation; 

    double right_translation;
    double left_translation;
    Matrix4d slip_hypotesis; 
    
    //Hipotesis 0 - Front Right Didn't slip 
    right_translation = translation[fr]; 
    left_translation = right_translation - axis_rotation * delta_theta_measured; 
    slip_hypotesis.col(fr) = slipValue(translation, right_translation, left_translation); 
    
    //Hipotesis 1 - Rear Right Didn't slip 
    right_translation = translation[rr]; 
    left_translation = right_translation - axis_rotation * delta_theta_measured;     
    slip_hypotesis.col(rr) = slipValue(translation, right_translation, left_translation); 
    
    //Hipotesis 2 - Front Left Didn't slip 
    left_translation = translation[fl]; 
    right_translation = left_translation + axis_rotation * delta_theta_measured;     
    slip_hypotesis.col(fl) = slipValue(translation, right_translation, left_translation); 
    
    //Hipotesis 3 - Front Right Didn't slip 
    left_translation = translation[rl]; 
    right_translation = left_translation + axis_rotation * delta_theta_measured; 
    slip_hypotesis.col(rl) = slipValue(translation, right_translation, left_translation); 
    
   
    total_slip.setZero(); 
    for(int i = 0; i < 4; i++) 
	total_slip.transpose() = total_slip.transpose() + slip_hypotesis.row(i); 
    
    total_slip = total_slip.array()/3; 
    for( int i =0; i < 4; i ++) 
	total_slip[i] = fabs(total_slip[i]); 
    
    bool has_slip; 
    slip_votes.setZero(); 
    for( int i =0; i < 4; i++) 
	for( int j = 0; j < 4; j++ ) 
	    if( fabs(slip_hypotesis(i,j)) > slip_threashold )
	    {
		slip_votes[i] = slip_votes[i] + 1; 
		has_slip = true; 
	    }
	    
    analyzeConsecutiveSlips(); 
    
    return has_slip; 
    
}

void SlipDetectionModelBased::analyzeConsecutiveSlips()
{
    for(int i = 0; i < 4; i++) 
	if( slip_votes[i] > 1 ) 
	    consectuive_slip_votes[i] = consectuive_slip_votes[i] + slip_votes[i];
	else
	    consectuive_slip_votes[i] = 0; 
}

bool SlipDetectionModelBased::hasSingleWheelSliped()
{
    
    //the number of wheels with more than 2 votes as sliped 
    int num_wheels_votes = 0; 
    for( int i = 0; i < 4; i++) 
	if( slip_votes[i] >= 2) 
	    num_wheels_votes++; 
	
    if(num_wheels_votes == 1)
	return true; 
    else 
	return false; 
    
}

bool SlipDetectionModelBased::hasWheelSliped(int wheel)
{
    if( slip_votes[wheel] >= 2) 
	return true; 
    else 
	return false; 
}

bool SlipDetectionModelBased::hasWheelConsecutivelySliped(int wheel)
{
    if( consectuive_slip_votes[wheel] >=5 ) 
	return true; 
    else 
	return false; 
}

bool SlipDetectionModelBased::hasWheelSingleSliped(int wheel)
{

    if(getWheelSlipSingleCase() ==  wheel)
	return true; 
    else 
	return false; 
    
}


double SlipDetectionModelBased::getWheelSlipSingleCase(){ 
  
    if(!hasSingleWheelSliped())
	return -1; 

    for( int i = 0; i < 4; i++) 
	if( slip_votes[i] >= 2) 
	    return i; 
    
    return -1; 
} 


/** ********* AsguardOdometry **************** */ 
/** ********* AsguardOdometry **************** */ 
/** ********* AsguardOdometry **************** */ 

    
double AsguardOdometry::numberOfCycles(double encoder) 
{
    return floor(encoder / angle_between_legs);   
    
}

double AsguardOdometry::getLegPos(double external_encoder_value) 
{ 
  double legPos = external_encoder_value - (round(external_encoder_value / angle_between_legs) * angle_between_legs);
    
    if(legPos >= 0) 
	legPos = M_PI / 5 - legPos; 
    else
	legPos = - M_PI / 5 - legPos; 
    return legPos;
}

double AsguardOdometry::translationAxis(double encoder, double initial_encoder)
{
    double leg_pos = getLegPos(encoder); 
    double init_leg_pos = getLegPos(initial_encoder); 
    //the number of cycle since the initial encoder value given 
    double num_cycles = numberOfCycles( encoder )- numberOfCycles(initial_encoder); 

    //the translation in relation with the ground  
    return (num_cycles * 2 * sin(angle_between_legs/2) + sin(angle_between_legs/2) - sin(leg_pos) - ( sin(angle_between_legs/2) - sin(init_leg_pos))) * radius; 
    
}

Vector4d AsguardOdometry::translationAxes(Vector4d encoder){
    Vector4d translation; 
    for(int i = 0; i < 4; i++) 
      translation[i] = translationAxis(encoder[i], init_encoder[i]); 
    return translation; 
}


