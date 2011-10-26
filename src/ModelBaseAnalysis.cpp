#include "ModelBaseAnalysis.hpp"

using namespace Eigen;
using namespace terrain_estimator;
using namespace std;

HistogramTerrainClassification::HistogramTerrainClassification(uint number_histograms, std::vector<double> svm_function)
{
    this->svm_function = svm_function; 
    this->number_histograms = number_histograms; 
}

bool HistogramTerrainClassification::addHistogram(std::vector<double> histogram)
{
    histogram_list.push_back(histogram);
    
    if(histogram_list.size() < number_histograms)
	return false; 
    
    if(histogram_list.size() > number_histograms)
	histogram_list.pop_front(); 
    
    combined_histogram.clear(); 
    for(uint i = 0; i < histogram.size(); i++) 
	combined_histogram.push_back(histogram_list.at(0).at(i) / number_histograms );
    
    for(uint list = 1; list < number_histograms; list++)
	for(uint i = 0; i < histogram.size(); i++) 
	    combined_histogram.at(i) = combined_histogram.at(i) + histogram_list.at(list).at(i) / number_histograms;
	
    return true; 
}

std::vector<double> HistogramTerrainClassification::getCombinedHistogram()
{
    return combined_histogram; 
}

double HistogramTerrainClassification::getSVMValue()
{
    return 0; 
}

/** ********* TractionForceGroupedIntoStep **************** */ 
/** ********* TractionForceGroupedIntoStep **************** */ 
/** ********* TractionForceGroupedIntoStep **************** */ 

TractionForceGroupedIntoStep::TractionForceGroupedIntoStep()
{
    for( uint wheel_idx = 0; wheel_idx < 4; wheel_idx++) 
	initCurrentStep(wheel_idx);
}

void TractionForceGroupedIntoStep::addTraction(uint wheel_idx, double traction, double encoder)
{
    double id = getStepId(encoder);
    
    if( id != current_step[wheel_idx].id) 
    {
	if( isCurrentStepCompleted(wheel_idx) )  
	{
	    completed_step[wheel_idx] = current_step[wheel_idx]; 
	    initCurrentStep(wheel_idx);
	}
    }
    
    current_step[wheel_idx].id = id; 
    
    if( traction > current_step[wheel_idx].max_traction )
	current_step[wheel_idx].max_traction = traction; 
    else if( traction < current_step[wheel_idx].min_traction )
	current_step[wheel_idx].min_traction = traction;
    
    double legAngle = getLegAngle(encoder); 
    
    if( legAngle > current_step[wheel_idx].max_angle )
	current_step[wheel_idx].max_angle = legAngle; 
    else if( legAngle < current_step[wheel_idx].min_angle )
	current_step[wheel_idx].min_angle = legAngle;
    
    //this is to avoid filling the traction vector when there are stationary reading 
    if(encoder == current_step[wheel_idx].last_encoder && current_step[wheel_idx].traction.size() > 0) 
	current_step[wheel_idx].traction.back() = (current_step[wheel_idx].traction.back() + traction ) / 2; 
    else 
    {
	current_step[wheel_idx].traction.push_back(traction);
	current_step[wheel_idx].last_encoder = encoder; 
    }
    
}

double TractionForceGroupedIntoStep::getCompletedStepId(uint wheel_idx)
{
    return current_step[wheel_idx].id; 
}

step TractionForceGroupedIntoStep::getCompletedStep(uint wheel_idx)
{
    return completed_step[wheel_idx]; 
}

bool TractionForceGroupedIntoStep::isCurrentStepCompleted(uint wheel_idx)
{
    // a step should go from 0 to 70 degrees 
    if( current_step[wheel_idx].min_angle < 2.0 && current_step[wheel_idx].max_angle > 70.0 )
	return true; 
    else
	return false; 
}

double TractionForceGroupedIntoStep::getStepId(double encoder)
{
    return floor( encoder / (2.0 * M_PI / 5.0));
}

void TractionForceGroupedIntoStep::initCurrentStep(uint wheel_idx)
{
    current_step[wheel_idx].id = 0; 
    current_step[wheel_idx].last_encoder = 0; 
    current_step[wheel_idx].max_angle = 0;
    current_step[wheel_idx].min_angle = 72;
    current_step[wheel_idx].max_traction = -999; 
    current_step[wheel_idx].min_traction = 999; 
    current_step[wheel_idx].traction.clear(); 
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
  
    return has_slip; 
    
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

bool SlipDetectionModelBased::hasThisWheelSingleSliped(int wheel)
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


