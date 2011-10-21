#include "ModelBaseAnalysis.hpp"

using namespace Eigen;
using namespace terrain_estimator;
using namespace std;

HistogramTerrainClassification::HistogramTerrainClassification(int numb_bins, double max_torque)
{
  
    for(int i =0; i < numb_bins ; i++) 
	histogram.push_back(0); 
    
    this->numb_bins = numb_bins; 
    this->max_torque = max_torque;
    number_points = 0; 
    bin_size = max_torque / (numb_bins-1); 
}


void HistogramTerrainClassification::addTraction(double traction)
{
    traction = fabs(traction); 
    
    number_points = number_points + 1;
   
    for(int i = 0; i < numb_bins; i++) 
    {
	if (traction <= (i+1)*bin_size ) 
	{
	    histogram.at(i) = histogram.at(i) + 1; 
	    return; 
	}
    }

}

void HistogramTerrainClassification::clearHistogram()
{
    histogram.clear(); 
    
    for(int i =0; i < numb_bins ; i++) 
	histogram.push_back(0); 
    
    number_points = 0; 
}

std::vector<double> HistogramTerrainClassification::getHistogram()
{
    if( number_points == 0 ) 
	return histogram; 
    
    std::vector<double> normalized_histogram; 
    normalized_histogram = histogram; 
    
    for(int i = 0; i < numb_bins; i++) 
	normalized_histogram.at(i) = normalized_histogram.at(i) / number_points; 
    
    return normalized_histogram; 
}


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


