#include "ModelBaseAnalysis.hpp"

using namespace Eigen;
using namespace terrain_estimator;
using namespace std;

SlipDetectionBasedOnDistanceBetweenAxis::SlipDetectionBasedOnDistanceBetweenAxis(double front_encoder, double rear_encoder, double angle_between_legs)
{ 
      transform << 10, 0; 
      init_front_encoder = front_encoder; 
      init_rear_encoder = rear_encoder; 
      this->angle_between_legs =  angle_between_legs;
      
      //place both frames within the same reference 
      //The transformation between the back wheel frame to the front wheel frame 
      Vector2d pos_front = positionInAxisFrame(front_encoder, init_front_encoder); 
      Vector2d pos_back = positionInAxisFrame(rear_encoder, init_rear_encoder); 
      //place both frame within the same reference 
      pos_front = pos_front + transform; 
      
      reference_dist = sqrt( pow(pos_front[0] - pos_back[0],2) + pow(pos_front[1] - pos_back[1],2) ); 
  
} 

double SlipDetectionBasedOnDistanceBetweenAxis::numberOfCycles(double encoder) 
{
    return floor(encoder / angle_between_legs);   
    
}

double SlipDetectionBasedOnDistanceBetweenAxis::getLegPos(double external_encoder_value) 
{ 
  double legPos = external_encoder_value - (round(external_encoder_value / angle_between_legs) * angle_between_legs);
    
    if(legPos >= 0) 
	legPos = M_PI / 5 - legPos; 
    else
	legPos = - M_PI / 5 - legPos; 
    return legPos;
}

Eigen::Vector2d SlipDetectionBasedOnDistanceBetweenAxis::positionInAxisFrame(double encoder, double initial_encoder)
{
    double leg_pos = getLegPos(encoder); 
    
    //the number of cycle since the initial encoder value given 
    double num_cycles = numberOfCycles( encoder )- numberOfCycles(initial_encoder); 
   
    //the position with reference to the axis frame 
    Vector2d pos(num_cycles * 2 * sin(angle_between_legs/2) + sin(angle_between_legs/2) - sin(leg_pos), cos(leg_pos) - cos ( angle_between_legs/2 )); 
    
    return pos;
}

double SlipDetectionBasedOnDistanceBetweenAxis::distanceVariation(double front_wheel_encoder, double back_wheel_encoder)
{
    //The frame is (0,0) at the beginning of the first cycle (double contact point) given by the initial encoder value
    //the position with reference to the axis frame 
    pos_front = positionInAxisFrame(front_wheel_encoder, init_front_encoder); 
    pos_back = positionInAxisFrame(back_wheel_encoder, init_rear_encoder); 
    
    //place both frame within the same reference 
    pos_front = pos_front + transform; 
    
    return sqrt( pow(pos_front[0] - pos_back[0],2) + pow(pos_front[1] - pos_back[1],2) ) - reference_dist; 
  
}

bool SlipDetectionBasedOnDistanceBetweenAxis::slipDetection(double front_encoder, double rear_encoder, double distance_threshold)
{
   distance = distanceVariation(front_encoder, rear_encoder); 

   if( fabs(distance) > distance_threshold) 
      return true; 
  else
      return false; 
    
}