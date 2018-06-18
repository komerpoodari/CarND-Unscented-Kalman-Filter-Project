#include <iostream>
#include <math.h>
#include "tools.h"

using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  	VectorXd rmse(4);
	rmse << 0.0,0.0,0.0,0.0;
	

    // TODO: YOUR CODE HERE

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	// ... your code here

    int est_size = estimations.size();
    int grd_size = ground_truth.size();
    
    if ( (est_size == 0 ) || (est_size != grd_size))
    {
        cout << "Estimations Size: " << est_size <<"; Ground Truth Size: " << grd_size << endl;
        return rmse;
	}
	
	
	//accumulate squared residuals
	for(int i=0; i < est_size; ++i){
        // ... your code here
        VectorXd residual = ground_truth [i] - estimations [i];
            
        residual = residual.array() * residual.array();
        rmse += residual;
    }
	    
	//calculate mean
	// ... your code here
	rmse = rmse / est_size;

	//calculate the squared root
    rmse = rmse.array().sqrt();
    
	//return the result
	return rmse;
}

/**
   *  Normalize angle.
   */
double Tools::normalize_angle(double in_radians){
    double radians;
    radians = in_radians;
    if (fabs(radians) > 2*M_PI) {
      radians = fmodf(radians, 2*M_PI);
  }
  
  // in case absolute theta  > 180 degrees 
  if (radians > M_PI) {
      radians -= 2*M_PI;
  }
  else if (radians < -M_PI) {
      radians += 2*M_PI;
  }
  return radians;
}