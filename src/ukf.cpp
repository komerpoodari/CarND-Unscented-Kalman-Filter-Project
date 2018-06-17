#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include "tools.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;


/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.5; //30;  //komer - may need tuning.

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.5; //30;  //komer - may need tuning.
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  //Komer initialized the following. 
  is_initialized_ = false;  
  

  ///* State dimension
  n_x_ = 5;

  ///* Augmented state dimension
  n_aug_ = 7;

  ///* Sigma point spreading parameter
  lambda_ = 3. - n_aug_;

  ///* initialize heart beat counter 
  process_counter_ = 0;
  
  ///* state covariance matrix P
  ///* komer - Revisit this 
  P_ = MatrixXd(n_x_, n_x_); 
  P_ << 0.25, 0., 0., 0., 0.,
        0., 0.25, 0., 0., 0.,
        0., 0., 0.25, 0., 0.,   
        0., 0., 0., 0.25, 0.,
        0., 0., 0., 0., 0.25;
          
  ///* H matrix initiatization for Laser
  H_ = MatrixXd(2, 5);
  H_ << 1., 0., 0., 0., 0.,
        0., 1., 0., 0., 0.;
    
  ///* R for laser
  R_ = MatrixXd(2, 2);  
  R_ << 0.0225, 0,
        0, 0.0225;
  

  //create matrix with predicted sigma points as columns
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1); //gets updated in Prediction and used in update function of Radar
  
  //Create tool
  tool = Tools();
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  process_counter_++;
  //std::cout << "process_counter: " << process_counter_ << std::endl;
  
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "UKF: " << endl;
    
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      double rho = meas_package.raw_measurements_(0);
      double theta = meas_package.raw_measurements_(1);

      x_(0) = rho * cos(theta); //px
      x_(1) = rho * sin(theta); //py

    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      x_(0) = meas_package.raw_measurements_[0]; //px
      x_(1) = meas_package.raw_measurements_[1]; //py
    }
    
    if (fabs(x_(0)) < 0.0001 and fabs(x_(1)) < 0.0001) {
        x_(0) = 0.0001;
        x_(1) = 0.0001;
    }


    //set vector for weights
    double wi =  0.5 / (lambda_ + n_aug_);
    //std::cout << "wi: " << wi << std::endl;
    
    ///* weights dimension declaraton
    weights_ = VectorXd(2*n_aug_ + 1);  
    weights_.fill(wi);
    weights_(0) *= 2.0 * lambda_;
    //std::cout << "weights: " << weights_ << std::endl;
  
    // initalize timestamp
    previous_timestamp_ = meas_package.timestamp_;
        
    // done initializing, no need to predict or update
    is_initialized_ = true;
    cout << "Initialization complete" << endl;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  	
  //compute the time elapsed between the current and previous measurements  - Lesson 5, section 11 & 12. 
  double dt = (meas_package.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  
  
  //invoke Prediction()
  //ticks = clock();
  Prediction(dt);
  //ticks = clock() - ticks;
  //cout << "Predict exec ticks" << "ticks:" << ticks << endl;
  
  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {

    //call Radar update function
    if (use_radar_)
        UpdateRadar(meas_package); 
    
  } else {
    // call Laser updates
    if(use_laser_)
        UpdateLidar(meas_package);
  }

  // update timestamp
  previous_timestamp_ = meas_package.timestamp_;

  // print the output
  cout << "x_ = " << x_ << endl;
  cout << "P_ = " << P_ << endl;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  /*****************************************************************************
  *  Create augmented sigma points
  ****************************************************************************/
  //Lesson 7,  Section 17, komer's submitted quiz


  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_);

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

/*******************************************************************************
 * Xsig_aug Main logic begin
 ******************************************************************************/
  //std::cout << "delta_t: " << delta_t << std::endl;
  //create augmented mean state
  x_aug.setZero(n_aug_);
  x_aug.head(n_x_) = x_;
  //std::cout << "x_aug: " << std::endl << x_aug << std::endl;
  
 
  //create augmented covariance matrix
  P_aug.setZero(n_aug_, n_aug_);
  //std::cout << "P_aug: " << std::endl << P_aug << std::endl;
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  //std::cout << "P_aug: " << std::endl << P_aug << std::endl;
  
  MatrixXd Q_cov = MatrixXd::Zero(2, 2);
  Q_cov(0,0) = std_a_ * std_a_;
  Q_cov(1,1) = std_yawdd_ * std_yawdd_;
  //std::cout << "Q_cov: " << std::endl << Q_cov << std::endl;
  
  P_aug.bottomRightCorner(2, 2) = Q_cov;
  //std::cout << "P_aug: " << std::endl << P_aug << std::endl;
  
  //create square root matrix
  MatrixXd P_aug_root = MatrixXd(n_aug_, n_aug_);
  
  P_aug_root = P_aug.llt().matrixL();
  //std::cout << "P_aug_root " << std::endl << P_aug_root << std::endl;
  
  //create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  double lambda_term = sqrt (lambda_ + n_aug_); 
  
  // assign  x + the sqrt term
  for (int i = 0; i < n_aug_; i++)
  {
      Xsig_aug.col(i+1) = x_aug +  lambda_term * P_aug_root.col(i);
      Xsig_aug.col(n_aug_ + i+1) = x_aug - lambda_term * P_aug_root.col(i);
  }
  
  //std::cout << "Xsig_aug main logic end" << std::endl;
  /*******************************************************************************
  * Xsig_aug main logic end
  ******************************************************************************/
  
  /*******************************************************************************
  * Predict sigma points main logic begins
  ******************************************************************************/
  //Lesson 7, section 20, komer's assignment
 
 
  //std::cout << "Xsig_aug: "  << std::endl;
  //std::cout << Xsig_  << std::endl;
  int n_cols = 2 * n_aug_ + 1;
  for (int i = 0; i < n_cols; i++)
  {
      //get the each column elements into discrete variables
      double px = Xsig_aug(0,i);
      double py = Xsig_aug(1,i);
      double v  = Xsig_aug(2,i);
      double yaw = Xsig_aug(3,i);
      double yaw_d = Xsig_aug(4,i);
      
      double n_a = Xsig_aug(5,i);
      double n_ydd = Xsig_aug(6,i);
      
      //avoid division by zero
      double px_p, py_p;
      if (fabs(yaw_d) > 0.001)
      {
          px_p = px + (v/yaw_d)*(sin(yaw + yaw_d*delta_t) - sin(yaw));
          py_p = py + (v/yaw_d)*(cos(yaw) - cos(yaw + yaw_d*delta_t));
      }
      else 
      {
          px_p = px + v * cos(yaw) * delta_t;
          py_p = py + v * sin(yaw) * delta_t;
      }
      
      double v_p, yaw_p, yawd_p;
      v_p = v;
      yaw_p = yaw + yaw_d*delta_t;
      yawd_p = yaw_d;
      
      // add the second vector pertaining to noise.
      double delta_t2 = delta_t * delta_t;
      px_p += 0.5 * delta_t2 * cos(yaw) * n_a;
      py_p += 0.5 * delta_t2 * sin(yaw) * n_a;  
      v_p  += delta_t * n_a;
      yaw_p += 0.5 * delta_t2 * n_ydd;
      yawd_p += delta_t * n_ydd;

    //write predicted sigma points into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }
  //std::cout << "Xsig_pred main logic end" << std::endl;
  /*******************************************************************************
  * Predict new state mean and covariance predictions 
  ******************************************************************************/
  //Lesson 7, section 23, komer's assignment
  
  //create vector for predicted state
  VectorXd x = VectorXd(n_x_);

  //create covariance matrix for prediction
  MatrixXd P = MatrixXd(n_x_, n_x_);

/*******************************************************************************
 * main prediction logic begins
 ******************************************************************************/

  
  //predict state mean
  x.fill(0.0);
  for (int i = 0; i < (2*n_aug_ + 1); i++)
    x += weights_(i) * Xsig_pred_.col(i);
    
    
  //std::cout << "Xsig_pred_: "  << std::endl;
  //std::cout << Xsig_pred_  << std::endl;
  //std::cout << "x: "  << std::endl << x  << std::endl;
  
  //predict state covariance matrix
  P.fill(0.0);
  for (int i = 0; i < (2*n_aug_ + 1); i++)
  {
      VectorXd xd = Xsig_pred_.col(i) - x;
      

/**
      while (xd(3) < -M_PI)
      {
        xd(3) += 2.0*M_PI; 
        std::cout << "*" << xd(3) << "*";
      }   
      std::cout << std::endl;      
      while (xd(3) > M_PI)
      {
          xd(3) -= 2.0*M_PI;
          std::cout << "#" << xd(3) << "#";  
      }
      std::cout << std::endl;
**/
      //std::cout << "Prediction: xd(3): " << xd(3);
      xd(3) = tool.normalize_angle(xd(3));
      //std::cout << "; " << xd(3) << std::endl;
      P = P + weights_(i) * xd * xd.transpose() ;
  }
/*******************************************************************************
 * main prediction logic ends
 ******************************************************************************/
 //std::cout << "main prediction logic end" << std::endl;
  x_ = x;
  P_ = P;
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  VectorXd z = meas_package.raw_measurements_;
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = (H_ * P_ * Ht) + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - (K * H_)) * P_;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  // Lesson 7, section 26, komer's assignment 
  
  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

/*******************************************************************************
 * Student part begin
 ******************************************************************************/

  //transform sigma points into measurement space
  for (int i=0; i<(2 * n_aug_ + 1); i++)
  {
    // convert curtersian coordinates to polar coordinates - komer
    
    //extract curtasian coordinates
    double px = Xsig_pred_(0,i);
    double py = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);
    
    //calculate polar coordinates
    double rho = sqrt(px*px + py*py);
    if (rho <= 0.001)
        rho = 0.001;
        
    double phi = atan2(py, px);
    
    double rho_dot = ((px*cos(yaw)+ py*sin(yaw)) * v) / rho; 
    
    //assign to Z sigma
    
    Zsig(0,i) = rho;
    Zsig(1,i) = phi;
    Zsig(2,i) = rho_dot;
  }
  
  //calculate mean predicted measurement
  z_pred.fill(0.0);
  for (int i=0; i<(2 * n_aug_ + 1); i++)
  {
    z_pred += weights_(i)*Zsig.col(i);
  }
      

  //calculate innovation covariance matrix S
  S.fill(0.0);
  MatrixXd R = MatrixXd(n_z,n_z);
  R.fill(0.0);
  R(0,0) = std_radr_ * std_radr_;
  R(1,1) = std_radphi_ * std_radphi_;
  R(2,2) = std_radrd_ * std_radrd_;
  

  for (int i=0; i<(2 * n_aug_ + 1); i++)
  {
    VectorXd z_diff = VectorXd(n_z);
    z_diff = Zsig.col(i) - z_pred;

    S += weights_(i)*z_diff*z_diff.transpose();
  }
  S += R;
  
  
  
/*******************************************************************************
 * Student part end
 ******************************************************************************/

  //print result
  //std::cout << "z_pred: " << std::endl << z_pred << std::endl;
  //std::cout << "S: " << std::endl << S << std::endl;

  // Lesson 7, section 29, komer's assignment
  
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);

/*******************************************************************************
 * Student part begin
 ******************************************************************************/

  //calculate cross correlation matrix
  Tc.fill(0.);
  for (int i=0; i<2*n_aug_+1; i++)
  {
      VectorXd z_diff = Zsig.col(i) - z_pred;
      //normalize angle
      
 /**     
      while(z_diff(1) < -M_PI)  z_diff(1) += 2.*M_PI;
      while(z_diff(1) > M_PI)  z_diff(1) -= 2.*M_PI;
  **/
      z_diff(1) = tool.normalize_angle(z_diff(1));  
      
      VectorXd x_diff = Xsig_pred_.col(i) - x_;
      //normalize angle
      x_diff(3) = tool.normalize_angle(x_diff(3));
  /**
      while(x_diff(3) < -M_PI)  x_diff(3) += 2.*M_PI;
      while(x_diff(3) > M_PI)  x_diff(3) -= 2.*M_PI;
   **/
   
      Tc += weights_(i) * x_diff * z_diff.transpose(); 
  }
  //calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();
  
  //update state mean and covariance matrix
  VectorXd z = meas_package.raw_measurements_;   //komer - check this
  VectorXd z_diff = z - z_pred;
  //normalize angle
  z_diff(1) = tool.normalize_angle(z_diff(1));  
 
/** 
  while(z_diff(1) < -M_PI)  z_diff(1) += 2.*M_PI;
  while(z_diff(1) > M_PI)  z_diff(1) -= 2.*M_PI;
 **/  
  x_ += K * z_diff;
  
  //covariance
  P_ -= K * S * K.transpose();

}


