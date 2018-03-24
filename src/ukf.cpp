#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

#define EPS 0.0001 // A very small number

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
    
  is_initialized_ = false;
     time_us_ = 0;
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2,
  // This is a reasonable assumption for a bike
  std_a_ = 1.8;

  // Process noise standard deviation yaw acceleration in rad/s^2
  // This is a reasonable assumption for a bike
  std_yawdd_ = .75;
    
    
    
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
    n_x_ = x_.size();
    n_aug_ = n_x_ + 2;
    n_sig_ = 2*n_aug_+1;
    lambda_ =  3 - n_aug_;
    weights_ = VectorXd(n_sig_);
    double weight_0 = lambda_/(lambda_+n_aug_);
    
    // Initilize the weights
    weights_(0) = weight_0;
    for (int i=1; i<n_sig_; i++) {
        double weight = 0.5/(n_aug_+lambda_);
        weights_(i) = weight;
    }
    
    // Initializa the measurement noise  co variance marix for Radar , which is 3 X 3
    R_radar = MatrixXd(3, 3);
    R_radar << std_radr_*std_radr_, 0, 0,
    0, std_radphi_*std_radphi_, 0,
    0, 0,std_radrd_*std_radrd_;
    
    
    // Initializa the measurement noise  co variance marix for Lidar , which is 2 X 2
    R_lidar = MatrixXd(2, 2);
    R_lidar << std_laspx_*std_laspx_,0,
    0,std_laspy_*std_laspy_;
 
    
    Xsig_pred_ = MatrixXd(n_x_, n_sig_);
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
    
    if (!is_initialized_)  {
         cout << "UKF: " << endl;
        
        // Initial covariance matrix
        P_ << 1, 0, 0, 0,0,
        0, 1, 0, 0,0,
        0, 0, 1, 0,0,
        0, 0, 0, 1,0,
        0, 0, 0, 0,1;
        
        
        if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
            
              ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
             cout << "UKF : First measurement RADAR" << endl;
             /**
              Convert radar from polar to cartesian coordinates and initialize state.
              */
             
             double rho = meas_package.raw_measurements_[0]; // range
             double phi = meas_package.raw_measurements_[1]; // bearing
             double rho_dot = meas_package.raw_measurements_[2]; // velocity of rho
             // Coordinates convertion from polar to cartesian
             double x = rho * cos(phi);
             double y = rho * sin(phi);
             float vx = rho_dot * cos(phi);
             float vy = rho_dot * sin(phi);
             float v  = sqrt(vx * vx + vy * vy);
             x_ << x, y, v,0,0;
             
         }
         else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
             /**
              Initialize state.
              */
             // No velocity and coordinates are cartesian already.
             cout << "UKF : First measurement LASER" << endl;
             double x = meas_package.raw_measurements_[0];
             double y = meas_package.raw_measurements_[1];
             x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0,0,0;
             if (fabs(x_(0)) < EPS and fabs(x_(1)) < EPS ) {
                 x_(0) = EPS;
                 x_(1) =  EPS;
             }
         }
       
       
        
        
        // Print the initialization results
        cout << "UKF init: " << x_ << endl;
        time_us_ = meas_package.timestamp_;
        
        // done initializing, no need to predict or update
        is_initialized_ = true;
        
         NIS_radar_count = 0;
         NIS_radar_over95 = 0;
        
         NIS_lidar_count = 0;
         NIS_lidar_over95 = 0;
        return;

    }
    // Calculate the timestep between measurements in seconds
    double dt = (meas_package.timestamp_ - time_us_);
    dt /= 1000000.0; // convert micros to s
    time_us_ = meas_package.timestamp_;
    
    Prediction(dt);
    //cout << "predict:" << endl;
    //cout << "x_" << endl << x_ << endl;
    
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
        //cout << "Radar " << measurement_pack.raw_measurements_[0] << " " << measurement_pack.raw_measurements_[1] << endl;
        UpdateRadar(meas_package);
    }
    if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
        //cout << "Lidar " << measurement_pack.raw_measurements_[0] << " " << measurement_pack.raw_measurements_[1] << endl;
        UpdateLidar(meas_package);
    }
    
    
    
    
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
    
    
    //create augmented mean vector
    VectorXd x_aug = VectorXd(n_aug_);
    
    //create augmented state covariance
    MatrixXd P_aug = MatrixXd(n_aug_,n_aug_);
    
    //create sigma point matrix
    MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sig_);
    
    x_aug.fill(0.0);
    x_aug.head(n_x_) = x_;
  
    
    //create augmented covariance matrix
    P_aug.fill(0.0);
    P_aug.topLeftCorner(n_x_,n_x_) = P_;
    P_aug(5,5) = std_a_*std_a_;
    P_aug(6,6) = std_yawdd_*std_yawdd_;
    
    //create square root matrix
    MatrixXd L = P_aug.llt().matrixL();
    
    Xsig_aug.col(0) = x_aug;
    // Pre calculate repeated expressions
    double sqrt_lamda_n_aug = sqrt(lambda_ + n_aug_);
    
    // Here we generate sigma points
    for (int i = 1; i <= n_aug_; i++) {
        Xsig_aug.col(i) = x_aug + sqrt_lamda_n_aug * L.col(i-1);
    }
    
    for (int i = n_aug_+1; i <= 2*n_aug_; i++) {
        Xsig_aug.col(i) = x_aug - sqrt_lamda_n_aug * L.col(i-1-n_aug_);
    }

    //predict sigma points
    //avoid division by zero
    //write predicted sigma points into right column
    for (int i = 0 ; i < n_sig_ ; i++ )   {
        double px = Xsig_aug(0,i);
        double py = Xsig_aug(1,i);
        double v = Xsig_aug(2,i);
        double yaw = Xsig_aug(3,i);
        double yawdd = Xsig_aug(4,i);
        double nu_a = Xsig_aug(5,i);
        double nu_dd = Xsig_aug(6,i);
        
        
        double px_p;
        double py_p  ;
        double v_p  ;
        double yaw_p;
        double yawdd_p ;
        
        // Pre calculate to save computation
        double sinyaw = sin(yaw);
        double cosyaw = cos(yaw);
        double yaw_yawdd_deltat = yaw + yawdd * delta_t;
        
        if (fabs(yawdd) >  EPS ) {
            double v_yawd = v / yawdd;
            px_p =  px + v_yawd * (sin(yaw_yawdd_deltat)  - sin(yaw));
            py_p =  py -  v_yawd * (cos(yaw_yawdd_deltat)  - cos(yaw));
        } else {
            double v_deltat = v * delta_t;
            px_p = px + cosyaw  * v_deltat;
            py_p =  py + sinyaw * v_deltat;
        }
        
        v_p  = v;
        yaw_p = yaw_yawdd_deltat;
        yawdd_p =  yawdd;
  
        double delta_t2 = delta_t*delta_t;
        
        double delta_t2_half__nua = 0.5 * delta_t2 *  nu_a;
        px_p = px_p + delta_t2_half__nua * cosyaw ;
        py_p = py_p + delta_t2_half__nua * sinyaw ;
        v_p = v_p + delta_t * nu_a;
        yaw_p = yaw_p + 0.5* delta_t2 * nu_dd;
        yawdd_p = yawdd_p + delta_t * nu_dd;
        
        Xsig_pred_(0,i) = px_p;
        Xsig_pred_(1,i) = py_p;
        Xsig_pred_(2,i) = v_p;
        Xsig_pred_(3,i) = yaw_p;
        Xsig_pred_(4,i) = yawdd_p;
        
    }
    
   
    //predicted state mean
    x_.fill(0.0);
    x_ = Xsig_pred_ * weights_; // vectorised sum
    
    //predicted state covariance matrix
    P_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        
        //angle normalization
        x_diff(3) = normalizeAngles(x_diff(3));
        
        P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
    }
}

/**
 A general method for code reuse hat si called several times
 */
double UKF::normalizeAngles(double angle)  {
    while (angle> M_PI) angle-=2.*M_PI;
    while (angle<-M_PI) angle+=2.*M_PI;
    return angle;
}

void UKF::UpdateUKF(MeasurementPackage meas_package, MatrixXd Zsig, int n_z) {
 
    //mean predicted measurement
    VectorXd z_pred = VectorXd(n_z);
    z_pred = Zsig * weights_;
    
    //measurement covariance matrix S
    MatrixXd S = MatrixXd(n_z,n_z);
   
    S.fill(0.0);
    for (int i = 0; i < n_sig_ ; i++) {
        VectorXd z_diff = Zsig.col(i) - z_pred;
        //angle normalization
        z_diff(1) = normalizeAngles(z_diff(1));
        S =  S + weights_(i) * z_diff * z_diff.transpose();
    }
    
    MatrixXd R = MatrixXd(n_z, n_z);
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR){ // Radar
        R = R_radar;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER){ // Lidar
        R = R_lidar;
    }
    S = S + R;
    
   
   
    //create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z);
    
    
    Tc.fill(0.0);
    for (int i = 0 ; i < n_sig_; i++) {
        
        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        //angle normalization
        
        z_diff(1) = normalizeAngles(z_diff(1));
        
        
        // state difference
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        //angle normalization
        x_diff(3) = normalizeAngles(x_diff(3));
        
        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }
    
    VectorXd z = meas_package.raw_measurements_;
    MatrixXd K = Tc * S.inverse();
    
    //residual
    VectorXd z_diff = z - z_pred;
    
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR){ // Radar
        //angle normalization
        z_diff(1) = normalizeAngles(z_diff(1));
    }
    
    x_ = x_ + K * z_diff;
    P_ = P_ - K * S * K.transpose();
    // Calculate NIS
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR){ // Radar
        NIS_radar = z_diff.transpose() * S.inverse() * z_diff;
        cout << " Radar NIS : " << NIS_radar << endl;
        NIS_radar_count++;
        if (NIS_radar > 7.815) {
            NIS_radar_over95++;
        }
        
        cout << " Radar NIS over95: " << (NIS_radar_over95 * 100.00) /  NIS_radar_count << endl;
        
       
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER){ // Lidar
        NIS_laser = z_diff.transpose() * S.inverse() * z_diff;
        cout << " Lidar NIS : " << NIS_laser << endl;
        
        NIS_lidar_count++;
        if (NIS_laser > 5.991) {
            NIS_lidar_over95++;
        }
        
        cout << " Lidar NIS over95: " << (NIS_lidar_over95 * 100.00) /  NIS_lidar_count << endl;
        
    }
    
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
    
    int n_z = 2;
    //create matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_z, n_sig_);
    
    for (int i = 0; i < n_sig_; i++) {
        // measurement model
        Zsig(0,i) = Xsig_pred_(0,i);
        Zsig(1,i) = Xsig_pred_(1,i);
    }
    UpdateUKF(meas_package, Zsig, n_z);
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
    int n_z = 3;
    
    //create matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_z, n_sig_);
    
    for (int i = 0; i < n_sig_; i++) {
        double p_x = Xsig_pred_(0,i);
        double p_y = Xsig_pred_(1,i);
        double v = Xsig_pred_(2,i);
        double yaw = Xsig_pred_(3,i);
        
        double v1 = cos(yaw)*v;
        double v2 = sin(yaw)*v;
        
        // transform the prediction onto the measurement space
        Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
        Zsig(1,i) = atan2(p_y,p_x);                                 //phi
        Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
        
    }
    UpdateUKF(meas_package, Zsig, n_z);
}
