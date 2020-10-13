#ifndef UKF_H
#define UKF_H

#include "Eigen/Dense"
#include "measurement_package.h"

class UKF {

 public:
  
  /* Constructor */
  UKF();

  /* Destructor */
  virtual ~UKF();

  /* The latest measurement data of either radar or laser */
  void ProcessMeasurement(MeasurementPackage meas_package);

  /* Predicts sigma points, the state, and the state covariance matrix */
  void Prediction(double delta_t);

  /* Updates the state and the state covariance matrix using a laser measurement */
  void UpdateLidar(MeasurementPackage meas_package);

  /* Updates the state and the state covariance matrix using a radar measurement */
  void UpdateRadar(MeasurementPackage meas_package);

  // initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  // if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  // if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // predicted sigma points matrix
  Eigen::MatrixXd Xsig_pred_;

  // time when the state is true, in us
  long long time_us_;

  // process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  // process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  // laser measurement noise standard deviation position1 in m
  double std_laspx_;

  // laser measurement noise standard deviation position2 in m
  double std_laspy_;

  // radar measurement noise standard deviation radius in m
  double std_radr_;

  // radar measurement noise standard deviation angle in rad
  double std_radphi_;

  // radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  // weights of sigma points
  Eigen::VectorXd weights_;

  // state dimension
  int n_x_;

  // augmented state dimension
  int n_aug_;

  // sigma point spreading parameter
  double lambda_;

  // current NIS for radar
  double NIS_radar_;

  // current NIS for laser
  double NIS_laser_;

};

#endif  // UKF_H
