#ifndef HYDROBATIC_LOCALIZATION_SAMMOTIONMODELFACTOR_H
#define HYDROBATIC_LOCALIZATION_SAMMOTIONMODELFACTOR_H

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <boost/optional.hpp>
#include <memory>
#include <boost/optional/optional_io.hpp>
#include <gtsam/navigation/ImuBias.h>
#include <iostream>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Matrix.h>
#include <hydrobatic_localization/SamMotionModel.h>
#include <gtsam/base/OptionalJacobian.h>
namespace gtsam {

struct Prediction{
  NavState state;
  Eigen::MatrixXd cov;
};

class PreintegratedMotionModel
{
  private:

  Eigen::VectorXd propagateStateVector(const Eigen::VectorXd& x,
                           double t0, double t1);

  // control sequence struct to store timestamp and control input
  struct controlSequence {
      double timestamp;
      Eigen::VectorXd u;};
      
  //list of the motion model inputs to use for the dynamics
  std::vector<controlSequence> control_list_;
  std::shared_ptr<SamMotionModelWrapper> sam_motion_model_; // might be better to seperate this from the preintegrated class since now every factro will share ownership
  controlSequence prev_control_;
  double dt_;
  NavState predictedState_j; 
  gtsam::Pose3 deltaPose_;       // Relative pose from i -> i+1
  gtsam::Vector3 deltaVel_;      // Relative velocity
  double deltaT_;  
  controlSequence prev_integrated_control_;            
  NavState motion_model_prediction_state_; // The state predicted by the motion model  

  // Covariance matrix for the noise propagation
  gtsam::Matrix covar_;

  public:
  /**
   * @brief Constructor for the PreintegratedMotionModel
   * @param dt: time step for the motion model
   */
  PreintegratedMotionModel(double dt): sam_motion_model_(std::shared_ptr<SamMotionModelWrapper>(new SamMotionModelWrapper(dt))) { 
    prev_control_.u = Eigen::VectorXd::Zero(6);
    prev_integrated_control_.u = Eigen::VectorXd::Zero(6);
  }
  /**
   * @brief Resets the control list and sets the prev control, should be after each optimization of the graph
   */
  void resetIntegration() {
    if (!control_list_.empty()) {
        prev_control_ = control_list_.back();
    } else {
        prev_control_.u = Eigen::VectorXd::Zero(6);
    }
    control_list_.clear();

  }


  /**
   * @brief Predicts the next state from the current state and the control inputs
   * @param state: the current state
   * @return NavState: the predicted state
   */
  NavState predict(const NavState& state,const Vector3& gyro,  const double start_time, const double end_time, const Eigen::MatrixXd& Sigma0) ;

  /**
   * @brief Function to add control inputs to the control seqeuence
   * @param u: the control input is either [4x1] or [2x1] depending if fb topic or command topic
   * @param timestamp: the timestamp of the message, this will be used to calculate the integration time
   * @param isThrusterVector: if true, u needs to be [2x1] and is a thruster vector cmd message, if false [4x1] and a fb control
   */
  void controlToList(const Eigen::VectorXd& u, const double& timestamp, const bool& isThrusterVector) ;
  
  /**     
   * @brief Function to convert the gtsam::NavState and gyro measurements to eta and nu for the motion model
   * @param state: the current NavState state from the optimizer in the navigation frame ENU
   * @param gyro: the raw gyro measurements from the IMU
   * @return Eigen::VectorXd: the state vector, eta and nu in NED. The linear and angluar velocities will tbe transformed to the base link frame 
   */
  Eigen::VectorXd stateToVector(const gtsam::NavState& state, const gtsam::Vector3 gyro) const;

  /** 
  * @brief Function to convert the state vector to a gtsam::NavState, converting from NED to ENU
  * @param state_vector: the state vector in NED
  * @return NavState: the gtsam::NavState object in NED
  */
  NavState vectorToState(const Eigen::VectorXd& state_vector, const NavState& pose1) const ;

  std::vector<controlSequence> getControlList() const {
      return control_list_;
  }
  /**
   * @brief Function to get the previous control input
   */
  Eigen::VectorXd getPrevControl() const {
      return prev_control_.u;
  }
  /**
   * @brief Function to get the delta pose computed from the predict function.
   */
      gtsam::Pose3 getDeltaPose() const {
        return deltaPose_;
    }

    /**
     * @brief Function to get the delta velocity computed from the predict function
     */
    gtsam::Vector3 getDeltaVel() const {
        return deltaVel_;
    }

    NavState getMotionModelPredictionState() const {
        return motion_model_prediction_state_;
    }


};

class SamMotionModelFactor : public NoiseModelFactorN<Pose3, Pose3, Vector3,Vector3> {
 private:

  double start_time_;
  double end_time_;
  const PreintegratedMotionModel PPM_;
  Vector3 gyro_;
  mutable bool nominal;
  mutable Matrix stored_H1_, stored_H2_, stored_H3_, stored_H4_;
  mutable Pose3 nom_Ti,nom_Tj;
  mutable Vector3 nom_velocity1,nom_velocity2;
  mutable Vector nominal_error_;

  
 public:
  using Base = NoiseModelFactorN<Pose3, Pose3, Vector3, Vector3>; //might need to include the bias for the gyro
  // SamMotionModelFactor(){};
  virtual ~SamMotionModelFactor() {}  
  /**
   * @brief Constructor
   * @param poseKey1 the previous key of the robot pose
   * @param velKey1 the previous key of the robot velocity
   * @param poseKey2 the current key of the robot pose
   * @param velKey2 the current key of the robot velocity
   * @param model the noise model for the factor
   * @param start_time the start time of the integration
   * @param end_time the end time of the integration
   * @param pmm the preintegrated motion model
   */
  SamMotionModelFactor(Key poseKey1,Key poseKey2, Key velKey1,Key velKey2 , const SharedNoiseModel& model, 
  const double& start_time, const double& end_time, const PreintegratedMotionModel& pmm, const Vector3& gyro):
  Base(model, poseKey1, poseKey2, velKey1, velKey2), start_time_(start_time), end_time_(end_time), PPM_(pmm), gyro_(gyro),nominal(true) {}
       
   /**
    * @brief Evaluate the error
    * @param pose1 the previous pose to evaluate the error
    * @param pose2 the current pose to evaluate the error
    * @param velocity1: the previous estimated velocity in the navigation frame
    * @param velocity2: the current estimated velocity in the navigation frame
    * @return returns the residual
    */
  Vector evaluateError(const Pose3 &pose1, const Pose3& pose2,const Vector3 &velocity1, const Vector3& velocity2,
                    gtsam::OptionalMatrixType H1 = OptionalNone, gtsam::OptionalMatrixType H2 = OptionalNone,
                     gtsam::OptionalMatrixType H3 = OptionalNone, gtsam::OptionalMatrixType H4 = OptionalNone) const override;

  Matrix getStoredH1() const { return stored_H1_; } 
  Matrix getStoredH2() const { return stored_H2_; }
  Matrix getStoredH3() const { return stored_H3_; }
  Matrix getStoredH4() const { return stored_H4_; }

  Vector rawError(
    const Pose3& Ti, const Pose3& Tj,
    const Vector3& vi, const Vector3& vj) const
  {
    Vector6 pose_err = Pose3::Logmap(
      PPM_.getDeltaPose().inverse()
              .compose(Ti.inverse().compose(Tj)));
    Vector3 vel_err = vj - (vi + PPM_.getDeltaVel());
    Vector e(9);
    e << pose_err, vel_err;
    return e;
}
};

}  // namespace gtsam
#endif  // HYDROBATIC_LOCALIZATION_SAMMOTIONMODELFACTOR_H