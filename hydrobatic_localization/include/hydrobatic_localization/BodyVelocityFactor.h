#ifndef HYDROBATIC_LOCALIZATION_BODYVELFACTOR_H
#define HYDROBATIC_LOCALIZATION_BODYVELFACTOR_H

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
namespace gtsam {

class BodyVelocityFactor : public NoiseModelFactorN<Pose3, Vector3> {
 private:
    Vector3 velocity_measurement_;

 public:
  using Base = NoiseModelFactorN<Pose3, Vector3>;
  BodyVelocityFactor(){};
  virtual ~BodyVelocityFactor() {}  
  /**
   * @brief Constructor
   * @param poseKey the key of the robot pose
   * @param velKey the key of the robot velocity
   * @param velocity_mesurment the velocity measurement from the DVL
   * @param model the noise model for the factor
   */
  BodyVelocityFactor(Key poseKey, Key velKey, const Vector3 velocity_mesurment, const SharedNoiseModel& model)
      : Base(model, poseKey, velKey), velocity_measurement_(velocity_mesurment){}
       
   /**
    * @brief Evaluate the error
    * @param pose the pose to evaluate the error
    * @param velocity the estimated velocity
    * @return returns the residual
    */
  Vector evaluateError(const Pose3 &pose, const Vector3 &velocity,
                    gtsam::OptionalMatrixType H1 = OptionalNone, gtsam::OptionalMatrixType H2 = OptionalNone) const override;


};

}  // namespace gtsam
#endif  // HYDROBATIC_LOCALIZATION_BODYVELFACTOR_H