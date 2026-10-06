#ifndef HYDROBATIC_LOCALIZATION_DVLFACTOR_H
#define HYDROBATIC_LOCALIZATION_DVLFACTOR_H

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

class DvlFactor : public NoiseModelFactorN<Pose3, Vector3, imuBias::ConstantBias> {
 private:
    Vector3 dvl_velocity_measurement_;
    Vector3 base_link_gyro_measurement_;
    Vector3 base_to_dvl_offset_;
    Rot3 base_link_dvl_rotations_;

 public:
  using Base = NoiseModelFactorN<Pose3, Vector3, imuBias::ConstantBias>;
  DvlFactor(){};
  virtual ~DvlFactor() {}  
  /**
   * @brief Constructor
   * @param poseKey the key of the robot pose
   * @param velKey the key of the robot velocity
   * @param bgyroKey the key of the gyro bias
   * @param dvl_velocity_mesurment the velocity measurement from the DVL
   * @param base_link_gyro_measurement the gyro measurement from the base_link
   * @param base_to_dvl_offset the offset between the base_link and the DVL
   * @param base_link_dvl_rotations the rotation between the base_link and the DVL
   * @param model the noise model for the factor
   */
  DvlFactor(Key poseKey, Key velKey, Key bgyroKey, const Vector3 dvl_velocity_mesurment, const Vector3 base_link_gyro_measurement,
   const Vector3 base_to_dvl_offset,const Rot3 base_link_dvl_rotations, const SharedNoiseModel& model)
      : Base(model, poseKey, velKey, bgyroKey), dvl_velocity_measurement_(dvl_velocity_mesurment), base_link_gyro_measurement_(base_link_gyro_measurement),
       base_to_dvl_offset_(base_to_dvl_offset), base_link_dvl_rotations_(base_link_dvl_rotations) {}
       
   /**
    * @brief Evaluate the error
    * @param pose the pose to evaluate the error
    * @param estimated_velocity the estimated velocity
    * @param bias_gyro the gyro bias
    * @return returns the residual
    */
  Vector evaluateError(const Pose3 &pose, const Vector3 &estimated_velocity, const imuBias::ConstantBias &bias_gyro,
                    gtsam::OptionalMatrixType H1 = OptionalNone, gtsam::OptionalMatrixType H2 = OptionalNone, gtsam::OptionalMatrixType H3 = OptionalNone) const override;


};

}  // namespace gtsam
#endif  // HYDROBATIC_LOCALIZATION_DVLFACTOR_H