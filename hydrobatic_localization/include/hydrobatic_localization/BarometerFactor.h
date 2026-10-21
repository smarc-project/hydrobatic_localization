#ifndef HYDROBATIC_LOCALIZATION_BAROMETERFACTOR_H
#define HYDROBATIC_LOCALIZATION_BAROMETERFACTOR_H

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <boost/optional.hpp>
#include <memory>
#include <boost/optional/optional_io.hpp>
#include <iostream>
namespace gtsam {

class BarometerFactor : public NoiseModelFactorN<Pose3> {
 private:
    double measuredDepth_;
    Vector3 base_to_pressure_offset_;

 public:
  using Base = NoiseModelFactorN<Pose3>;
  BarometerFactor(){};
  virtual ~BarometerFactor() {}  
  /**
   * @brief Constructor of the BarometerFactor class
   * @param poseKey: the key of the pose
   * @param measuredDepth: the measured depth in meters from the barometer
   * @param baseToPressureOffset: the translation offset between base link and the pressure sensor
   * @param model: the noice model
   */
  BarometerFactor(Key poseKey, double measuredDepth, const Vector3 baseToPressureOffset, const SharedNoiseModel& model)
      : Base(model, poseKey), measuredDepth_(measuredDepth), base_to_pressure_offset_(baseToPressureOffset) {}

    /**
     * @brief the residual function
     * @param pose: the current estimated pose, correspond to the poseKey
     * @return the residual error.
     */
  Vector evaluateError(const Pose3 &pose,
                       gtsam::OptionalMatrixType H = OptionalNone ) const override;


};

}  // namespace gtsam
#endif  // HYDROBATIC_LOCALIZATION_BAROMETERFACTOR_H