#include "hydrobatic_localization/SamMotionModel.h"

SamMotionModelWrapper::SamMotionModelWrapper(double dt): dt_(dt)
{
  

    {
        pybind11::gil_scoped_acquire acquire;
        py::module motion_model = py::module::import("smarc_modelling.vehicles.SAM");
        sam_object_ = motion_model.attr("SAM")();
        dynamics_func_ = sam_object_.attr("dynamics");
        dt_func_ = sam_object_.attr("update_dt");
    }

}

Eigen::VectorXd SamMotionModelWrapper::Dynamics(const Eigen::VectorXd& x, const Eigen::VectorXd& u, double dt) const {

  try {
    py::object x_dot_py;
    pybind11::gil_scoped_acquire acquire;
    {
    dt_func_(dt);
    x_dot_py = dynamics_func_(x, u);
    }
    py::tuple x_dot_tuple = x_dot_py.cast<py::tuple>();
    return x_dot_tuple[0].cast<Eigen::VectorXd>();
  } catch (const py::error_already_set& e) {
    throw;
  }
}

Eigen::VectorXd SamMotionModelWrapper::integrateState(const Eigen::VectorXd& x, const Eigen::VectorXd& control,
                                                      const double& integration_time) const
{

    double remaining_time = integration_time;
    Eigen::VectorXd new_state = x;

    while (remaining_time > 0) {
        double dt_step = std::min(dt_, remaining_time);
        if (dt_step <= 0.0) {
            std::cout<< " [Warning] dt_step is less than or equal to 0.0, breaking the loop." << std::endl;
            break;
        }
        
        // Call the dynamics
        Eigen::VectorXd x_dot = Dynamics(new_state, control,dt_step);
        // Euler integration
        new_state = new_state + dt_step * x_dot;

        Eigen::Vector4d q = new_state.segment(3, 4);
        q.normalize();
        new_state.segment(3, 4) = q;

        remaining_time -= dt_step;
        
    }




    return new_state;
}
