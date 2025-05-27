#pragma once
#include "motion_controller/visibility_control.h"
#include <Eigen/Dense>
#include "design_patterns/CRTP.h" // Your CRTP base

namespace motion_controller {

  template <class Derived, class... Args>
  class RobotKinematicsBase : public CRTP<RobotKinematicsBase, Derived, Args...> {
  public:
    using Twist = Eigen::Matrix<double, 6, 1>;
    using Actuators = Eigen::VectorXd;

    Actuators robotTwistToActuators(const Twist& twist) const {
      return this->derived().robotTwistToActuatorsImpl(twist);
    }

    Twist actuatorsToRobotTwist(const Actuators& actuators) const {
      return this->derived().actuatorsToRobotTwistImpl(actuators);
    }

  private:
    RobotKinematicsBase() = default;
    friend Derived;
  };

} // namespace motion_controller
