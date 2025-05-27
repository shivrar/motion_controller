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


  template <class Derived, class... Args>
class RobotDynamicsBase : public CRTP<RobotDynamicsBase, Derived, Args...> {
public:
    using Twist     = Eigen::Matrix<double, 6, 1>;     // [vx, vy, vz, wx, wy, wz]
    using Actuators = Eigen::VectorXd;                 // Actuator inputs (forces/torques)
    using State     = Eigen::VectorXd;                 // Full robot state: [position, orientation, velocity, ...]
    using Accel     = Eigen::Matrix<double, 6, 1>;     // [ax, ay, az, alphax, alphay, alphaz]

    // Map actuator inputs to generalized accelerations (robot frame)
    Accel actuatorsToAcceleration(const State& state, const Actuators& actuators) const {
        return this->derived().actuatorsToAccelerationImpl(state, actuators);
    }

    // Optionally: advance the full state (using, e.g., Euler integration)
    State integrateDynamics(const State& state, const Actuators& actuators, double dt) const {
        return this->derived().integrateDynamicsImpl(state, actuators, dt);
    }

protected:
    RobotDynamicsBase() = default;
    friend Derived;
};

} // namespace motion_controller
