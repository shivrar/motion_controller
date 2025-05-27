#include "motion_controller/RobotBaseModel.hpp"
#include <iostream>

namespace motion_controller
{
    class DiffDriveKinematics
        : public RobotKinematicsBase<DiffDriveKinematics, double, double> {
    public:
        DiffDriveKinematics(double wheel_radius, double wheel_base)
          : r_(wheel_radius), L_(wheel_base) {}

        Actuators robotTwistToActuatorsImpl(const Twist& twist) const {
            double vx = twist(0), wz = twist(5);
            Actuators out(2);
            out(0) = (vx - (L_ / 2.0) * wz) / r_; // left wheel
            out(1) = (vx + (L_ / 2.0) * wz) / r_; // right wheel
            return out;
        }

        Twist actuatorsToRobotTwistImpl(const Actuators& ws) const {
            double wL = ws(0), wR = ws(1);
            Twist twist = Twist::Zero();
            twist(0) = r_ * (wR + wL) / 2.0;
            twist(5) = r_ * (wR - wL) / L_;
            return twist;
        }
    private:
        double r_, L_;
        friend class RobotKinematicsBase<DiffDriveKinematics, double, double>;
    };


    // State: [x, y, theta, v, omega] (2D pose + linear and angular velocity)
    class DiffDriveDynamics : public RobotDynamicsBase<DiffDriveDynamics> {
    public:
        // Assume 2 actuators (left/right torques)
        using State = Eigen::Matrix<double, 5, 1>;
        using Actuators = Eigen::Matrix<double, 2, 1>;
        using Accel = Eigen::Matrix<double, 2, 1>; // [a_v, a_omega]

        // Robot params
        double mass = 10.0;     // kg
        double inertia = 1.0;   // kg.m^2
        double wheel_radius = 0.1;
        double wheelbase = 0.5;

        Accel actuatorsToAccelerationImpl(const State& state, const Actuators& torques) const {
            // Very basic model: tau = I * alpha for rotation, F = m*a for translation
            // Map left/right torques to [a_v, a_omega] (linear and angular acceleration)
            double tau_l = torques(0);
            double tau_r = torques(1);

            double force = (tau_l + tau_r) / wheel_radius;
            double torque = (tau_r - tau_l) * (wheelbase / 2.0) / wheel_radius;

            double a_v = force / mass;
            double a_omega = torque / inertia;

            Accel accel;
            accel << a_v, a_omega;
            return accel;
        }

        State integrateDynamicsImpl(const State& state, const Actuators& torques, double dt) const {
            // Unpack state
            double x = state(0);
            double y = state(1);
            double theta = state(2);
            double v = state(3);
            double omega = state(4);

            // Get accelerations
            Accel accel = actuatorsToAccelerationImpl(state, torques);
            double a_v = accel(0);
            double a_omega = accel(1);

            // Euler integration
            double new_v = v + a_v * dt;
            double new_omega = omega + a_omega * dt;
            double new_theta = theta + new_omega * dt;
            double new_x = x + new_v * std::cos(new_theta) * dt;
            double new_y = y + new_v * std::sin(new_theta) * dt;

            State new_state;
            new_state << new_x, new_y, new_theta, new_v, new_omega;
            return new_state;
        }
    };

}

int main()
{
    using namespace motion_controller;
    DiffDriveKinematics robot(0.1, 0.5);
    RobotKinematicsBase<DiffDriveKinematics, double, double>::Twist twist;
    twist.setZero();
    twist(0) = 1.0; // vx
    twist(5) = 0.3; // wz

    auto actuators = robot.robotTwistToActuators(twist);
    std::cout << "Wheel speeds: " << actuators.transpose() << std::endl;

    auto twist_back = robot.actuatorsToRobotTwist(actuators);
    std::cout << "Recovered twist: " << twist_back.transpose() << std::endl;

    motion_controller::DiffDriveDynamics model;
    motion_controller::DiffDriveDynamics::State state;       // [x, y, theta, v, omega]
    motion_controller::DiffDriveDynamics::Actuators torques; // [tau_l, tau_r]

    // Fill in state and torques...

    // Compute accelerations
    auto accel = model.actuatorsToAcceleration(state, torques);

    // Advance state by dt
    double dt = 0.01;
    auto new_state = model.integrateDynamics(state, torques, dt);

    return 0;
}