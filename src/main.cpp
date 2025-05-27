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
    return 0;
}