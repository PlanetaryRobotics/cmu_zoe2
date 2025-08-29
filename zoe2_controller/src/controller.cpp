#include "zoe2_controller/controller.hpp"

DrivingController::DrivingController(double baseWidth, double wheelRadius, double length, double Kpval)
    : B(baseWidth), Rw(wheelRadius), L(length), Kp(Kpval) {}

// Getter and Setter methods
double DrivingController::getVfr() const { return Vfr; }
double DrivingController::getVfl() const { return Vfl; }
double DrivingController::getVbr() const { return Vbr; }
double DrivingController::getVbl() const { return Vbl; }

void DrivingController::setVfr(double val) { Vfr = val; }
void DrivingController::setVfl(double val) { Vfl = val; }
void DrivingController::setVbr(double val) { Vbr = val; }
void DrivingController::setVbl(double val) { Vbl = val; }

double DrivingController::getcVfr() const { return cVfr; }
double DrivingController::getcVfl() const { return cVfl; }
double DrivingController::getcVbr() const { return cVbr; }
double DrivingController::getcVbl() const { return cVbl; }

double DrivingController::getWheelRadius() const { return Rw; }

void DrivingController::setThetaf(double val) { Thetaf = val; }
void DrivingController::setThetab(double val) { Thetab = val; }

// Set the target drive arc
void DrivingController::setDriveCommand(double command_vel, double command_theta) {
    cV = command_vel;
    cTheta = command_theta;
}

// Compute steering angle
double DrivingController::cOmega_z() const {
    return cV * 2 * std::tan(cTheta) / L;
}

void DrivingController::computeFront() {
    double E1 = -(cTheta - Thetaf);
    double E2 = -E1;
    cVfl += Kp * E1;
    cVfr += Kp * E2;
    // RCLCPP_INFO(rclcpp::get_logger("controller.hpp"), "Error Front: %f", E1);
    // RCLCPP_INFO(rclcpp::get_logger("controller.hpp"), "Thetaf: %f", Thetaf);
    // RCLCPP_INFO(rclcpp::get_logger("controller.hpp"), "cTheta: %f", cTheta());
}

void DrivingController::computeBack() {
    double E1 = cTheta + Thetab;
    double E2 = -E1;
    cVbl += Kp * E1;
    cVbr += Kp * E2; 
    // RCLCPP_INFO(rclcpp::get_logger("controller.hpp"), "Error Back: %f", E1);
    // RCLCPP_INFO(rclcpp::get_logger("controller.hpp"), "Thetab: %f", Thetab);
    // RCLCPP_INFO(rclcpp::get_logger("controller.hpp"), "Target cTheta: %f", -cTheta());
}

void DrivingController::computeWheelSpeed() {

    double commonTerm = cV / std::cos(cTheta);
    double adjustment = 0.5 * B * cOmega_z();

    cVfl = commonTerm - adjustment;
    cVfr = commonTerm + adjustment;
    cVbl = commonTerm - adjustment;
    cVbr = commonTerm + adjustment;

    computeBack();
    computeFront();
}