#include "zoe_controller/controller.hpp"

DrivingController::DrivingController(float baseWidth, float wheelRadius, float length, float Kpval)
    : B(baseWidth), Rw(wheelRadius), L(length), Kp(Kpval) {}

// Getter and Setter methods
float DrivingController::getVfr() const { return Vfr; }
float DrivingController::getVfl() const { return Vfl; }
float DrivingController::getVbr() const { return Vbr; }
float DrivingController::getVbl() const { return Vbl; }

void DrivingController::setVfr(float val) { Vfr = val; }
void DrivingController::setVfl(float val) { Vfl = val; }
void DrivingController::setVbr(float val) { Vbr = val; }
void DrivingController::setVbl(float val) { Vbl = val; }

float DrivingController::getcVfr() const { return cVfr; }
float DrivingController::getcVfl() const { return cVfl; }
float DrivingController::getcVbr() const { return cVbr; }
float DrivingController::getcVbl() const { return cVbl; }

float DrivingController::getWheelRadius() const { return Rw; }

void DrivingController::setThetaf(float val) { Thetaf = val; }
void DrivingController::setThetab(float val) { Thetab = val; }

// Set the target drive arc
void DrivingController::setTarget(float radius, float velocity) {
    cR = radius;
    cV = velocity;
}

// Compute steering angle
float DrivingController::cTheta() const {
    if (floatEqual(cR, 0)) {
        return PI / 2;
    }
    return std::atan(L / (2 * cR));
}

void DrivingController::computeFront() {
    float E1 = -(cTheta() - Thetaf);
    float E2 = -E1;
    cVfl += Kp * E1;
    cVfr += Kp * E2;
    // RCLCPP_INFO(rclcpp::get_logger("controller.hpp"), "Error Front: %f", E1);
    // RCLCPP_INFO(rclcpp::get_logger("controller.hpp"), "Thetaf: %f", Thetaf);
    // RCLCPP_INFO(rclcpp::get_logger("controller.hpp"), "cTheta: %f", cTheta());
}

void DrivingController::computeBack() {
    float E1 = cTheta() + Thetab;
    float E2 = -E1;
    cVbl += Kp * E1;
    cVbr += Kp * E2; 
    // RCLCPP_INFO(rclcpp::get_logger("controller.hpp"), "Error Back: %f", E1);
    // RCLCPP_INFO(rclcpp::get_logger("controller.hpp"), "Thetab: %f", Thetab);
    // RCLCPP_INFO(rclcpp::get_logger("controller.hpp"), "Target cTheta: %f", -cTheta());
}

void DrivingController::computeWheelSpeed() {
    if (floatEqual(cR, 0)) {
        cVfr = cV;
        cVfl = cV;
        cVbl = cV;
        cVbr = cV;
        return;
    }

    float commonTerm = cV / std::cos(cTheta());
    float adjustment = cV * B / (2 * cR);

    cVfl = commonTerm - adjustment;
    cVfr = commonTerm + adjustment;
    cVbl = commonTerm - adjustment;
    cVbr = commonTerm + adjustment;

    // RCLCPP_INFO(rclcpp::get_logger("controller.hpp"), "======================================");
    computeBack();
    // RCLCPP_INFO(rclcpp::get_logger("controller.hpp"), "======================================");
    computeFront();
    // RCLCPP_INFO(rclcpp::get_logger("controller.hpp"), "======================================");
}