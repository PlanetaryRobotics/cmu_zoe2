#pragma once
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>
#include "rclcpp/logging.hpp"

// Constants
constexpr double EPSILON = 1e-6;
constexpr double PI = 3.14159265358979323846;

// Inline functions to replace macros
inline double absolute(double x) {
    return std::fabs(x);
}

inline bool doubleEqual(double a, double b) {
    return absolute(a - b) <= EPSILON;
}

inline bool doubleEqualWithError(double a, double b, double e) {
    return absolute(a - b) <= e;
}

class DrivingController {
private:
    // Command Variables
    double cVfr{0}, cVfl{0}, cVbr{0}, cVbl{0};
    
    // State
    double Vfr{0}, Vfl{0}, Vbr{0}, Vbl{0};

    const double B;    // Base Width
    const double Rw;   // Wheel Radius
    const double L;    // Length
    const double Kp; // Proportional gain
    
    double cV_x{0}; // Target velocity
    double cOmega_z{0}; // Target angular velocity

    double Thetaf{0}; // Front theta
    double Thetab{0}; // Back theta

    // Private member functions
    void computeFront();
    void computeBack();

public:
    DrivingController(double baseWidth, double wheelRadius, double length, double Kpval);

    // Getter and Setter methods
    double getVfr() const;
    double getVfl() const;
    double getVbr() const;
    double getVbl() const;

    void setVfr(double val);
    void setVfl(double val);
    void setVbr(double val);
    void setVbl(double val);

    double getcVfr() const;
    double getcVfl() const;
    double getcVbr() const;
    double getcVbl() const;

    double getWheelRadius() const;

    void setThetaf(double val);
    void setThetab(double val);

    void computeWheelSpeed();

    double cTheta() const;

    void setDriveCommand(double command_vel, double command_angular);
};
