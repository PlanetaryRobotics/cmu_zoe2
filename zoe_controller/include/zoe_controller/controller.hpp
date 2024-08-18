#pragma once
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>
#include "rclcpp/logging.hpp"

// Constants
constexpr float EPSILON = 1e-6f;
constexpr float PI = 3.14159265358979323846f;

// Inline functions to replace macros
inline float absolute(float x) {
    return std::fabs(x);
}

inline bool floatEqual(float a, float b) {
    return absolute(a - b) <= EPSILON;
}

inline bool floatEqualWithError(float a, float b, float e) {
    return absolute(a - b) <= e;
}

class DrivingController {
private:
    // Command Variables
    float cVfr{0}, cVfl{0}, cVbr{0}, cVbl{0};
    
    // State
    float Vfr{0}, Vfl{0}, Vbr{0}, Vbl{0};

    const float B;    // Base Width
    const float Rw;   // Wheel Radius
    const float L;    // Length
    const float Kp; // Proportional gain
    
    float cR{0}; // anti-clockwise is positive
    float cV{0}; // Target velocity

    float Thetaf{0}; // Front theta
    float Thetab{0}; // Back theta

    // Private member functions
    void computeFront();
    void computeBack();

public:
    DrivingController(float baseWidth, float wheelRadius, float length, float Kpval);

    // Getter and Setter methods
    float getVfr() const;
    float getVfl() const;
    float getVbr() const;
    float getVbl() const;

    void setVfr(float val);
    void setVfl(float val);
    void setVbr(float val);
    void setVbl(float val);

    float getcVfr() const;
    float getcVfl() const;
    float getcVbr() const;
    float getcVbl() const;

    float getWheelRadius() const;

    void setThetaf(float val);
    void setThetab(float val);

    void computeWheelSpeed();

    float cTheta() const;

    void setTarget(float radius, float velocity);
};
