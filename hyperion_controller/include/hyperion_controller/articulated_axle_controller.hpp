// PassiveArticulatedKinematics.hpp
// Pure math controller from Shamah et al. (Hyperion rover steering)
// Computes wheel angular velocities [rad/s] given:
//   - desired forward speed v_d [m/s]
//   - commanded steering radius R_cmd [m] (sign: +left, -right, inf = straight)
//   - measured axle angle θ_a [rad]
// No clamping, timing, or safety checks are included.

#pragma once
#include <cmath>
#include <limits>

struct WheelSpeeds {
    // Order: front-left, front-right, rear-left, rear-right
    double w_fl{0.0}, w_fr{0.0}, w_rl{0.0}, w_rr{0.0};
};

class PassiveArticulatedKinematics {
public:
    struct Params {
        double L;
        double B;
        double r_wheel;
        double Kp;

        Params(double L_=1.50, double B_=1.20,
               double r_wheel_=0.25, double Kp_=30.0)
            : L(L_), B(B_), r_wheel(r_wheel_), Kp(Kp_) {}
    };

    explicit PassiveArticulatedKinematics(const Params& p = Params())
        : P(p) {}

    // Compute wheel angular velocities [rad/s]
    // v_d   = desired rover speed [m/s]
    // R_cmd = commanded arc radius [m] (inf = straight, sign = turn direction)
    // theta_a = measured axle angle [rad]
    WheelSpeeds compute(double v_d, double R_cmd, double theta_a) const {
        double theta_d = radiusToTheta(R_cmd);

        // Helper functions (paper Fig. 6)
        auto Rsteer = [&](double th)->double {
            return P.L / std::sin(th);
        };
        auto Rback = [&](double th)->double {
            double Rs = Rsteer(th);
            return std::sqrt(Rs*Rs - (P.L*P.L)/4.0);
        };
        auto Rrobot = [&](double th)->double {
            double Rb = Rback(th);
            return std::sqrt(Rb*Rb + (P.L*P.L)/4.0);
        };

        // Base wheel speed factor [rad/s]
        const double base = v_d / P.r_wheel;

        // --- FRONT (use θ_d) ---
        double Rs_d   = Rsteer(theta_d);
        double Rrob_d = Rrobot(theta_d);
        double f_front_inner = (Rs_d - P.B/2.0) / Rrob_d;
        double f_front_outer = (Rs_d + P.B/2.0) / Rrob_d;

        // --- REAR (use θ_a) ---
        double Rb_a   = Rback(theta_a);
        double Rrob_a = Rrobot(theta_a);
        double f_rear_inner = (Rb_a - P.B/2.0) / Rrob_a;
        double f_rear_outer = (Rb_a + P.B/2.0) / Rrob_a;

        WheelSpeeds w;

        // Decide inner/outer sides: left turn if θ>0
        if (theta_d >= 0.0) { // left turn
            w.w_fl = base * f_front_inner;
            w.w_fr = base * f_front_outer;
        } else {              // right turn
            w.w_fr = base * f_front_inner;
            w.w_fl = base * f_front_outer;
        }

        if (theta_a >= 0.0) { // left turn
            w.w_rl = base * f_rear_inner;
            w.w_rr = base * f_rear_outer;
        } else {              // right turn
            w.w_rr = base * f_rear_inner;
            w.w_rl = base * f_rear_outer;
        }

        // --- Proportional steering bias (Δω) ---
        double delta = P.Kp * (theta_d - theta_a);
        if (theta_d >= 0.0) { // left turn: inner=FL, outer=FR
            w.w_fl -= delta;
            w.w_fr += delta;
        } else {              // right turn: inner=FR, outer=FL
            w.w_fr -= delta;
            w.w_fl += delta;
        }

        return w;
    }

private:
    Params P;

    // Convert arc radius → desired axle angle θ_d
    double radiusToTheta(double R_cmd) const {
        if (std::isinf(R_cmd)) return 0.0; // straight
        double th = std::asin(P.L / std::fabs(R_cmd));
        if (R_cmd < 0) th = -th;
        return th;
    }
};
