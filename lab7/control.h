/*
Author: Shanyu Jiang
Class: ECE6122
Last Date Modified: 02/12/2025
Description:
    Controller
*/

#pragma once
#include <cmath>
#include <algorithm>

namespace control 
{

struct Vec3 
{
    double x, y, z;

    Vec3(double x_=0, double y_=0, double z_=0) : x(x_), y(y_), z(z_) {}

    Vec3 operator+(const Vec3& o) const { return Vec3(x+o.x, y+o.y, z+o.z); }
    Vec3 operator-(const Vec3& o) const { return Vec3(x-o.x, y-o.y, z-o.z); }
    Vec3 operator*(double s) const { return Vec3(x*s, y*s, z*s); }
    Vec3 operator/(double s) const { return Vec3(x/s, y/s, z/s); }

    Vec3& operator+=(const Vec3& o) { x+=o.x; y+=o.y; z+=o.z; return *this; }
    Vec3& operator-=(const Vec3& o) { x-=o.x; y-=o.y; z-=o.z; return *this; }

    double dot(const Vec3& o) const { return x*o.x + y*o.y + z*o.z; }

    double mag() const { return std::sqrt(x*x + y*y + z*z); }

    Vec3 normalized() const 
    {
        double m = mag();
        if (m < 1e-8) return Vec3(0,0,0);
        return *this / m;
    }
};

inline double distance(const Vec3& a, const Vec3& b) 
{
    return (a - b).mag();
}

class PIDController 
{
public:
    PIDController(double kp_=0, double ki_=0, double kd_=0,
                  double integral_limit_=100.0, double output_limit_=100.0)
        : kp(kp_), ki(ki_), kd(kd_),
          integral(0.0), prev_error(0.0),
          integral_limit(integral_limit_),
          output_limit(output_limit_) {}

    void setGains(double p, double i, double d) 
    {
        kp = p; ki = i; kd = d;
    }

    double calculate(double error, double dt) 
    {
        if (dt <= 0.0) return 0.0;

        // P
        double p_term = kp * error;

        // I (with windup clamping)
        integral += error * dt;
        integral = std::clamp(integral, -integral_limit, integral_limit);
        double i_term = ki * integral;

        // D
        double derivative = (error - prev_error) / dt;
        double d_term = kd * derivative;

        prev_error = error;

        double out = p_term + i_term + d_term;
        out = std::clamp(out, -output_limit, output_limit);
        return out;
    }

    void reset() 
    {
        integral = 0.0;
        prev_error = 0.0;
    }

private:
    double kp, ki, kd;
    double integral;
    double prev_error;
    double integral_limit;
    double output_limit;
};

enum class Phase 
{
    GroundWait,
    ClimbToCenter,
    OnSphere
};

// Helper: convert Phase to human-readable string
inline const char* phaseToString(Phase p)
{
    switch (p) {
        case Phase::GroundWait:   return "GroundWait";
        case Phase::ClimbToCenter: return "ClimbToCenter";
        case Phase::OnSphere:     return "OnSphere";
        default:                  return "Unknown";
    }
}
struct ControlConfig 
{
    Vec3  center         = Vec3(0.0, 0.0, 50.0); // sphere center
    double sphereRadius  = 10.0;
    double groundWait    = 5.0;   // seconds
    double maxForce      = 20.0;  // N (total magnitude)
    double minSpeed      = 2.0;   // m/s
    double maxSpeed      = 10.0;  // m/s
};

struct ControlState 
{
    Phase  phase         = Phase::GroundWait;
    double timeInPhase   = 0.0;  // sec
    bool   visitedCenter = false;

    // For simple wandering on sphere
    Vec3 tangentialDir = Vec3(1, 0, 0);
};

struct ControlPIDs 
{
    PIDController radialPID;  // keep |r| ~ R
    PIDController speedPID;   // keep speed within band
};

// Utility: clamp a vector's magnitude
inline Vec3 clampMagnitude(const Vec3& v, double maxMag) 
{
    double m = v.mag();
    if (m <= maxMag || m < 1e-8) return v;
    return v * (maxMag / m);
}

// Main control law: given position/velocity, update control state and return force
inline Vec3 computeControlForce(
    const Vec3& pos,
    const Vec3& vel,
    ControlState& state,
    ControlPIDs& pids,
    const ControlConfig& cfg,
    double dt
) 
{
    state.timeInPhase += dt;

    // Phase transitions
    if (state.phase == Phase::GroundWait) 
    {
        if (state.timeInPhase >= cfg.groundWait) 
        {
            state.phase = Phase::ClimbToCenter;
            state.timeInPhase = 0.0;
        } 
        else 
        {
            // Sit on ground: motors off (ground reaction balances gravity)
            return Vec3(0, 0, 0);
        }
    }

    if (state.phase == Phase::ClimbToCenter) 
    {
        double dCenter = distance(pos, cfg.center);
        if (dCenter < 2.0) 
        { // "close enough"
            state.phase = Phase::OnSphere;
            state.timeInPhase = 0.0;
            state.visitedCenter = true;
            pids.radialPID.reset();
            pids.speedPID.reset();
        }
    }

    // Shared geometry
    Vec3 toCenter = cfg.center - pos;
    double r      = toCenter.mag();
    Vec3 radialDir = toCenter.normalized();

    if (state.phase == Phase::ClimbToCenter) 
    {
        // Simple "go to center" behaviour (radial PID towards center)
        double radialError = r; // want r -> 0
        double radialAccel = pids.radialPID.calculate(radialError, dt);
        Vec3 force = radialDir * radialAccel;

        // Also lightly limit speed ≤ 2 m/s by damping when too fast
        double speed = vel.mag();
        if (speed > 2.0) 
        {
            force -= vel.normalized() * (0.5 * (speed - 2.0));
        }

        return clampMagnitude(force, cfg.maxForce);
    }

    // Phase::OnSphere
    // --- radial control: keep |r| ≈ R ---
    double radialError = r - cfg.sphereRadius; // want this = 0
    double radialOut   = pids.radialPID.calculate(radialError, dt);
    Vec3 radialForce   = radialDir * (-radialOut); // push inward if outside

    // --- tangential wandering on sphere ---
    // Choose a tangential direction orthogonal to radialDir (simple deterministic choice)
    Vec3 worldUp(0, 0, 1);
    Vec3 tangent = Vec3(
        radialDir.y * worldUp.z - radialDir.z * worldUp.y,
        radialDir.z * worldUp.x - radialDir.x * worldUp.z,
        radialDir.x * worldUp.y - radialDir.y * worldUp.x
    );
    if (tangent.mag() < 1e-3) tangent = Vec3(1, 0, 0);
    tangent = tangent.normalized();
    state.tangentialDir = tangent; // could be randomized over time

    double speed = vel.mag();
    double targetSpeed = 0.5 * (cfg.minSpeed + cfg.maxSpeed); // mid-band
    double speedError  = targetSpeed - speed;
    double speedOut    = pids.speedPID.calculate(speedError, dt);

    Vec3 tangentialForce = tangent * speedOut;

    Vec3 totalForce = radialForce + tangentialForce;

    // Final clamp to max motor force
    return clampMagnitude(totalForce, cfg.maxForce);
}

} // namespace control
