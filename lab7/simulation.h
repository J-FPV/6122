/*
Author: Shanyu Jiang
Class: ECE6122
Last Date Modified: 02/12/2025
Description:
    Implementation file for UAV simulation.
*/

#pragma once
#include "control.h"
#include <thread>
#include <mutex>
#include <vector>
#include <memory>
#include <atomic>

namespace sim {

constexpr double g = 10.0;  // m/s^2
constexpr double mass = 1.0;

class UAV 
{
public:
    UAV(const control::Vec3& startPos,
        const control::ControlConfig& cfg = control::ControlConfig());

    ~UAV();

    void start();
    void stop();

    control::Vec3 getPosition() const;
    control::Vec3 getVelocity() const;
    control::Vec3 getAcceleration() const;

    struct Snapshot 
    {
        control::Vec3 pos;
        control::Vec3 vel;
        control::Vec3 acc;
    };

    Snapshot getSnapshot() const;
    void setVelocity(const control::Vec3& v);

private:
    void threadFunc();

    mutable std::mutex mtx;
    control::Vec3 position;
    control::Vec3 velocity;
    control::Vec3 acceleration;

    control::ControlConfig cfg;
    control::ControlState  ctrlState;
    control::ControlPIDs   pids;
    // For printing/debug: remember last printed phase and a small timer
    control::Phase lastPrintedPhase = control::Phase::GroundWait;
    double printTimer = 0.0;

    std::thread worker;
    std::atomic<bool> running{false};
};

// Collision helper – call this from main/render thread every frame
void checkAndResolveCollisions(std::vector<std::unique_ptr<UAV>>& uavs,
                               double minDist = 0.01); // 1 cm

} // namespace sim
