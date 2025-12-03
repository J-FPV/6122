/*
Author: Shanyu Jiang
Class: ECE6122
Last Date Modified: 02/12/2025
Description:
    Implementation file for UAV simulation.
*/

#include "simulation.h"
#include <chrono>
#include <cmath>

namespace sim 
{

using namespace std::chrono;
using control::Vec3;

UAV::UAV(const Vec3& startPos, const control::ControlConfig& cfg_)
    : position(startPos),
      velocity(0,0,0),
      acceleration(0,0,0),
      cfg(cfg_) {
    // Default PID gains
    pids.radialPID = control::PIDController(5, 1.0, 0.5, 100.0, 20.0);
    pids.speedPID  = control::PIDController(0.8, 0.0, 10.0, 100.0, 10.0);
}

UAV::~UAV() 
{
    stop();
}

void UAV::start() {
    if (running.load()) return;
    running = true;
    worker = std::thread(&UAV::threadFunc, this);
}

void UAV::stop() {
    if (!running.load()) return;
    running = false;
    if (worker.joinable()) {
        worker.join();
    }
}

Vec3 UAV::getPosition() const 
{
    std::lock_guard<std::mutex> lock(mtx);
    return position;
}

Vec3 UAV::getVelocity() const 
{
    std::lock_guard<std::mutex> lock(mtx);
    return velocity;
}

control::Vec3 UAV::getAcceleration() const 
{
    std::lock_guard<std::mutex> lock(mtx);
    return acceleration;
}

UAV::Snapshot UAV::getSnapshot() const 
{
    std::lock_guard<std::mutex> lock(mtx);
    return { position, velocity, acceleration };
}

void UAV::setVelocity(const Vec3& v) 
{
    std::lock_guard<std::mutex> lock(mtx);
    velocity = v;
}

void UAV::threadFunc() {
    const double dt = 0.01; // 10 ms

    while (running.load()) {
        // 1) Read current state (no long lock)
        control::Vec3 pos, vel;
        {
            std::lock_guard<std::mutex> lock(mtx);
            pos = position;
            vel = velocity;
        }

        // 2) Control: compute motor force
        control::Vec3 motorForce = control::computeControlForce(
            pos, vel, ctrlState, pids, cfg, dt
        );

        // 3) Physics: F = ma + gravity
        control::Vec3 accel = motorForce / mass + control::Vec3(0, 0, -g);

        // 4) Integrate using local copies
        vel += accel * dt;
        pos += vel * dt;

        // Ground contact
        if (pos.z < 0.0) {
            pos.z = 0.0;
            if (vel.z < 0.0) vel.z = 0.0;
        }

        // 5) Write back & enforce climb-phase speed limit
        {
            std::lock_guard<std::mutex> lock(mtx);

            // Limit speed to 2 m/s only during ClimbToCenter
            if (ctrlState.phase == control::Phase::ClimbToCenter) {
                double speed = vel.mag();
                const double maxClimbSpeed = 2.0;
                if (speed > maxClimbSpeed && speed > 1e-6) {
                    vel = vel * (maxClimbSpeed / speed);
                }
            }

            position = pos;
            velocity = vel;
            acceleration = accel;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

// Simple "swap velocities" collision response
void checkAndResolveCollisions(std::vector<std::unique_ptr<UAV>>& uavs,
                               double minDist) 
{
    const size_t n = uavs.size();
    if (n < 2) return;

    // take snapshot
    std::vector<UAV::Snapshot> snaps;
    snaps.reserve(n);
    for (auto& u : uavs) 
    {
        snaps.push_back(u->getSnapshot());
    }

    for (size_t i = 0; i < n; ++i)
    {
        for (size_t j = i+1; j < n; ++j)
        {
            double d = distance(snaps[i].pos, snaps[j].pos);
            if (d < minDist) 
            {
                // swap velocities
                uavs[i]->setVelocity(snaps[j].vel);
                uavs[j]->setVelocity(snaps[i].vel);
            }
        }
    }
}

} // namespace sim
