#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <chrono>
#include <thread>

// 3D Vector class for position, velocity, and forces
class Vec3 {
public:
    double x, y, z;
    
    Vec3(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}
    
    Vec3 operator+(const Vec3& v) const { return Vec3(x + v.x, y + v.y, z + v.z); }
    Vec3 operator-(const Vec3& v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
    Vec3 operator*(double s) const { return Vec3(x * s, y * s, z * s); }
    Vec3 operator/(double s) const { return Vec3(x / s, y / s, z / s); }
    Vec3& operator+=(const Vec3& v) { x += v.x; y += v.y; z += v.z; return *this; }
    
    double magnitude() const { return sqrt(x*x + y*y + z*z); }
    double distance(const Vec3& v) const { return (*this - v).magnitude(); }
    Vec3 normalized() const { 
        double mag = magnitude();
        if (mag > 0) return *this / mag;
        return Vec3(0, 0, 0);
    }
};

// PID Controller class with improved control
class PIDController {
private:
    double kp, ki, kd;
    double integral;
    double prev_error;
    double integral_limit;
    double output_limit;
    
public:
    PIDController(double kp = 1.0, double ki = 0.0, double kd = 0.0, 
                  double integral_limit = 100.0, double output_limit = 50.0) 
        : kp(kp), ki(ki), kd(kd), integral(0), prev_error(0), 
          integral_limit(integral_limit), output_limit(output_limit) {}
    
    double calculate(double error, double dt) {
        // Proportional term
        double p_term = kp * error;
        
        // Integral term with anti-windup
        integral += error * dt;
        if (integral > integral_limit) integral = integral_limit;
        if (integral < -integral_limit) integral = -integral_limit;
        double i_term = ki * integral;
        
        // Derivative term (with filter for noise reduction)
        double derivative = 0;
        if (dt > 0) {
            derivative = (error - prev_error) / dt;
        }
        double d_term = kd * derivative;
        
        prev_error = error;
        
        // Calculate total output
        double output = p_term + i_term + d_term;
        
        // Limit output
        if (output > output_limit) output = output_limit;
        if (output < -output_limit) output = -output_limit;
        
        return output;
    }
    
    void reset() {
        integral = 0;
        prev_error = 0;
    }
    
    void setGains(double p, double i, double d) {
        kp = p;
        ki = i;
        kd = d;
    }
    
    double getIntegral() const { return integral; }
};

// UAV class with improved physics and control
class UAV {
private:
    Vec3 position;
    Vec3 velocity;
    Vec3 acceleration;
    double mass;
    double max_force_per_axis;  // Maximum force per axis
    double drag_coefficient;
    double gravity_compensation;
    
    // PID controllers for each axis
    PIDController pid_x;
    PIDController pid_y;
    PIDController pid_z;
    
    // Velocity controller (cascade control)
    PIDController pid_vx;
    PIDController pid_vy;
    PIDController pid_vz;
    
public:
    UAV(Vec3 initial_pos = Vec3(0, 0, 0), double mass = 1.0, double max_force = 30.0) 
        : position(initial_pos), velocity(0, 0, 0), acceleration(0, 0, 0),
          mass(mass), max_force_per_axis(max_force), drag_coefficient(0.05) {
        
        // Position PID controllers - generates desired velocity
        pid_x.setGains(4.0, 0.2, 2.0);   // P, I, D gains for X position
        pid_y.setGains(4.0, 0.2, 2.0);   // P, I, D gains for Y position
        pid_z.setGains(5.0, 0.3, 2.5);   // Higher gains for Z (altitude)
        
        // Velocity PID controllers - generates force commands
        pid_vx.setGains(3.0, 0.1, 0.5);  // X velocity control
        pid_vy.setGains(3.0, 0.1, 0.5);  // Y velocity control
        pid_vz.setGains(4.0, 0.2, 0.8);  // Z velocity control
        
        // Calculate gravity compensation
        gravity_compensation = 9.81 * mass;
    }
    
    // Calculate control forces using cascade PID (position -> velocity -> force)
    Vec3 calculateControlForces(const Vec3& target, double dt) {
        // Position error
        Vec3 pos_error = target - position;
        
        // Calculate desired velocity from position error (outer loop)
        double desired_vx = pid_x.calculate(pos_error.x, dt);
        double desired_vy = pid_y.calculate(pos_error.y, dt);
        double desired_vz = pid_z.calculate(pos_error.z, dt);
        
        // Limit desired velocity to reasonable values
        double max_velocity = 10.0;
        desired_vx = std::max(-max_velocity, std::min(max_velocity, desired_vx));
        desired_vy = std::max(-max_velocity, std::min(max_velocity, desired_vy));
        desired_vz = std::max(-max_velocity, std::min(max_velocity, desired_vz));
        
        // Velocity error
        double vel_error_x = desired_vx - velocity.x;
        double vel_error_y = desired_vy - velocity.y;
        double vel_error_z = desired_vz - velocity.z;
        
        // Calculate force from velocity error (inner loop)
        double force_x = pid_vx.calculate(vel_error_x, dt);
        double force_y = pid_vy.calculate(vel_error_y, dt);
        double force_z = pid_vz.calculate(vel_error_z, dt);
        
        // Add gravity compensation to Z force (feed-forward term)
        force_z += gravity_compensation;
        
        // Apply per-axis force limits
        force_x = std::max(-max_force_per_axis, std::min(max_force_per_axis, force_x));
        force_y = std::max(-max_force_per_axis, std::min(max_force_per_axis, force_y));
        force_z = std::max(-max_force_per_axis, std::min(max_force_per_axis, force_z));
        
        return Vec3(force_x, force_y, force_z);
    }
    
    // Alternative: Simple P-D controller with feed-forward for better stability
    Vec3 calculateSimpleControlForces(const Vec3& target, double dt) {
        Vec3 pos_error = target - position;
        
        // Position control with velocity damping
        double kp_pos = 5.0;
        double kd_vel = 3.0;
        
        Vec3 force;
        force.x = kp_pos * pos_error.x - kd_vel * velocity.x;
        force.y = kp_pos * pos_error.y - kd_vel * velocity.y;
        force.z = kp_pos * pos_error.z - kd_vel * velocity.z + gravity_compensation;
        
        // Apply force limits
        force.x = std::max(-max_force_per_axis, std::min(max_force_per_axis, force.x));
        force.y = std::max(-max_force_per_axis, std::min(max_force_per_axis, force.y));
        force.z = std::max(-max_force_per_axis, std::min(max_force_per_axis, force.z));
        
        return force;
    }
    
    // Update UAV physics
    void update(const Vec3& control_force, double dt) {
        // Calculate drag force (proportional to velocity squared for more realism)
        Vec3 drag;
        drag.x = -drag_coefficient * velocity.x * std::abs(velocity.x);
        drag.y = -drag_coefficient * velocity.y * std::abs(velocity.y);
        drag.z = -drag_coefficient * velocity.z * std::abs(velocity.z);
        
        // Gravity acts only on Z axis
        Vec3 gravity(0, 0, -9.81 * mass);
        
        // Total force
        Vec3 total_force = control_force + drag + gravity;
        
        // Newton's second law: F = ma
        acceleration = total_force / mass;
        
        // Update velocity and position using Euler integration
        velocity = velocity + acceleration * dt;
        position = position + velocity * dt;
        
        // Optional: Add ground constraint
        if (position.z < 0) {
            position.z = 0;
            if (velocity.z < 0) velocity.z = 0;
        }
    }
    
    // Getters
    Vec3 getPosition() const { return position; }
    Vec3 getVelocity() const { return velocity; }
    Vec3 getAcceleration() const { return acceleration; }
    
    // Reset all controllers
    void resetControllers() {
        pid_x.reset();
        pid_y.reset();
        pid_z.reset();
        pid_vx.reset();
        pid_vy.reset();
        pid_vz.reset();
    }
    
    // Set position (for testing)
    void setPosition(const Vec3& pos) {
        position = pos;
        velocity = Vec3(0, 0, 0);
        acceleration = Vec3(0, 0, 0);
        resetControllers();
    }
};

// Path manager for waypoint navigation
class PathManager {
private:
    std::vector<Vec3> waypoints;
    size_t current_waypoint_index;
    double waypoint_tolerance;
    double approach_speed_factor;
    
public:
    PathManager(double tolerance = 1.0) 
        : current_waypoint_index(0), waypoint_tolerance(tolerance),
          approach_speed_factor(1.0) {}
    
    void addWaypoint(const Vec3& point) {
        waypoints.push_back(point);
    }
    
    void addWaypoints(const std::vector<Vec3>& points) {
        waypoints.insert(waypoints.end(), points.begin(), points.end());
    }
    
    Vec3 getCurrentTarget() const {
        if (waypoints.empty()) return Vec3(0, 0, 0);
        return waypoints[current_waypoint_index];
    }
    
    bool updateTarget(const Vec3& current_position) {
        if (waypoints.empty()) return false;
        
        // Check if UAV reached current waypoint
        double distance = current_position.distance(waypoints[current_waypoint_index]);
        
        // Dynamic tolerance based on altitude (more lenient at higher altitudes)
        double dynamic_tolerance = waypoint_tolerance;
        if (waypoints[current_waypoint_index].z > 5) {
            dynamic_tolerance = waypoint_tolerance * 1.5;
        }
        
        if (distance < dynamic_tolerance) {
            std::cout << "Reached waypoint " << current_waypoint_index + 1 
                     << " (distance: " << distance << ")\n";
            current_waypoint_index++;
            if (current_waypoint_index >= waypoints.size()) {
                current_waypoint_index = 0;  // Loop back to start
                return true;  // Path completed
            }
        }
        return false;
    }
    
    bool hasWaypoints() const { return !waypoints.empty(); }
    size_t getCurrentIndex() const { return current_waypoint_index; }
    size_t getWaypointCount() const { return waypoints.size(); }
    void reset() { current_waypoint_index = 0; }
};

// Simulation class
class Simulation {
private:
    UAV uav;
    PathManager path_manager;
    double simulation_time;
    double dt;
    bool verbose;
    bool use_cascade_control;
    
public:
    Simulation(double timestep = 0.01, bool verbose = true, bool cascade = true) 
        : uav(Vec3(0, 0, 0)), simulation_time(0), dt(timestep), 
          verbose(verbose), use_cascade_control(cascade) {}
    
    void setupPath() {
        // Create a 3D path with multiple waypoints
        path_manager.addWaypoints({
            Vec3(0, 0, 5),      // Take off
            Vec3(10, 0, 5),     // Move forward
            Vec3(10, 10, 5),    // Move right
            Vec3(10, 10, 10),   // Climb
            Vec3(0, 10, 10),    // Move back
            Vec3(0, 0, 10),     // Complete square at altitude
            Vec3(0, 0, 0)       // Land
        });
    }
    
    void setupSimplePath() {
        // Simpler path for testing
        path_manager.addWaypoints({
            Vec3(0, 0, 2),      // Small takeoff
            Vec3(5, 0, 2),      // Move forward
            Vec3(5, 5, 2),      // Move right
            Vec3(0, 5, 2),      // Move back
            Vec3(0, 0, 2),      // Return to start
            Vec3(0, 0, 0)       // Land
        });
    }
    
    void run(double duration) {
        std::cout << "\n=== UAV PID Path Control Simulation ===\n";
        std::cout << "Control mode: " << (use_cascade_control ? "Cascade PID" : "Simple PD+FF") << "\n";
        std::cout << "Simulation duration: " << duration << " seconds\n";
        std::cout << "Time step: " << dt << " seconds\n\n";
        
        int display_counter = 0;
        int display_interval = 50;  // Display every 50 iterations (0.5 seconds)
        
        double min_error = 999999;
        double max_error = 0;
        double total_error = 0;
        int error_samples = 0;
        
        while (simulation_time < duration && path_manager.hasWaypoints()) {
            // Get current target waypoint
            Vec3 target = path_manager.getCurrentTarget();
            
            // Calculate control forces
            Vec3 control_force;
            if (use_cascade_control) {
                control_force = uav.calculateControlForces(target, dt);
            } else {
                control_force = uav.calculateSimpleControlForces(target, dt);
            }
            
            // Update UAV physics
            uav.update(control_force, dt);
            
            // Track error statistics
            double error = uav.getPosition().distance(target);
            min_error = std::min(min_error, error);
            max_error = std::max(max_error, error);
            total_error += error;
            error_samples++;
            
            // Check if waypoint reached and update target
            if (path_manager.updateTarget(uav.getPosition())) {
                uav.resetControllers();  // Reset PID controllers for new waypoint
                if (verbose) {
                    std::cout << "\n>>> Path completed! Restarting...\n\n";
                    path_manager.reset();
                }
            }
            
            // Display status periodically
            if (verbose && display_counter % display_interval == 0) {
                displayStatus(target, control_force);
            }
            
            simulation_time += dt;
            display_counter++;
        }
        
        std::cout << "\n=== Simulation Statistics ===\n";
        std::cout << "Minimum error: " << std::fixed << std::setprecision(3) << min_error << " m\n";
        std::cout << "Maximum error: " << std::fixed << std::setprecision(3) << max_error << " m\n";
        std::cout << "Average error: " << std::fixed << std::setprecision(3) 
                  << (total_error / error_samples) << " m\n";
        std::cout << "Final position: (" << uav.getPosition().x << ", " 
                  << uav.getPosition().y << ", " << uav.getPosition().z << ")\n";
    }
    
    void displayStatus(const Vec3& target, const Vec3& control_force) {
        Vec3 pos = uav.getPosition();
        Vec3 vel = uav.getVelocity();
        double error = pos.distance(target);
        
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "T:" << std::setw(5) << simulation_time << "s | ";
        std::cout << "WP:" << path_manager.getCurrentIndex() + 1 
                  << "/" << path_manager.getWaypointCount() << " | ";
        std::cout << "Pos:(" << std::setw(5) << pos.x << "," 
                  << std::setw(5) << pos.y << "," 
                  << std::setw(5) << pos.z << ") | ";
        std::cout << "Tgt:(" << std::setw(5) << target.x << "," 
                  << std::setw(5) << target.y << "," 
                  << std::setw(5) << target.z << ") | ";
        std::cout << "Err:" << std::setw(5) << error << "m | ";
        std::cout << "Vel:" << std::setw(5) << vel.magnitude() << "m/s | ";
        std::cout << "F:(" << std::setw(5) << control_force.x << ","
                  << std::setw(5) << control_force.y << ","
                  << std::setw(5) << control_force.z << ")\n";
    }
};

int main() {
    std::cout << "UAV PID Path Control System\n";
    std::cout << "===========================\n\n";
    
    // Test 1: Simple path with cascade control
    std::cout << "Test 1: Simple Path with Cascade PID Control\n";
    std::cout << "---------------------------------------------\n";
    Simulation sim1(0.01, true, true);
    sim1.setupSimplePath();
    sim1.run(30.0);
    
    std::cout << "\n\n";
    
    // Test 2: Complex path with simple PD control
    std::cout << "Test 2: Complex Path with Simple PD+FeedForward Control\n";
    std::cout << "-------------------------------------------------------\n";
    Simulation sim2(0.01, true, false);
    sim2.setupPath();
    sim2.run(40.0);
    
    std::cout << "\n=== Control System Notes ===\n";
    std::cout << "1. Cascade Control: Uses position->velocity->force cascade for smooth control\n";
    std::cout << "2. Simple PD+FF: Uses proportional-derivative with gravity feedforward\n";
    std::cout << "3. Gravity compensation is applied to maintain altitude\n";
    std::cout << "4. Force limits are applied per-axis for realistic behavior\n";
    std::cout << "5. Waypoint tolerance adapts based on altitude\n";
    
    return 0;
}