#include <iostream>
#include <cmath>

// Represents the dynamic state of the vehicle
struct State {
    double x = 0.0;            // Position of X in meters
    double y = 0.0;            // Position of Y in meters
    double heading = 0.0;      // Orientation angle in radians
    double velocity = 0.0;     // Velocity in m/s
};

// Represents a target waypoint on the road or trajectory.
struct TargetPoint {
    double x;
    double y;
};

void updateState(State& state, double steering_angle, double acceleration, double dt, double wheelbase = 2.5) {
    // Update velocity with acceleration
    state.velocity += acceleration * dt;

    // Update position based on current velocity and heading
    state.x += state.velocity * std::cos(state.heading) * dt;
    state.y += state.velocity * std::sin(state.heading) * dt;

    // Update heading based on steering angle and wheelbase geometry
    state.heading += (state.velocity / wheelbase) * std::tan(steering_angle) * dt;

    // Normalize heading to the range [-pi, pi] to prevent angle overflow
    state.heading = std::atan2(std::sin(state.heading), std::cos(state.heading));
}

// Computes spatial error metrics relative to a target point
void computeTrajectoryError(const State& state, const TargetPoint& target, double& out_distance, double& out_heading_offset) {
    // Vector components from vehicle to target
    double dx = target.x - state.x;
    double dy = target.y - state.y;

    // Euclidean distance using hypotenuse utility
    out_distance = std::hypot(dx, dy);

    // Absolute angle from vehicle position to target position
    double angle_to_target = std::atan2(dy, dx);

    // Difference between target direction and current vehicle heading
    out_heading_offset = angle_to_target - state.heading;

    // Normalize error angle to [-pi, pi]
    out_heading_offset = std::atan2(std::sin(out_heading_offset), std::cos(out_heading_offset));
}

int main() {
    // Initialize vehicle starting at origin going 10 m/s straight
    State vehicle;
    vehicle.velocity = 10.0;

    // Define a target waypoint ahead
    TargetPoint target{40.0, 15.0};

    double dt = 0.1; // 100 ms control loop timestep

    // Simulate 5 control steps
    for (int step = 1; step <= 5; ++step) {
        double steering_command = 0.02; // slight steering curve input (radians)
        double acceleration = 0.0;      // constant speed

        updateState(vehicle, steering_command, acceleration, dt);

        double distance_error, heading_error;
        computeTrajectoryError(vehicle, target, distance_error, heading_error);

        std::cout << "Step " << step << " -> "
                  << "x: " << vehicle.x << "m, "
                  << "y: " << vehicle.y << "m | "
                  << "Dist to Target: " << distance_error << "m, "
                  << "Heading Err: " << heading_error << " rad\n";
    }

    return 0;
}