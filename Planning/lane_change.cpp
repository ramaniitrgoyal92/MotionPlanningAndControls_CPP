#include <iostream>
#include <vector>
#include <cmath>

// Simple kinematic model parameters
const double MAX_ACCELERATION = 2.0; // Maximum acceleration (m/s^2)
const double MAX_VELOCITY = 20.0;    // Maximum velocity (m/s)
const double TIME_STEP = 0.1;        // Time step (s)

// Polynomial trajectory planner
class PolynomialTrajectoryPlanner {
public:
    // Generate a smooth lane change trajectory
    std::vector<double> generateLaneChangeTrajectory(double initialPosition, double initialVelocity, double targetLane) {
        std::vector<double> trajectory;
        double currentPosition = initialPosition;
        double currentVelocity = initialVelocity;
        double acceleration = 0.0;

        while (currentPosition < targetLane) {
            // Calculate acceleration to reach target lane
            acceleration = calculateAcceleration(currentPosition, targetLane, currentVelocity);
            
            // Update velocity and position using kinematic equations
            currentVelocity += acceleration * TIME_STEP;
            currentPosition += currentVelocity * TIME_STEP + 0.5 * acceleration * TIME_STEP * TIME_STEP;

            // Add position to trajectory
            trajectory.push_back(currentPosition);
        }

        return trajectory;
    }

private:
    // Calculate acceleration to reach target lane
    double calculateAcceleration(double currentPosition, double targetLane, double currentVelocity) {
        double distanceToTarget = targetLane - currentPosition;
        double maxVelocity = std::min(MAX_VELOCITY, std::sqrt(2 * MAX_ACCELERATION * std::abs(distanceToTarget)));

        // Apply acceleration limits
        double desiredAcceleration = (maxVelocity - currentVelocity) / TIME_STEP;
        return std::max(-MAX_ACCELERATION, std::min(MAX_ACCELERATION, desiredAcceleration));
    }
};

int main() {
    // Initial conditions
    double initialPosition = 0.0;    // Initial position (lane)
    double initialVelocity = 10.0;   // Initial velocity (m/s)
    double targetLane = 3.0;         // Target lane for lane change

    // Generate lane change trajectory
    PolynomialTrajectoryPlanner planner;
    std::vector<double> trajectory = planner.generateLaneChangeTrajectory(initialPosition, initialVelocity, targetLane);

    // Print trajectory
    std::cout << "Lane Change Trajectory:" << std::endl;
    for (double position : trajectory) {
        std::cout << position << std::endl;
    }

    return 0;
}
