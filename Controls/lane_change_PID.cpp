#include <iostream>
#include <vector>
#include <cmath>

// Vehicle parameters
const double L = 2.5; // Wheelbase (meters)

// PD Controller parameters
const double Kp_lateral = 0.5; // Proportional gain for lateral control
const double Kd_lateral = 0.2; // Derivative gain for lateral control

const double Kp_longitudinal = 0.5; // Proportional gain for longitudinal control
const double Kd_longitudinal = 0.2; // Derivative gain for longitudinal control

class PathFollowingController {
public:
    // Constructor
    PathFollowingController(std::vector<double>& trajectory) : trajectory_(trajectory), targetIndex_(0) {}

    // Control function to compute steering angle and throttle
    std::pair<double, double> control(double currentX, double currentY, double currentHeading, double currentVelocity) {
        // Find the nearest point on the trajectory
        updateTargetIndex(currentX, currentY);

        // Calculate cross-track error (lateral error)
        double cte = calculateCrossTrackError(currentX, currentY, currentHeading);

        // Compute desired steering angle using lateral control (PD controller)
        double steeringAngle = computeSteeringAngle(cte, currentVelocity);

        // Compute desired throttle using longitudinal control (PD controller)
        double throttle = computeThrottle(currentVelocity);

        return {steeringAngle, throttle};
    }

private:
    std::vector<double>& trajectory_; // Predefined trajectory
    size_t targetIndex_; // Index of the target point on the trajectory

    // Update the target index based on current vehicle position
    void updateTargetIndex(double currentX, double currentY) {
        double minDistance = std::numeric_limits<double>::max();
        size_t minIndex = 0;

        // Find the nearest point on the trajectory to the current position
        for (size_t i = targetIndex_; i < trajectory_.size(); ++i) {
            double dx = trajectory_[i] - currentX;
            double dy = trajectory_[i + 1] - currentY;
            double distance = std::sqrt(dx * dx + dy * dy);

            if (distance < minDistance) {
                minDistance = distance;
                minIndex = i;
            }
        }

        targetIndex_ = minIndex;
    }

    // Calculate cross-track error (lateral error)
    double calculateCrossTrackError(double currentX, double currentY, double currentHeading) {
        double dx = trajectory_[targetIndex_] - currentX;
        double dy = trajectory_[targetIndex_ + 1] - currentY;

        // Project the position onto the trajectory
        double projection = dx * std::cos(currentHeading) + dy * std::sin(currentHeading);

        // Calculate cross-track error (distance between the vehicle and the trajectory)
        double cte = -dx * std::sin(currentHeading) + dy * std::cos(currentHeading);

        return cte;
    }

    // Compute desired steering angle using lateral control (PD controller)
    double computeSteeringAngle(double cte, double currentVelocity) {
        // Compute proportional and derivative terms
        double proportionalTerm = Kp_lateral * cte;
        double derivativeTerm = Kd_lateral * (cte - previousCTE_);
        previousCTE_ = cte;

        // Compute steering angle
        double steeringAngle = proportionalTerm + derivativeTerm;

        return steeringAngle;
    }

    // Compute desired throttle using longitudinal control (PD controller)
    double computeThrottle(double currentVelocity) {
        // Compute error (desired speed - current speed)
        double velocityError = MAX_VELOCITY - currentVelocity;

        // Compute proportional and derivative terms
        double proportionalTerm = Kp_longitudinal * velocityError;
        double derivativeTerm = Kd_longitudinal * (velocityError - previousVelocityError_);
        previousVelocityError_ = velocityError;

        // Compute throttle
        double throttle = proportionalTerm + derivativeTerm;

        return throttle;
    }

    // Variables for storing previous errors
    double previousCTE_ = 0.0;
    double previousVelocityError_ = 0.0;

    // Maximum velocity
    const double MAX_VELOCITY = 20.0; // Maximum velocity (m/s)
};

int main() {
    // Define the predefined trajectory (for simplicity, assume a straight trajectory)
    std::vector<double> trajectory = {0.0, 0.0, 10.0, 10.0};

    // Create the path following controller
    PathFollowingController controller(trajectory);

    // Simulate the vehicle's movement along the trajectory
    double currentX = 0.0; // Initial X position
    double currentY = 0.0; // Initial Y position
    double currentHeading = 0.0; // Initial heading angle (radians)
    double currentVelocity = 5.0; // Initial velocity (m/s)

    // Simulate for a certain number of time steps
    for (int i = 0; i < 100; ++i) {
        // Compute control inputs (steering angle and throttle)
        auto controlInputs = controller.control(currentX, currentY, currentHeading, currentVelocity);
        double steeringAngle = controlInputs.first;
        double throttle = controlInputs.second;

        // Update vehicle state
        currentX += currentVelocity * std::cos(currentHeading);
        currentY += currentVelocity * std::sin(currentHeading);
        currentHeading += std::tan(steeringAngle) * currentVelocity / L; // Update heading angle (Ackermann steering model)
        currentVelocity += throttle;

        // Print vehicle state
        std::cout << "Step " << i << ": X = " << currentX << ", Y = " << currentY << ", Heading = " << currentHeading
                  << ", Velocity = " << currentVelocity << std::endl;
    }

    return 0;
}
