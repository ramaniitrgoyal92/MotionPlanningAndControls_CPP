#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <limits>

// Define a structure to represent a node in the RRT tree
struct Node {
    int lane; // Lane number
    Node* parent; // Pointer to the parent node
    double x, y; // Coordinates (for visualization)

    Node(int lane, double x, double y, Node* parent)
        : lane(lane), x(x), y(y), parent(parent) {}
};

// Function to generate a random node within the lane space
Node* generateRandomNode(int numLanes, double laneWidth) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> laneDist(0, numLanes - 1);
    // std::uniform_int_distribution<int> laneDist(0, numLanes - 1);
    std::uniform_real_distribution<double> widthDist(0, laneWidth);
    int lane = static_cast<int>(laneDist(gen));
    double x = 0.0; // Assuming vehicle starts at the leftmost edge of the lane
    double y = lane * laneWidth + widthDist(gen);
    return new Node(lane, x, y, nullptr);
}

// Function to find the nearest node in the tree to a given node
Node* findNearestNode(const std::vector<Node*>& tree, const Node* newNode) {
    double minDist = std::numeric_limits<double>::max();
    Node* nearestNode = nullptr;
    for (Node* node : tree) {
        double dist = std::sqrt(std::pow(node->x - newNode->x, 2) + std::pow(node->y - newNode->y, 2));
        if (dist < minDist) {
            minDist = dist;
            nearestNode = node;
        }
    }
    return nearestNode;
}

// Function to check if two nodes are connected without collision
bool canConnect(const Node* nearestNode, const Node* newNode) {
    // Here you would implement collision checking logic
    // For simplicity, we assume all connections are valid
    return true;
}

// RRT algorithm for vehicle lane change
std::vector<Node*> RRTLaneChange(int numLanes, double laneWidth, int maxIterations) {
    // Initialize tree with a single node at the start position
    std::vector<Node*> tree;
    tree.push_back(new Node(0, 0.0, laneWidth / 2.0, nullptr));

    // Main RRT loop
    for (int i = 0; i < maxIterations; ++i) {
        // Generate a random node within the lane space
        Node* randomNode = generateRandomNode(numLanes, laneWidth);

        // Find the nearest node in the tree to the random node
        Node* nearestNode = findNearestNode(tree, randomNode);

        // Steer towards the random node
        // Here, you could implement a more sophisticated steering method (e.g., Dubins path)
        double dx = randomNode->x - nearestNode->x;
        double dy = randomNode->y - nearestNode->y;
        double theta = std::atan2(dy, dx);
        double newX = nearestNode->x + 0.1 * std::cos(theta); // Step size of 0.1 for simplicity
        double newY = nearestNode->y + 0.1 * std::sin(theta);

        // Create a new node
        Node* newNode = new Node(nearestNode->lane, newX, newY, nearestNode);

        // Check if the new node can be connected to the tree without collision
        if (canConnect(nearestNode, newNode)) {
            // Add the new node to the tree
            tree.push_back(newNode);

            // Check if the new node reaches the goal lane
            if (newNode->lane == numLanes - 1) {
                return tree; // Goal reached, return the tree
            }
        }
    }
    // Return the tree after reaching the maximum number of iterations
    return tree;
}

int main() {
    // Example: Parameters
    int numLanes = 3; // Number of lanes
    double laneWidth = 3.0; // Width of each lane
    int maxIterations = 1000; // Maximum number of iterations for RRT

    // Run RRT algorithm for lane change
    std::vector<Node*> tree = RRTLaneChange(numLanes, laneWidth, maxIterations);

    // Output the resulting tree (for visualization or further processing)
    std::cout << "Number of nodes in the tree: " << tree.size() << std::endl;
    for (const Node* node : tree) {
        std::cout << "Lane: " << node->lane << ", X: " << node->x << ", Y: " << node->y << std::endl;
    }

    return 0;
}
