#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <limits>

// Define a structure to represent a state in the search space
struct State {
    int lane; // Lane number
    double cost; // Cost associated with reaching this state
    double heuristic; // Heuristic value (estimated cost to reach the goal)
    State* parent; // Pointer to the parent state

    State(int lane, double cost, double heuristic, State* parent)
        : lane(lane), cost(cost), heuristic(heuristic), parent(parent) {}
};

// Custom comparator for priority queue (min-heap) based on total cost
struct CompareState {
    bool operator()(const State* s1, const State* s2) const {
        return s1->cost + s1->heuristic > s2->cost + s2->heuristic;
    }
};

// A* algorithm for vehicle lane change
std::vector<int> AStarLaneChange(int startLane, int targetLane, std::vector<std::vector<double>>& laneCosts) {
    // Initialize priority queue (open set) and visited set
    std::priority_queue<State*, std::vector<State*>, CompareState> openSet;
    // std::vector<std::vector<bool>> visited(laneCosts.size(), std::vector<bool>(laneCosts[0].size(), false));
    std::vector<bool> visited(laneCosts.size(), false);

    // Initialize start state
    double heuristic = std::abs(targetLane - startLane); // Heuristic: Manhattan distance to the target lane
    State* startState = new State(startLane, 0.0, heuristic, nullptr);
    openSet.push(startState);

    // A* search
    while (!openSet.empty()) {
        // Get the state with the lowest total cost from the open set
        State* currentState = openSet.top();
        openSet.pop();

        // Check if the current state is the goal state
        if (currentState->lane == targetLane) {
            // Reconstruct and return the optimal path
            std::vector<int> path;
            while (currentState != nullptr) {
                path.push_back(currentState->lane);
                currentState = currentState->parent;
            }
            // std::reverse(path.begin(), path.end()); // Reverse the path to get the correct order
            return path;
        }

        // Mark the current state as visited
        // visited[currentState->lane][currentState->lane] = true;
        visited[currentState->lane] = true;

        // Generate successor states (possible lane changes)
        for (int i = 0; i < laneCosts.size(); ++i) {
            if (i != currentState->lane && !visited[i]) {
                double cost = currentState->cost + laneCosts[currentState->lane][i];
                double heuristic = std::abs(targetLane - i);
                State* nextState = new State(i, cost, heuristic, currentState);
                openSet.push(nextState);
            }
        }
    }

    // If no path is found, return an empty path
    return {};
}

int main() {
    // Example: Lane change costs (adjacency matrix)
    std::vector<std::vector<double>> laneCosts = {
        {0.0, 1.0, 2.0}, // Lane 0 (costs to change to lanes 0, 1, 2)
        {1.0, 0.0, 1.5}, // Lane 1 (costs to change to lanes 0, 1, 2)
        {2.0, 1.5, 0.0}  // Lane 2 (costs to change to lanes 0, 1, 2)
    };

    // Example: Perform lane change from lane 0 to lane 2
    int startLane = 0;
    int targetLane = 2;
    std::vector<int> optimalPath = AStarLaneChange(startLane, targetLane, laneCosts);

    // Output the optimal path
    std::cout << "Optimal path for lane change from lane " << startLane << " to lane " << targetLane << ":" << std::endl;
    for (int lane : optimalPath) {
        std::cout << lane << " ";
    }
    std::cout << std::endl;

    return 0;
}
