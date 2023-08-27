#include <iostream>
#include <vector>
#include <set>
#include <algorithm>
#include <cmath>

#include "astar.hpp"

// class AStarNode {
// public:
//     std::set<AStarNode*> neighbors;
//     bool done;
//     bool seen;
//     std::vector<AStarNode*> parent;
//     double creach;
//     double ctogo;

//     AStarNode() {
//         reset();
//     }

//     void reset() {
//         done = false;
//         seen = false;
//         parent.clear();
//         creach = 0;
//         ctogo = std::numeric_limits<double>::infinity();
//     }

//     bool operator<(const AStarNode& other) const {
//         return (creach + ctogo) < (other.creach + other.ctogo);
//     }
// };

std::vector<Node*> astar(const std::vector<Node*>& nodes, Node* start, Node* goal) {
    for (Node* node : nodes) {
        node->reset();
    }

    std::vector<Node*> onDeck;

    start->done = false;
    start->seen = true;
    start->parent = std::vector<Node*>();
    start->creach = 0;
    start->ctogo = start->costToGoEst(goal);
    onDeck.push_back(start);

    while (true) {
        Node* node = onDeck.front();
        onDeck.erase(onDeck.begin());

        for (Node* neighbor : node->neighbors) {
            if (neighbor->done) {
                continue;
            }

            double creach = node->creach + node->costToConnect(neighbor);

            if (!neighbor->seen) {
                neighbor->seen = true;
                neighbor->parent = node->parent;
                neighbor->creach = creach;
                neighbor->ctogo = neighbor->costToGoEst(goal);
                onDeck.push_back(neighbor);
                continue;
            }

            if (neighbor->creach <= creach) {
                continue;
            }

            neighbor->parent = node->parent;
            neighbor->creach = creach;
            onDeck.erase(std::remove(onDeck.begin(), onDeck.end(), neighbor), onDeck.end());
            onDeck.push_back(neighbor);
        }

        node->done = true;

        if (goal->done) {
            break;
        }

        if (onDeck.empty()) {
            return std::vector<Node*>();
        }
    }

    std::vector<Node*> path;
    path.push_back(goal);
    while (!path.front()->parent.empty()) {
        path.insert(path.begin(), path.front()->parent.front());
    }

    return path;
}

// class Node : public AStarNode {
// public:
//     double costToConnect(Node* other) {
//         // Define your own distance metric here
//         return 0.0;
//     }

//     double costToGoEst(Node* other) {
//         // Define your own distance metric here
//         return 0.0;
//     }
// };

// int main() {
//     std::vector<Node*> nodes;
//     Node* startNode = new Node();
//     Node* goalNode = new Node();

//     std::vector<Node*> path = astar(nodes, startNode, goalNode);

//     // Process the path

//     // Clean up
//     delete startNode;
//     delete goalNode;
//     for (Node* node : nodes) {
//         delete node;
//     }

//     return 0;
// }
