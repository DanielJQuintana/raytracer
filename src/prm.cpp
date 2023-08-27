/**
 * Implementation of 3 DOF robot PRM
 * @author Shiva Sreeram and Daniel Quintana
 * @date May 30, 2023
 */

#include <cstdlib>
#include <algorithm>
#include <iostream>
#include <cassert>
#include <string>
#include <sstream>
#include <random>
#include <cmath>
#include <vector>
#include <set>
#include <array>
#include <limits>
#include <numeric>
#include <iterator>
#include <tuple>
#include <utility>
#include <functional>

// #include "astar.hpp"
#include "vandercorput.hpp"
#include "planarutils.hpp"

const double N = 90; // Select the number of nodes
const double K = 10; // Select the number of nearest neighbors

const double pi = 3.14159265358979323846;

// World Definitions
const double xmin = -3.5;
const double xmax = 3.5;
const double ymin = -1.5;
const double ymax = 3.5;

const double ybottom = -0.5;
const double xleft = -0.5;
const double yleft = 1.5;
const double xright = 0.5;
const double yright = 1.0;

std::vector<double> wall1_init = { xmin, ybottom };
std::vector<double> wall1_final = { xmax, ybottom };
std::vector<std::vector<double>> wall1 = { wall1_init, wall1_final };

std::vector<double> wall2_init = { xmin, yleft };
std::vector<double> wall2_final = { xleft, yleft };
std::vector<std::vector<double>> wall2 = { wall2_init, wall2_final };

std::vector<double> wall3_init = { xleft, yleft };
std::vector<double> wall3_final = { xleft, ymax };
std::vector<std::vector<double>> wall3 = { wall3_init, wall3_final };

std::vector<double> wall4_init = { xright, yright };
std::vector<double> wall4_final = { xright, ymax };
std::vector<std::vector<double>> wall4 = { wall4_init, wall4_final };

std::vector<std::vector<std::vector<double>>> walls = { wall1, wall2, wall3, wall4 };

std::vector<double> xticks = {-3.5, -3, -2.5, -2, -1.5, -1, -0.5, 0, 0.5, 1, 1.5, 2, 2.5, 3, 3.5};
std::vector<double> yticks = {-1.5, -1, -0.5, 0, 0.5, 1, 1.5, 2, 2.5, 3, 3.5};

const double startq1 = 0.0;
const double startq2 = 0.0;
const double startq3 = 0.0;
const double goalq1 = pi / 2;
const double goalq2 = 0.0;
const double goalq3 = 0.0;

const double Dx = 0.1;
const double Dq = Dx / 6;
const bool wrap = true;

// Utilities
double wrap90(double angle) {
    return angle - pi * round(angle / pi);
}

double wrap180(double angle) {
    return angle - 2 * pi * round(angle / (2 * pi));
}

class Node {
public:
    double q1, q2, q3;
    double s1, s2, s3;
    double c1, c2, c3;
    std::vector<std::vector<std::vector<double>>> links;
    std::set<Node*> neighbors;
    bool done;
    bool seen;
    Node* parent;
    double creach;
    double ctogo;

    void reset() {
        done = false;
        seen = false;
        parent = NULL;
        creach = 0;
        ctogo = std::numeric_limits<double>::infinity();
    }

    bool operator<(const Node& other) const {
        return (creach + ctogo) < (other.creach + other.ctogo);
    }

    Node(double q1, double q2, double q3) : q1(q1), q2(q2), q3(q3) {
        reset();
        s1 = sin(q1);
        s2 = sin(q2);
        s3 = sin(q3);
        c1 = cos(q1);
        c2 = cos(q2);
        c3 = cos(q3);

        double xA = cos(q1);
        double yA = sin(q1);
        double xB = xA + cos(q1 + q2);
        double yB = yA + sin(q1 + q2);
        double xC = xB + cos(q1 + q2 + q3);
        double yC = yB + sin(q1 + q2 + q3);

        std::vector<double> link1_init = { 0, 0 };
        std::vector<double> link1_final = { xA, yA };
        std::vector<std::vector<double>> link1 = { link1_init, link1_final };

        std::vector<double> link2_init = { xA, yA };
        std::vector<double> link2_final = { xB, yB };
        std::vector<std::vector<double>> link2 = { link2_init, link2_final };

        std::vector<double> link3_init = { xB, yB };
        std::vector<double> link3_final = { xC, yC };
        std::vector<std::vector<double>> link3 = { link3_init, link3_final };



        // links.push_back(std::make_pair(std::make_pair(0, 0), std::make_pair(xA, yA)));
        // links.push_back(std::make_pair(std::make_pair(xA, yA), std::make_pair(xB, yB)));
        // links.push_back(std::make_pair(std::make_pair(xB, yB), std::make_pair(xC, yC)));

        links.push_back(link1);
        links.push_back(link2);
        links.push_back(link3);
    }

    double distance(const Node& other) const {
        if (wrap) {
            return sqrt((s1 - other.s1) * (s1 - other.s1) + (c1 - other.c1) * (c1 - other.c1) +
                        (s2 - other.s2) * (s2 - other.s2) + (c2 - other.c2) * (c2 - other.c2) +
                        (s3 - other.s3) * (s3 - other.s3) + (c3 - other.c3) * (c3 - other.c3));
        } else {
            return sqrt((q1 - other.q1) * (q1 - other.q1) +
                        (q2 - other.q2) * (q2 - other.q2) +
                        (q3 - other.q3) * (q3 - other.q3));
        }
    }

    // std::pair<double, double> coordinates() const {
    //     if (wrap) {
    //         return std::make_pair(s1, c1, s2, c2, s3, c3);
    //     } else {
    //         return std::make_pair(q1, q2, q3);
    //     }
    // }

    bool inFreespace() const {
        for (const auto& link : links) {
            for (const auto& wall : walls) {
                if (SegmentNearSegment(Dx, link, wall)) {
                    return false;
                }
            }
        }
        return true;
    }

    bool connectsTo(const Node& other) const {
        // std::cout << "Connects To Distance: " << distance(other) << "\n";
        for (double delta : sequence(Dq / distance(other))) {
            if (!intermediate(other, delta).inFreespace()) {
                return false;
            }
        }
        // std::cout << "Connects!\n";
        return true;
    }

    Node intermediate(const Node& other, double alpha) const {
        if (wrap) {
            return Node(q1 + alpha * wrap180(other.q1 - q1),
                        q2 + alpha * wrap180(other.q2 - q2),
                        q3 + alpha * wrap180(other.q3 - q3));
        } else {
            return Node(q1 + alpha * (other.q1 - q1),
                        q2 + alpha * (other.q2 - q2),
                        q3 + alpha * (other.q3 - q3));
        }
    }

    double costToConnect(Node* other) {
        return distance(*other);
    }

    double costToGoEst(Node* other) {
        return distance(*other);
    }
};

std::vector<Node*> createNodes(int N) {
    std::vector<Node*> nodes;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(-pi, pi);

    while (nodes.size() < N) {
        double q1 = dist(gen);
        double q2 = dist(gen);
        double q3 = dist(gen);
        // Node node(q1, q2, q3);
        Node* node = new Node(q1, q2, q3);
        if (node->inFreespace()) {
            nodes.push_back(node);
        }
    }
    return nodes;
}

void connectKNeighbors(std::vector<Node*>& nodes, int K) {
    for (Node* node : nodes) {
        node->neighbors.clear();
    }

    std::vector<std::array<double, 3>> coordinates;
    coordinates.reserve(nodes.size());
    for (const Node* node : nodes) {
        coordinates.push_back({ node->q1, node->q2, node->q3 });
    }

    std::vector<std::vector<double>> dist(nodes.size(), std::vector<double>(nodes.size()));
    for (int i = 0; i < nodes.size(); ++i) {
        for (int j = 0; j < nodes.size(); ++j) {
            double d = std::sqrt(std::pow(coordinates[i][0] - coordinates[j][0], 2) +
                                 std::pow(coordinates[i][1] - coordinates[j][1], 2) +
                                 std::pow(coordinates[i][2] - coordinates[j][2], 2));
            dist[i][j] = d;
        }
    }

    for (int i = 0; i < nodes.size(); ++i) {
        std::vector<int> indices(nodes.size());
        std::iota(indices.begin(), indices.end(), 0);
        std::sort(indices.begin(), indices.end(), [&](int a, int b) {
            return dist[i][a] < dist[i][b];
        });

        int count = 0;
        for (int j : indices) {
            if (count >= K) {
                break;
            }
            if (j != i && nodes[j]->neighbors.size() < K) {
                if (nodes[i]->connectsTo(*nodes[j])) {
                    // std::cout << "Connects!\n";
                    nodes[i]->neighbors.insert(nodes[j]);
                    nodes[j]->neighbors.insert(nodes[i]);
                    count++;
                }
            }
        }
    }
}


// Planner
std::vector<Node*> astar(const std::vector<Node*>& nodes, Node* start, Node* goal) {
    for (Node* node : nodes) {
        node->reset();
    }

    std::vector<Node*> onDeck;

    start->done = false;
    start->seen = true;
    start->parent = NULL;
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
                neighbor->parent = node;
                neighbor->creach = creach;
                neighbor->ctogo = neighbor->costToGoEst(goal);
                onDeck.push_back(neighbor);
                continue;
            }

            if (neighbor->creach <= creach) {
                continue;
            }

            neighbor->parent = node;
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
    // std::cout << "Goal Parent joints: " << goal->parent->q1 * 180 / pi << "deg, " << goal->parent->q2 * 180 / pi << "deg, " << goal->parent->q3 * 180 / pi << "deg\n";
    while (!path.front()->parent == NULL) {
        path.insert(path.begin(), path.front()->parent);
    }

    return path;
}

void postProcess(std::vector<Node*>& path) {
    if (path.size() > 0) {
        int i = 0;
        while (i < path.size() - 2) {
            if (path[i]->connectsTo(*path[i + 2])) {
                path.erase(path.begin() + i + 1);
            }
            else {
                i++;
            }
        }
    }
}

int main() {
    int N = 90; // Select the number of nodes
    int K = 10; // Select the number of nearest neighbors

    Node* start = new Node(startq1, startq2, startq3);
    Node* goal = new Node(goalq1,  goalq2,  goalq3);

    std::cout << "Create nodes \n";
    std::vector<Node*> nodes = createNodes(N);
    nodes.push_back(start);
    nodes.push_back(goal);
    std::cout << "Connect neighbors \n";
    connectKNeighbors(nodes, K);

    // // Usage example:
    // for (const Node& node : nodes) {
    //     std::cout << "Node (" << node.q1 << ", " << node.q2 << ", " << node.q3 << ") - Neighbors: ";
    //     for (const Node* neighbor : node.neighbors) {
    //         std::cout << "(" << neighbor->q1 << ", " << neighbor->q2 << ", " << neighbor->q3 << ") ";
    //     }
    //     std::cout << std::endl;
    // }

    // std::vector<Node*> path_nodes;
    // // Node* temp_nodes = (Node*)malloc(sizeof(Node) * nodes.size());
    // for (uint i = 0; i < nodes.size(); i++) {
    //     // temp_nodes[i] = &Node(nodes[i].q1, nodes[i].q2, nodes[i].q3);
    //     path_nodes.push_back(new Node(nodes[i].q1, nodes[i].q2, nodes[i].q3));
    //     path_nodes[i]->neighbors = nodes[i].neighbors;
    // } 

    // Node* start = new Node(startq1, startq2, startq3);
    // Node* goal = new Node(goalq1,  goalq2,  goalq3);
    // Node* start = (Node*)malloc(sizeof(Node));
    // start = &Node(startq1, startq2, startq3);
    // Node* goal = (Node*)malloc(sizeof(Node));
    // goal = &Node(goalq1,  goalq2,  goalq3);

    // std::cout << "Nodes size: " << nodes.size() << "\n";
    // for (uint i = 0; i < nodes.size(); i++) {
    //     std::cout << "Node " << i << " neighbor count: " << nodes[i]->neighbors.size() << "\n";
    // }

    // std::cout << "Goal neighbors:\n";
    // for (Node * neighbor : nodes[N+1]->neighbors) {
    //     std::cout << "Neighbor joints: " << neighbor->q1 * 180 / pi << "deg, " << neighbor->q2 * 180 / pi << "deg, " << neighbor->q3 * 180 / pi << "deg\n";
    // }

    std::cout << "Plan path \n";
    std::vector<Node*> path = astar(nodes, nodes[N], nodes[N+1]);
    
    std::cout << "Post Process path \n";
    postProcess(path);
    std::cout << "Showing path \n";
    std::cout << "Path size: " << path.size() << "\n";
    for(unsigned int i = 0; i < path.size(); i++)
    {
        std::cout << "Joints " << path[i]->q1 * 180 / pi << "deg, " << path[i]->q2 * 180 / pi << "deg, " << path[i]->q3 * 180 / pi << "deg\n";
    }

    // Destruction
    // for (uint i = 0; i < nodes.size(); i++) path_nodes[i].~Node(); 
    // free(temp_nodes);
    // free(start);
    // free(goal);


    return 0;
}
