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
#include <fstream>

// #include "astar.hpp"
#include "vandercorput.hpp"
#include "planarutils.hpp"

const double N = 500; // Select the number of nodes
const double K = 100; // Select the number of nearest neighbors

const double pi = 3.14159265358979323846;

// World Definitions
// Spheres represented as (x, y, z, radius)
std::vector<double> sphere1 = { 0.0, 1.5, 1.5, 1.0 };
std::vector<double> sphere2 = { 0.0, 1.5, -1.5, 1.0 };
std::vector<double> sphere3 = { -1.5, 0.0, 0.0, 1.0 };
std::vector<double> sphere4 = { 0.0, -1.5, 0.0, 1.0 };

std::vector<std::vector<double>> spheres = { sphere1, sphere2, sphere3, sphere4 };


std::vector<double> xticks = {-3.5, -3, -2.5, -2, -1.5, -1, -0.5, 0, 0.5, 1, 1.5, 2, 2.5, 3, 3.5};
std::vector<double> yticks = {-1.5, -1, -0.5, 0, 0.5, 1, 1.5, 2, 2.5, 3, 3.5};

const double startq1 = 0.0;
const double startq2 = 0.0;
const double startq3 = 0.0;
const double goalq1 = pi / 2;
const double goalq2 = pi / 2;
const double goalq3 = 0.0;

const double Dx = 0.1;
const double Dq = Dx / 6;
// const bool wrap = true;

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
        parent = nullptr;
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

        double xA = -sin(q1) * cos(q2);
        double yA = cos(q1) * cos(q2);
        double zA = sin(q2);
        double xB = xA - sin(q1) * cos(q2 + q3);
        double yB = yA + cos(q1) * cos(q2 + q3);
        double zB = zA + sin(q2 + q3);


        std::vector<double> link1_init = { 0, 0, 0 };
        std::vector<double> link1_final = { xA, yA, zA };
        std::vector<std::vector<double>> link1 = { link1_init, link1_final };

        std::vector<double> link2_init = { xA, yA, zA };
        std::vector<double> link2_final = { xB, yB, zB };
        std::vector<std::vector<double>> link2 = { link2_init, link2_final };

        links.push_back(link1);
        links.push_back(link2);
    }

    double distance(const Node& other) const {
        return sqrt((q1 - other.q1) * (q1 - other.q1) +
                    (q2 - other.q2) * (q2 - other.q2) +
                    (q3 - other.q3) * (q3 - other.q3));
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
            for (const auto& sphere : spheres) {
                if (SegmentNearSphere(Dx, link, sphere)) {
                    return false;
                }
            }
        }
        return true;
    }

    bool connectsTo(const Node& other) const {
        for (double delta : sequence(Dq / distance(other))) {
            if (!intermediate(other, delta).inFreespace()) {
                return false;
            }
        }
        return true;
    }

    Node intermediate(const Node& other, double alpha) const {
        return Node(q1 + alpha * (other.q1 - q1),
                    q2 + alpha * (other.q2 - q2),
                    q3 + alpha * (other.q3 - q3));
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

    while ((int)nodes.size() < N) {
        double q1 = dist(gen);
        double q2 = dist(gen);
        double q3 = dist(gen);
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
    for (int i = 0; i < (int)nodes.size(); ++i) {
        for (int j = 0; j < (int)nodes.size(); ++j) {
            double d = std::sqrt(std::pow(coordinates[i][0] - coordinates[j][0], 2) +
                                 std::pow(coordinates[i][1] - coordinates[j][1], 2) +
                                 std::pow(coordinates[i][2] - coordinates[j][2], 2));
            dist[i][j] = d;
        }
    }

    for (int i = 0; i < (int)nodes.size(); ++i) {
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
            if (j != i && (int)nodes[j]->neighbors.size() < K) {
                if (nodes[i]->connectsTo(*nodes[j])) {
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
    start->parent = nullptr;
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
    while (path.front()->parent) {
        path.insert(path.begin(), path.front()->parent);
    }

    return path;
}

void postProcess(std::vector<Node*>& path) {
    if (path.size() > 0) {
        int i = 0;
        while (i < (int)path.size() - 2) {
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

    Node* start = new Node(startq1, startq2, startq3);
    Node* goal = new Node(goalq1,  goalq2,  goalq3);

    std::cout << "Create nodes \n";
    std::vector<Node*> nodes = createNodes(N);
    nodes.push_back(start);
    nodes.push_back(goal);
    std::cout << "Connect neighbors \n";
    connectKNeighbors(nodes, K);

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

    std::cout << "\nShowing path with intermediate nodes \n";
    std::vector<Node> path_nodes_intermediates;
    for(unsigned int i = 0; i < path.size() - 1; i++)
    {
        double n = 25 * ceil(path[i]->distance(*path[i+1]) / 0.5);
        for (unsigned int j = 0; j < n; j++)
        {
            path_nodes_intermediates.push_back(path[i]->intermediate(*path[i+1], j/n));
        }
    }
    path_nodes_intermediates.push_back(Node(path[path.size() - 1]->q1, 
                                            path[path.size() - 1]->q2, 
                                            path[path.size() - 1]->q3));

    for(unsigned int i = 0; i < path_nodes_intermediates.size(); i++)
    {
        std::cout << "Joints " << path_nodes_intermediates[i].q1 * 180 / pi << "deg, " << path_nodes_intermediates[i].q2 * 180 / pi << "deg, " << path_nodes_intermediates[i].q3 * 180 / pi << "deg\n";
    }

    std::cout << "\nShowing link locations of intermediate nodes \n";
    for(unsigned int i = 0; i < path_nodes_intermediates.size(); i++)
    {
        std::vector<std::vector<std::vector<double>>> links = path_nodes_intermediates[i].links;
        std::vector<std::vector<double>> link1 = links[0];
        std::vector<std::vector<double>> link2 = links[1];

        std::cout << "Link 1: (" << link1[0][0] << ", " << link1[0][1] << ", " << link1[0][2] << ") -> (" << link1[1][0] << ", " << link1[1][1] << ", " << link1[1][2] << ") Link 2: (" << link2[0][0] << ", " << link2[0][1] << ", " << link2[0][2] << ") -> (" << link2[1][0] << ", " << link2[1][1] << ", " << link2[1][2] << ")\n";
    }

    std::ofstream myfile;
    myfile.open ("prm_test_cpu_output.txt");
    myfile << 2 << "; " << path_nodes_intermediates.size() << "\n";
    for(unsigned int i = 0; i < path_nodes_intermediates.size(); i++)
    {
        std::vector<std::vector<std::vector<double>>> links = path_nodes_intermediates[i].links;
        std::vector<std::vector<double>> link1 = links[0];
        std::vector<std::vector<double>> link2 = links[1];

        myfile << link1[0][0] << ", " << link1[0][1] << ", " << link1[0][2] << ",; " << link1[1][0] << ", " << link1[1][1] << ", " << link1[1][2] << ",; " << link2[1][0] << ", " << link2[1][1] << ", " << link2[1][2] << ",;\n";
    }
    myfile.close();


    return 0;
}
