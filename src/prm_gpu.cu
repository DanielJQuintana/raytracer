/**
 * Implementation of 3 DOF robot PRM on GPU
 * @author Shiva Sreeram and Daniel Quintana
 * @date June 7, 2023
 */

#include <cstdlib>
#include <algorithm>
#include <iostream>
#include <cassert>
#include <string>
#include <cmath>
#include <vector>
#include <limits>
#include <numeric>
#include <iterator>
#include <fstream>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <stdio.h>
#include <curand.h>

#include "helper_cuda.h"

const int N = 500; // Select the number of nodes
const int K = 100; // Select the number of nearest neighbors
const int NUM_RANDS = 20; // Metric for the amount of random numbers to generate

const double pi = 3.14159265358979323846;

// World Definitions
// Spheres represented as (x, y, z, radius)
const int NUM_OBSTACLES = 4;
std::vector<double> sphere1 = { 0.0, 1.5, 1.5, 1.0 };
std::vector<double> sphere2 = { 0.0, 1.5, -1.5, 1.0 };
std::vector<double> sphere3 = { -1.5, 0.0, 0.0, 1.0 };
std::vector<double> sphere4 = { 0.0, -1.5, 0.0, 1.0 };

std::vector<std::vector<double>> spheres = { sphere1, sphere2, sphere3, sphere4 };

const double startq1 = 0.0;
const double startq2 = 0.0;
const double startq3 = 0.0;
const double goalq1 = pi / 2;
const double goalq2 = pi / 2;
const double goalq3 = 0.0;

const double Dx = 0.1;
const double Dq = Dx / 6.0;

class Node {
public:
    double q1, q2, q3;
    double s1, s2, s3;
    double c1, c2, c3;
    int* neighbors;
    int total_neighbors;
    bool done;
    bool seen;
    int parent;
    double creach;
    double ctogo;
    double ctotal;
    int id;

    void reset() {
        done = false;
        seen = false;
        parent = -1;
        creach = 0;
        ctogo = std::numeric_limits<double>::infinity();
        ctotal = creach + ctogo;
    }

    Node() {
        q1 = 0;
        q2 = 0;
        q3 = 0;
        s1 = 0;
        s2 = 0;
        s3 = 0;
        c1 = 0;
        c2 = 0;
        c3 = 0;
        neighbors = nullptr;
        total_neighbors = 0;
        done = false;
        seen = false;
        parent = -1;
        creach = 0;
        ctogo = std::numeric_limits<double>::infinity();
        ctotal = creach + ctogo;
        id = 0;
    }

    Node(double q1, double q2, double q3) : q1(q1), q2(q2), q3(q3) {
        reset();
        id = 0;
        total_neighbors = 0;
        neighbors = (int*)malloc((N + 2) * sizeof(int));
        s1 = sin(q1);
        s2 = sin(q2);
        s3 = sin(q3);
        c1 = cos(q1);
        c2 = cos(q2);
        c3 = cos(q3);
    }

    double distance(const Node& other) const {
        return sqrt((q1 - other.q1) * (q1 - other.q1) +
                    (q2 - other.q2) * (q2 - other.q2) +
                    (q3 - other.q3) * (q3 - other.q3));
    }

    Node intermediate(const Node& other, double alpha) const {
        return Node(q1 + alpha * (other.q1 - q1),
                    q2 + alpha * (other.q2 - q2),
                    q3 + alpha * (other.q3 - q3));
    }
};

// Function to obtain the links of a Node to be used when generating the text file
std::vector<std::vector<std::vector<double>>> get_links(Node node) {
    std::vector<std::vector<std::vector<double>>> links;
    double q1 = node.q1;
    double q2 = node.q2;
    double q3 = node.q3;
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

    return links;
}

// Device function for checking if a segment is near a sphere
__device__ bool device_segment_near_sphere(double d, double startX, double startY, double startZ, 
                                           double endX, double endY, double endZ, 
                                           double sphereX, double sphereY, double sphereZ, double sphereRadius) {

    // Calculate the direction vector of the segment
    double segmentDirectionX = endX - startX;
    double segmentDirectionY = endY - startY;
    double segmentDirectionZ = endZ - startZ;

    // Calculate the vector from the segment's start point to the sphere's center
    double vectorX = sphereX - startX;
    double vectorY = sphereY - startY;
    double vectorZ = sphereZ - startZ;

    // Calculate the projection of the vector onto the segment's direction
    double projection = (vectorX * segmentDirectionX) + (vectorY * segmentDirectionY) + (vectorZ * segmentDirectionZ);

    if (projection <= 0.0) {
        // The sphere is behind the segment's start point
        return false;
    }

    // Calculate the length of the segment
    double segmentLength = sqrt(
        segmentDirectionX * segmentDirectionX +
        segmentDirectionY * segmentDirectionY+
        segmentDirectionZ * segmentDirectionZ
    );

    if (projection >= segmentLength + d) {
        // The sphere is past the segment's end point
        return false;
    }

    // Calculate the closest point on the segment to the sphere's center
    double closestPointX = startX + (segmentDirectionX * (projection / segmentLength));
    double closestPointY = startY + (segmentDirectionY * (projection / segmentLength));
    double closestPointZ = startZ + (segmentDirectionZ * (projection / segmentLength));

    // Calculate the distance between the closest point and the sphere's center
    double distance = sqrt(
        (closestPointX - sphereX)*(closestPointX - sphereX) +
        (closestPointY - sphereY)*(closestPointY - sphereY) +
        (closestPointZ - sphereZ)*(closestPointZ - sphereZ)
    );

    if (distance <= sphereRadius + d) {
        // The segment is near the sphere
        return true;
    }

    // The segment is not near the sphere
    return false;

}

// Device function for checking whether the given set of joint angles lead to a robot in free space
__device__ bool device_in_freespace(double q1, double q2, double q3, const double* obstacles, int numObstacles) {
    double xA = -sin(q1) * cos(q2);
    double yA = cos(q1) * cos(q2);
    double zA = sin(q2);
    double xB = xA - sin(q1) * cos(q2 + q3);
    double yB = yA + cos(q1) * cos(q2 + q3);
    double zB = zA + sin(q2 + q3);

    for (int i = 0; i < numObstacles; ++i) {
        double sphereX = obstacles[i];
        double sphereY = obstacles[i+1];
        double sphereZ = obstacles[i+2];
        double sphereR = obstacles[i+3];
        if (device_segment_near_sphere(0.15, 0.0, 0.0, 0.0,
                                   xA, yA, zA,
                                   sphereX, sphereY, sphereZ, sphereR)) {
            return false;
        }
        if (device_segment_near_sphere(0.15, xA, yA, zA,
                                   xB, yB, zB,
                                   sphereX, sphereY, sphereZ, sphereR)) {
            return false;
        }
    }
    return true;
}

// Kernel for creating nodes
__global__ void create_nodes_kernel(Node* nodes, const int numNodes, curandGenerator_t gen, const double* rand_pts, const double* obstacles, const int numObstacles, const int numRandPts) {
    __shared__ double data[501*3];

    const int tid = blockIdx.x * blockDim.x + threadIdx.x;
    bool found = false;
    if (tid < numNodes) {
        double pi = 3.14159265358979323846;
        int idx = 3 * tid;
        while (!found) {
            double q1 = rand_pts[idx] * 2 * pi - pi;
            double q2 = rand_pts[idx+1] * 2 * pi - pi;
            double q3 = rand_pts[idx+2] * 2 * pi - pi;
            if (device_in_freespace(q1, q2, q3, obstacles, numObstacles)) {
                found = true;
                data[0 * 501 + tid] = q1;
                data[1 * 501 + tid] = q2;
                data[2 * 501 + tid] = q3;
            }
            idx = idx + (3 * numNodes);
        }
    }

    __syncthreads();

    if (tid < numNodes) {
        Node* node = &nodes[tid];
        node->id = tid;
        node->q1 = data[tid];
        node->q2 = data[tid+501];
        node->q3 = data[tid+501*2];
    }

}


// Host function for creating nodes
void createNodes(Node* cpu_nodes, int N, double* obstacles) {
    curandGenerator_t gen;
    Node *d_nodes;
    double *d_rand_pts;
    double *d_obstacles;

    // Allocating obstacles on device
    CUDA_CALL(cudaMalloc((void **)&d_obstacles, (NUM_OBSTACLES * 4)*sizeof(double)));
    CUDA_CALL(cudaMemcpy(d_obstacles, obstacles, (NUM_OBSTACLES * 4)*sizeof(double), cudaMemcpyHostToDevice));

    // Allocating nodes on device
    CUDA_CALL(cudaMalloc((void **)&d_nodes, (N+2)*sizeof(Node)));

    // Allocate doubles on device
    CUDA_CALL(cudaMalloc((void **)&d_rand_pts, (N*3*NUM_RANDS)*sizeof(double)));

    /* Create pseudo-random number generator */
    CURAND_CALL(curandCreateGenerator(&gen, CURAND_RNG_PSEUDO_DEFAULT));

    // Generate doubles on device
    CURAND_CALL(curandGenerateUniformDouble(gen, d_rand_pts, N*3*NUM_RANDS));

    int numThreads = 512;
    int numBlocks = max((int)(N / numThreads), 1);
    std::cout << "Create Nodes: calling kernel\n";
    create_nodes_kernel<<<numBlocks, numThreads>>>(d_nodes, N, gen, d_rand_pts, d_obstacles, NUM_OBSTACLES, NUM_RANDS);
    CUDA_CALL(cudaDeviceSynchronize());

    // Putting nodes on cpu
    CUDA_CALL(cudaMemcpy(cpu_nodes, d_nodes, (N+2)*sizeof(Node), cudaMemcpyDeviceToHost));

    // Allocating empty neighbors
    for (int i = 0; i < N+2; i++) {
        Node* node = &cpu_nodes[i];
        node->neighbors = (int*)malloc((N + 2) * sizeof(int));
    }

    // Freeing device memory
    CUDA_CALL(cudaFree(d_nodes));
    CUDA_CALL(cudaFree(d_rand_pts));
    CUDA_CALL(cudaFree(d_obstacles));
    CURAND_CALL(curandDestroyGenerator(gen));
}

// Device function for distance between nodes
__device__ double distance(Node* node, Node* other) {
    return sqrt((node->q1 - other->q1) * (node->q1 - other->q1) +
                (node->q2 - other->q2) * (node->q2 - other->q2) +
                (node->q3 - other->q3) * (node->q3 - other->q3));
}

// Device funtion to obtain the angles at which the intermediate node would be located
__device__ void device_intermediate(Node* node, Node* other, double alpha, double& q1, double& q2, double& q3) {
        q1 = node->q1 + alpha * (other->q1 - node->q1);
        q2 = node->q2 + alpha * (other->q2 - node->q2);
        q3 = node->q3 + alpha * (other->q3 - node->q3);
    }

// Generate a set of deltas to mimic the Vandercorput sequence and test that intermediates are in free space
__device__ bool device_connects_to(double dq, Node* node, Node* other, const double* obstacles, int numObstacles) {
    double dist = distance(node, other);
    double delta_update = dq / dist / 2.0;
    double delta = dq / dist / 2.0;
    double q1, q2, q3;
    
    while (delta < 1.0) {
        device_intermediate(node, other, delta, q1, q2, q3);
        if (!device_in_freespace(q1, q2, q3, obstacles, numObstacles)) {
            return false;
        }
        delta = delta + delta_update;
    }
    return true;
}

// Kernel for connecting neighbors
__global__ void connect_neighbors_kernel(Node* nodes, int numNodes, int k, double dq, const double* obstacles, int numObstacles) {
    int tid = blockIdx.x * blockDim.x + threadIdx.x;

    if (tid < numNodes) {
        Node* node = &nodes[tid];
        int id = node->id;
        for (int i = 0; i < numNodes; ++i) {
            __syncthreads();
            if (node->total_neighbors < k) {
                if (i != id) {
                    Node* potential_neighbor = &nodes[i];
                    double dist = distance(node, potential_neighbor);
                    if (dist < 2.0 && device_connects_to(dq, node, potential_neighbor, obstacles, numObstacles)) {
                        node->neighbors[node->total_neighbors] = potential_neighbor->id;
                        potential_neighbor->neighbors[potential_neighbor->total_neighbors] = id;
                        node->total_neighbors++;
                        potential_neighbor->total_neighbors++;
                    }
                }
            }
        }
    }
}

// Host function for connect K neighbors
void connectKNeighbors(Node* cpu_nodes, int K, double* obstacles) {

    // Putting nodes on device
    Node* d_nodes;
    CUDA_CALL(cudaMalloc((void **) &d_nodes, (N+2)*sizeof(Node)));
    CUDA_CALL(cudaMemcpy(d_nodes, cpu_nodes, (N+2)*sizeof(Node), cudaMemcpyHostToDevice));

    // Putting neighbors on device
    int* neighbors[(N+2)];
    for (int i = 0; i < N+2; i++) {
        CUDA_CALL(cudaMalloc((void**) &(neighbors[i]), sizeof(int)*(N+2)));
        CUDA_CALL(cudaMemcpy(neighbors[i], cpu_nodes[i].neighbors, sizeof(int)*(N+2), cudaMemcpyHostToDevice));
        CUDA_CALL(cudaMemcpy(&(d_nodes[i].neighbors), &(neighbors[i]), sizeof(int*), cudaMemcpyHostToDevice));
    }
    

    double *d_obstacles;
    // Allocating obstacles on device
    CUDA_CALL(cudaMalloc((void **)&d_obstacles, (NUM_OBSTACLES * 4)*sizeof(double)));
    CUDA_CALL(cudaMemcpy(d_obstacles, obstacles, (NUM_OBSTACLES * 4)*sizeof(double), cudaMemcpyHostToDevice));


    // Run connect neighbors algorithm on the GPU
    std::cout << "Connect neighbors: run on GPU \n";
    int numThreads = 512;
    int numBlocks = max((int)(N+2) / numThreads, 1);
    connect_neighbors_kernel<<<numBlocks, numThreads>>>(d_nodes, N+2, K, Dq, d_obstacles, NUM_OBSTACLES);
    CUDA_CALL(cudaDeviceSynchronize());

    // Copy data back to the host
    CUDA_CALL(cudaMemcpy(cpu_nodes, d_nodes, (N+2)*sizeof(Node), cudaMemcpyDeviceToHost));

    // Copy neighbors to host
    for (int i = 0; i < N+2; i++) {
        int* cpu_neighbors = (int *)malloc((N+2)*sizeof(int));
        cudaMemcpy(cpu_neighbors, neighbors[i], sizeof(int)*(N+2), cudaMemcpyDeviceToHost);
        memcpy(&(cpu_nodes[i].neighbors), &cpu_neighbors, sizeof(int*));
        CUDA_CALL(cudaFree(neighbors[i]));
    }

    CUDA_CALL(cudaFree(d_nodes));
    CUDA_CALL(cudaFree(d_obstacles));

}

// Kernel for A* algorithm
__global__ void astar_kernel(Node* nodes, int numNodes, Node* start, Node* goal, bool* path_found) {

    int tid = blockIdx.x * blockDim.x + threadIdx.x;

    if (tid < numNodes && !path_found[0]) {
        Node* current = &nodes[tid];


        // Calculate the heuristic cost
        current->ctogo = distance(current, goal);
        current->parent = -1;

        // Initialize start node
        if (current->id == start->id) {
            current->creach = 0;
            current->ctotal = current->creach + current->ctogo;
            current->seen = true;
        }

        __syncthreads();
        
        // A* algorithm
        if (threadIdx.x == 0) {
            while (!path_found[0]) {
                // Find the node with the lowest score
                int minNodeIdx = -1;
                double minF = 999999;

                for (int i = 0; i < numNodes; ++i) {
                    Node* node = &nodes[i];
                    if (!node->done && node->seen && node->ctotal < minF) {
                        minNodeIdx = node->id;
                        minF = node->ctotal;
                    }                }

                if (minNodeIdx == -1)
                    break;

                // Update neighboring nodes
                Node* minNode = &nodes[minNodeIdx];
                for (int i = 0; i < minNode->total_neighbors; i++) {
                    int neighborId = minNode->neighbors[i];
                    Node* neighbor = &nodes[neighborId];

                    if (!neighbor->done) {
                        float tentative_creach = minNode->creach + distance(minNode, neighbor);

                        if (!neighbor->seen) {
                            neighbor->seen = true;
                            neighbor->parent = minNodeIdx;
                            neighbor->creach = tentative_creach;
                            neighbor->ctotal = neighbor->creach + neighbor->ctogo;
                            continue;
                        }

                        if (tentative_creach < neighbor->creach) {
                            neighbor->creach = tentative_creach;
                            neighbor->ctotal = neighbor->creach + neighbor->ctogo;
                            neighbor->parent = minNodeIdx;
                        }
                    }
                }

                minNode->done = true;

                // Check if the goal is reached
                if (minNode->id == goal->id) {
                    path_found[0] = 1;
                }

            }
        }
    }
}

// Host function for A* algorithm
void astar(Node* cpu_nodes, Node& start, Node& goal, bool& path_found) {

    Node* d_nodes;
    CUDA_CALL(cudaMalloc((void **) &d_nodes, (N+2)*sizeof(Node)));
    CUDA_CALL(cudaMemcpy(d_nodes, cpu_nodes, (N+2)*sizeof(Node), cudaMemcpyHostToDevice));

    for (int i = 0; i < N+2; i++) {
        int* neighbors;
        CUDA_CALL(cudaMalloc((void**) &neighbors, sizeof(int)*(N+2)));
        CUDA_CALL(cudaMemcpy(neighbors, cpu_nodes[i].neighbors, sizeof(int)*(N+2), cudaMemcpyHostToDevice));
        CUDA_CALL(cudaMemcpy(&(d_nodes[i].neighbors), &neighbors, sizeof(int*), cudaMemcpyHostToDevice));
    }
    
    // Copy data to device
    Node* d_start;
    Node* d_goal;
    CUDA_CALL(cudaMalloc((void **) &d_start, sizeof(Node)));
    CUDA_CALL(cudaMalloc((void **) &d_goal, sizeof(Node)));
    CUDA_CALL(cudaMemcpy(d_start, &start, sizeof(Node), cudaMemcpyHostToDevice));
    CUDA_CALL(cudaMemcpy(d_goal, &goal, sizeof(Node), cudaMemcpyHostToDevice));
    bool* d_path_found;
    CUDA_CALL(cudaMalloc((void**) &d_path_found, sizeof(bool)));
    CUDA_CALL(cudaMemcpy(d_path_found, &path_found, sizeof(bool), cudaMemcpyHostToDevice));


    // Run A* algorithm on the GPU
    std::cout << "A-star: run on GPU \n";
    int numThreads = 512;
    int numBlocks = max((int)(N+2) / numThreads, 1);
    astar_kernel<<<numBlocks, numThreads>>>(d_nodes, N+2, d_start, d_goal, d_path_found);
    CUDA_CALL(cudaDeviceSynchronize());

    // Copy path found flag back to the host
    CUDA_CALL(cudaMemcpy(&path_found, d_path_found, sizeof(bool), cudaMemcpyDeviceToHost));

    // Copy data back to the host
    CUDA_CALL(cudaMemcpy(cpu_nodes, d_nodes, (N+2)*sizeof(Node), cudaMemcpyDeviceToHost));
    CUDA_CALL(cudaMemcpy(&start, d_start, sizeof(Node), cudaMemcpyDeviceToHost));
    CUDA_CALL(cudaMemcpy(&goal, d_goal, sizeof(Node), cudaMemcpyDeviceToHost));

    // Free dev memory
    CUDA_CALL(cudaFree(d_nodes));
    CUDA_CALL(cudaFree(d_start));
    CUDA_CALL(cudaFree(d_goal));
}


int main() {

    Node start = Node(startq1, startq2, startq3);
    Node goal = Node(goalq1,  goalq2,  goalq3);
    start.id = N;
    goal.id = N+1;
    double* obstacles = (double *)malloc(NUM_OBSTACLES * 4 * sizeof(double));
    for (int i = 0; i < NUM_OBSTACLES; ++i) {
        obstacles[i]   = spheres[i][0];
        obstacles[i+1] = spheres[i][1];
        obstacles[i+2] = spheres[i][2];
        obstacles[i+3] = spheres[i][3];
    }

    std::cout << "Create nodes \n";
    Node* nodes = (Node *)calloc(N+2, sizeof(Node));;
    createNodes(nodes, N, obstacles);
    nodes[N].q1 = startq1;
    nodes[N].q2 = startq2;
    nodes[N].q3 = startq3;
    nodes[N].id = N;

    nodes[N+1].q1 = goalq1;
    nodes[N+1].q2 = goalq2;
    nodes[N+1].q3 = goalq3;
    nodes[N+1].id = N+1;

    std::cout << "Connect neighbors \n";
    connectKNeighbors(nodes, K, obstacles);

    bool path_found = false;
    std::cout << "Plan path \n";
    astar(nodes, start, goal, path_found);

    std::vector<Node*> path;
    
    if (path_found) {
        std::cout << "Path found \n";
        Node* current = &nodes[(N+1)];
        while (current != nullptr) {
            path.insert(path.begin(), current);
            if (current->parent == -1) {
                break;
            }
            current = &nodes[current->parent];
        }        
    }
    else {
        std::cout << "Path not found" << std::endl;
    }

    if (path.size() > 0) {
        std::cout << "Showing path \n";

        for(unsigned int i = 0; i < path.size(); i++)
        {
            std::cout << "Joints " << path[i]->q1 * 180 / pi << "deg, " << path[i]->q2 * 180 / pi << "deg, " << path[i]->q3 * 180 / pi << "deg\n";
        }

        // Generating intermediate nodes for the purpose of visualizing
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

        std::cout << "\nShowing path with intermediate nodes \n";
        for(unsigned int i = 0; i < path_nodes_intermediates.size(); i++)
        {
            std::cout << "Joints " << path_nodes_intermediates[i].q1 * 180 / pi << "deg, " << path_nodes_intermediates[i].q2 * 180 / pi << "deg, " << path_nodes_intermediates[i].q3 * 180 / pi << "deg\n";
        }

        std::cout << "\nShowing link locations of intermediate nodes \n";
        for(unsigned int i = 0; i < path_nodes_intermediates.size(); i++)
        {
            std::vector<std::vector<std::vector<double>>> links = get_links(path_nodes_intermediates[i]);
            std::vector<std::vector<double>> link1 = links[0];
            std::vector<std::vector<double>> link2 = links[1];

            std::cout << "Link 1: (" << link1[0][0] << ", " << link1[0][1] << ", " << link1[0][2] << ") -> (" << link1[1][0] << ", " << link1[1][1] << ", " << link1[1][2] << ") Link 2: (" << link2[0][0] << ", " << link2[0][1] << ", " << link2[0][2] << ") -> (" << link2[1][0] << ", " << link2[1][1] << ", " << link2[1][2] << ")\n";
        }

        // Output the links to text file to be used by visualizer
        std::ofstream myfile;
        myfile.open ("prm_test_output.txt");
        myfile << 2 << "; " << path_nodes_intermediates.size() << "\n";
        for(unsigned int i = 0; i < path_nodes_intermediates.size(); i++)
        {
            std::vector<std::vector<std::vector<double>>> links = get_links(path_nodes_intermediates[i]);
            std::vector<std::vector<double>> link1 = links[0];
            std::vector<std::vector<double>> link2 = links[1];

            myfile << link1[0][0] << ", " << link1[0][1] << ", " << link1[0][2] << ",; " << link1[1][0] << ", " << link1[1][1] << ", " << link1[1][2] << ",; " << link2[1][0] << ", " << link2[1][1] << ", " << link2[1][2] << ",;\n";
        }
        myfile.close();
    }

    // Free nodes and obstacles
    free(nodes);
    free(obstacles);

    return 0;
}
