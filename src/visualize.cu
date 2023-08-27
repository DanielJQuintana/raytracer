#include <cuda_runtime.h>
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <stdlib.h>
#include <tuple>
#include <cstring>
#include <stdio.h>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "visualize.cuh"
#include "helper_cuda.h"

#define PI 3.14159265

// Converts degrees to radians (float)
float hostDegToRad(float angle) {
    return angle * PI / 180;
}

// Converts radians to degrees (float)
float hostRadToDeg(float angle) {
    return 180 * angle / PI;
}

// Adds input Vectors a and b to produce output vector c = a + b
Vector hostAddVector(Vector a, Vector b) {
    return Vector {a.x + b.x, a.y + b.y, a.z + b.z};
}

// Subtracts input Vector b from a to produce output vector c = a - b
Vector hostSubtractVector(Vector a, Vector b) {
    return Vector {a.x - b.x, a.y - b.y, a.z - b.z};
}

// Scales input Vector a by a factor of alpha to produce output vector b = alpha * a
Vector hostScaleVector(Vector a, float alpha) {
    return Vector {a.x * alpha, a.y * alpha, a.z * alpha};
}

// Returns the dot product between input Vectors a and b
float hostDotProduct(Vector a, Vector b) {
   return a.x * b.x + a.y * b.y + a.z * b.z;
}

// Computes the cross product of input Vectors a and b, c = a x b, returning c
Vector hostCrossProduct(Vector a, Vector b) {
    return Vector {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

Vector hostNormalizeVector(Vector a) {
    return hostScaleVector(a, 1/sqrtf(hostDotProduct(a, a)));
}

// Generates a random float
float randomFloat() {
    return (float)(rand()) / (float)(rand());
}

// Generates a random Vector
Vector randomVector() {
    return {randomFloat(), randomFloat(), randomFloat()};
}


// Converts degrees to radians (float)
__device__ float degToRad(float angle) {
    return angle * PI / 180;
}

// Converts radians to degrees (float)
__device__ float radToDeg(float angle) {
    return 180 * angle / PI;
}

// Adds input Vectors a and b to produce output vector c = a + b
__device__ Vector addVector(Vector a, Vector b) {
    return Vector {a.x + b.x, a.y + b.y, a.z + b.z};
}

// Subtracts input Vector b from a to produce output vector c = a - b
__device__ Vector subtractVector(Vector a, Vector b) {
    return Vector {a.x - b.x, a.y - b.y, a.z - b.z};
}

// Scales input Vector a by a factor of alpha to produce output vector b = alpha * a
__device__ Vector scaleVector(Vector a, float alpha) {
    return Vector {a.x * alpha, a.y * alpha, a.z * alpha};
}

// Returns the dot product between input Vectors a and b
__device__ float dotProduct(Vector a, Vector b) {
   return a.x * b.x + a.y * b.y + a.z * b.z;
}

// Computes the cross product of input Vectors a and b, c = a x b, returning c
__device__ Vector crossProduct(Vector a, Vector b) {
    return Vector {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

__device__ Vector normalizeVector(Vector a) {
    return scaleVector(a, 1/sqrtf(dotProduct(a, a)));
}


// Rotates input Vector a about the y-axis by angle degrees, returning the resulting vector
__device__ Vector rotateY(Vector a, float angle) {
    return Vector {a.x * cos(degToRad(angle)) + a.z * sin(degToRad(angle)),
                    a.y,
                    -1 * a.x * sin(degToRad(angle)) + a.z * cos(degToRad(angle))};
}

// Rotates input Vector a about the x-axis by angle degrees, returning the resulting Vector
__device__ Vector rotateX(Vector a, float angle) {
    return Vector {a.x,
                    a.y * cos(degToRad(angle)) - a.z * sin(degToRad(angle)),
                    a.y * sin(degToRad(angle)) + a.z * cos(degToRad(angle))};
}

// Returns Plane of the input Triangle
Plane hostFindPlane(Triangle t) {
    Vector a = hostSubtractVector(t.b, t.a);
    Vector b = hostSubtractVector(t.c, t.a);

    Vector n = hostCrossProduct(a, b);
    Plane p = {n.x, n.y, n.z, -1 * hostDotProduct(n, t.a)};

    return p;
}

// Returns Plane of the input Triangle
__device__ Plane findPlane(Triangle t) {
    Vector a = subtractVector(t.b, t.a);
    Vector b = subtractVector(t.c, t.a);

    Vector n = crossProduct(a, b);
    Plane p = {n.x, n.y, n.z, -1 * dotProduct(n, t.a)};

    return p;
}

// Returns ray that lies between the camera and the pixel (pixel_x, pixel_y) in the image.
__device__ Ray getRay(int pixel_x, int pixel_y, Camera c, Image img) {
    Ray r;
    r.origin = c.pos;

    float pixel_x_dim = img.x_dim / static_cast<float>(img.x_pixels);
    float pixel_y_dim = img.y_dim / static_cast<float>(img.y_pixels);

    Vector adjust = {pixel_x_dim * (pixel_x - (img.x_pixels / 2)), 
                        pixel_y_dim * (pixel_y - (img.y_pixels / 2)),
                        c.dist};

    r.m = rotateX(rotateY(adjust, c.yaw), c.pitch);

    return r;
}

/*
 Checks that the x coordinate of the input Point is within the box boundaries of the input Triangle.
*/
__device__ bool checkXbounds(Point i, Triangle t) {
    return !(i.x < min(min(t.a.x, t.b.x), t.c.x) || i.x > max(max(t.a.x, t.b.x), t.c.x));
}

/*
 Checks that the y coordinate of the input Point is within the box boundaries of the input Triangle.
*/
__device__ bool checkYbounds(Point i, Triangle t) {
    return !(i.y < min(min(t.a.y, t.b.y), t.c.y) || i.y > max(max(t.a.y, t.b.y), t.c.y));
}

/*
 Checks that the z coordinate of the input Point is within the box boundaries of the input Triangle.
*/
__device__ bool checkZbounds(Point i, Triangle t) {
    return !(i.z < min(min(t.a.z, t.b.z), t.c.z) || i.z > max(max(t.a.z, t.b.z), t.c.z));
}


// Checks that Point i and Vector a lie on the same side of the line between Vectors b and c.
__device__ bool sameSide(Point i, Vector a, Vector b, Vector c) {
    Vector v = subtractVector(b, c);
    return (dotProduct(crossProduct(v, subtractVector(i, c)), crossProduct(v, subtractVector(a, c))) >= 0);
}

// Determines if the input Point is a valid intersection point on Triangle t. 
__device__ bool isValidIntersect(Point i, float lmda, Triangle t) {
    if (lmda < 0) return false;
    if (!checkXbounds(i, t)) return false;
    if (!checkYbounds(i, t)) return false;
    if (!checkZbounds(i, t)) return false;

    return sameSide(i, t.a, t.b, t.c) && sameSide(i, t.b, t.a, t.c) && sameSide(i, t.c, t.a, t.b);
}

/*
 Returns a Point representing the interection point of a Ray and a Triangle's Plane. Returns a zero
 vector if no intersect is found.
*/
__device__ Point triangleIntersect(Ray r, Triangle t) {
    float lmda = -1 * (dotProduct({t.p.a, t.p.b, t.p.c}, r.origin) + t.p.d)
                            / dotProduct({t.p.a, t.p.b, t.p.c}, r.m);

    Point intersect = {lmda * r.m.x + r.origin.x, lmda * r.m.y + r.origin.y, lmda * r.m.z + r.origin.z};

    if (isValidIntersect(intersect, lmda, t)) {
        return intersect;
    }

    return {0, 0, 0};
}

/*
 Returns a Point representing the intersection Point of a Ray and a Sphere that is closer to the 
 camera. Returns a zero vector if no intersect is found.
*/
__device__ Point sphereIntersect(Ray r, Sphere s) {
    Vector originToCenter = subtractVector(r.origin, s.center);
    float rayMagnitude = dotProduct(r.m, r.m);
    float disc = pow(dotProduct(r.m, originToCenter), 2) - rayMagnitude * (dotProduct(originToCenter, originToCenter) - powf(s.radius, 2));

    if (disc < 0) {
        return {0, 0, 0};
    }

    float lmda = (-1 * dotProduct(r.m, originToCenter) - sqrtf(disc)) / rayMagnitude;
    return addVector(r.origin, scaleVector(r.m, lmda));
}

// Checks if the input vector is zero in all dimensions
__device__ bool isZeroVector(Vector a) {
    return a.x == 0 && a.y == 0 && a.z == 0;
}

// Returns the distance between Points i and j.
__device__ float distance(Point i, Point j) {
    return sqrtf(powf(i.x - j.x, 2) + powf(i.y - j.y, 2) + powf(i.z - j.z, 2));
}

// Adjusts the brightness of the input Color to sharpen the contrast of neighboring pixels.
__device__ Color adjustBrightness(Color color, float brightness) {
    color.r = pow(color.r * 3 * brightness, 2);
    color.g = powf(color.g * 3 * brightness, 2);
    color.b = powf(color.b * 3 * brightness, 2);
    return color;
}

// Sets the pixel at (i, j) to the input Color with the given input brightness.
__device__ void setPixel(int i, int j, float* pixels, int x_dim, Color color, float brightness) {
    if (brightness != -1) color = adjustBrightness(color, brightness);
    pixels[i * 3 + j * x_dim * 3    ] = color.r;
    pixels[i * 3 + j * x_dim * 3 + 1] = color.g;
    pixels[i * 3 + j * x_dim * 3 + 2] = color.b;
}

// Helper for optimized ray tracing kernel
__device__ void optimizeRaytraceHelper(int i, int j, int mod_i, int mod_j, float* data, Image* img, TriangleSurface* triangles, int num_triangles, SphereSurface* spheres, int num_spheres, Camera* c) {
    Ray r = getRay(i, j, *c, *img);

    float min_dist = -1;
    Color color = {1, 1, 1};

    for (int k = 0; k < num_triangles; k++) {
        Triangle t = triangles[k].t;

        Point intersect = triangleIntersect(r, t);
        float dist = distance(intersect, c->pos);

        if (!isZeroVector(intersect) && (dist < min_dist || min_dist == -1)) {
            //printf("Found valid triangle intersect\n");
            color = triangles[k].color;
            min_dist = dist;
        }
    }

    for (int k = 0; k < num_spheres; k++) {
        Point intersect = sphereIntersect(r, spheres[k].s);
        float dist = distance(intersect, c->pos);

        if (!isZeroVector(intersect) && (dist < min_dist || min_dist == -1)) {
            //printf("Found valid sphere intersect\n");
            color = spheres[k].color;
            min_dist = dist;
        }
    }

    setPixel(mod_i, 31 - mod_j, data, 33, color, 1/min_dist);
}

// Optimized ray tracing kernel - finds all object intersects for each pixel and sets their color accordingly
// Uses shared memory and loop unrolling
// This function does not currently work - causes cudaIllegalAddress error
__global__ void optimizeRaytraceKernel(Image* img, TriangleSurface* triangles, int num_triangles, SphereSurface* spheres, int num_spheres, Camera* c) {
    // Allocate shared memory for 32 x 32 block of pixels (requires change of block and grid dimensions, since 64 x 64 x 3 is too much shared memory)
    __shared__ float data[33 * 3 * 32];

    unsigned i = threadIdx.x + 32 * blockIdx.x;
    unsigned mod_i = threadIdx.x;
    unsigned j = 4 * threadIdx.y + 32 * blockIdx.y;
    unsigned mod_j = 4 * threadIdx.y;
    unsigned end_j = j + 4;

    // Load pixel data
    for (; j < end_j; j++, mod_j++) {
        data[mod_i * 3 + 33 * mod_j * 3    ] = img->pixels[i * 3 + img->x_pixels * j * 3    ];
        data[mod_i * 3 + 33 * mod_j * 3 + 1] = img->pixels[i * 3 + img->x_pixels * j * 3 + 1];
        data[mod_i * 3 + 33 * mod_j * 3 + 2] = img->pixels[i * 3 + img->x_pixels * j * 3 + 2];
    }

    __syncthreads();
    
    // Unrolling loop
    optimizeRaytraceHelper(i, j    , mod_i, mod_j    , data, img, triangles, num_triangles, spheres, num_spheres, c);
    optimizeRaytraceHelper(i, j + 1, mod_i, mod_j + 1, data, img, triangles, num_triangles, spheres, num_spheres, c);
    optimizeRaytraceHelper(i, j + 2, mod_i, mod_j + 2, data, img, triangles, num_triangles, spheres, num_spheres, c);
    optimizeRaytraceHelper(i, j + 3, mod_i, mod_j + 3, data, img, triangles, num_triangles, spheres, num_spheres, c);
    
    __syncthreads();

    j = 4 * threadIdx.y + 32 * blockIdx.y;
    mod_j = 4 * threadIdx.y;

    // Write to global memory
    for (; j < end_j; j++, mod_j++) {
        img->pixels[i * 3 + img->x_pixels * j * 3    ] = data[mod_i * 3 + 33 * mod_j * 3    ];
        img->pixels[i * 3 + img->x_pixels * j * 3 + 1] = data[mod_i * 3 + 33 * mod_j * 3 + 1];
        img->pixels[i * 3 + img->x_pixels * j * 3 + 2] = data[mod_i * 3 + 33 * mod_j * 3 + 2];
    }
}

// Ray tracing kernel - finds all object intersects for each pixel and sets their color accordingly
__global__ void naiveRaytraceKernel(Image* img, TriangleSurface* triangles, int num_triangles, SphereSurface* spheres, int num_spheres, Camera* c) {
    unsigned i = threadIdx.x + 64 * blockIdx.x;
    unsigned j = 4 * threadIdx.y + 64 * blockIdx.y;
    unsigned end_j = j + 4;

    for (; j < end_j; j++) {
        Ray r = getRay(i, j, *c, *img);

        float min_dist = -1;
        Color color = {1, 1, 1};

        for (int k = 0; k < num_triangles; k++) {
            Triangle t = triangles[k].t;

            Point intersect = triangleIntersect(r, t);
            float dist = distance(intersect, c->pos);

            if (!isZeroVector(intersect) && (dist < min_dist || min_dist == -1)) {
                color = triangles[k].color;
                min_dist = dist;
            }
        }

        for (int k = 0; k < num_spheres; k++) {
            Point intersect = sphereIntersect(r, spheres[k].s);
            float dist = distance(intersect, c->pos);

            if (!isZeroVector(intersect) && (dist < min_dist || min_dist == -1)) {
                color = spheres[k].color;
                min_dist = dist;
            }
        }

        setPixel(i, img->x_pixels - j - 1, img->pixels, img->x_pixels, color, 1/min_dist);
    }
}

// Transfers memory to device and calls the ray tracing kernel
// The image is broken into 64 x 64 chunks, each handled by a single block. A block contains 64 x 16 threads, with threads handling 4 pixels each. 
void cudaRaytrace(Image img, TriangleSurface* triangles, int num_triangles, SphereSurface* spheres, int num_spheres, Camera c) {
    dim3 blockSize(64, 16);
    dim3 gridSize(img.x_pixels / 64, img.y_pixels / 64);

    int n_pixels = img.x_pixels * img.y_pixels;

    Image* d_img;
    CUDA_CALL( cudaMalloc((void **) &d_img, sizeof(Image)) );
    CUDA_CALL( cudaMemcpy(d_img, &img, sizeof(Image), cudaMemcpyHostToDevice) );
    float* d_pixels;
    CUDA_CALL( cudaMalloc((void **) &d_pixels, sizeof(float) * 3 * n_pixels ) );
    CUDA_CALL( cudaMemcpy(d_pixels, img.pixels, sizeof(float) * 3 * n_pixels, cudaMemcpyHostToDevice) );
    CUDA_CALL( cudaMemcpy(&(d_img->pixels), &d_pixels, sizeof(float*), cudaMemcpyHostToDevice) );

    TriangleSurface* d_triangles;
    CUDA_CALL( cudaMalloc((void **)&d_triangles, sizeof(TriangleSurface) * num_triangles) );
    CUDA_CALL( cudaMemcpy(d_triangles, triangles, sizeof(TriangleSurface) * num_triangles, cudaMemcpyHostToDevice) );

    SphereSurface* d_spheres;
    CUDA_CALL( cudaMalloc((void**)&d_spheres, sizeof(SphereSurface) * num_spheres) );
    CUDA_CALL( cudaMemcpy(d_spheres, spheres, sizeof(SphereSurface) * num_spheres, cudaMemcpyHostToDevice) );
    
    Camera* d_c;
    CUDA_CALL( cudaMalloc((void**)&d_c, sizeof(Camera)) );
    CUDA_CALL( cudaMemcpy(d_c, &c, sizeof(Camera), cudaMemcpyHostToDevice) );

    naiveRaytraceKernel<<<gridSize, blockSize>>>(d_img, d_triangles, num_triangles, d_spheres, num_spheres, d_c);

	float* h_pixels;
	CUDA_CALL( cudaMalloc((void **)&h_pixels, sizeof(float) * 3 * n_pixels) );
	CUDA_CALL( cudaMemcpy(&h_pixels, &(d_img->pixels), sizeof(float*), cudaMemcpyDeviceToHost) );

    CUDA_CALL( cudaMemcpy(img.pixels, h_pixels, sizeof(float) * 3 * n_pixels, cudaMemcpyHostToHost) );

    CUDA_CALL( cudaFree(d_img) );
    CUDA_CALL( cudaFree(d_pixels) );
    CUDA_CALL( cudaFree(d_triangles) );
    CUDA_CALL( cudaFree(d_spheres) );
    CUDA_CALL( cudaFree(d_c) );
}

/*
 Builds the representation of our robot arm according to a sequence of points that 
 represent the joint positions (in tip space). Joints are represented by spheres, while links 
 between joints are made up of 8 triangles to form a cuboid. 
*/ 

void constructScene(Image img, Camera c, Point* points, int numPoints, SphereSurface* spheres, int num_spheres, TriangleSurface* triangles, int num_triangles, Vector arm) {
    Color sphereColor = {0 / 255.0, 0 / 255.0, 0 / 255.0};
    Color triangleColor = {252.0 / 255.0, 102.0 / 255.0, 3.0 / 255.0};

    float armSize = 0.1;

    for (int i = 0; i < numPoints - 1; i++) {
        spheres[i] = {{points[i], 0.1}, sphereColor};

        Vector normal = hostSubtractVector(points[i+1], points[i]);
        Vector test = hostAddVector(points[i], arm);
        Vector proj = hostSubtractVector(test, hostScaleVector(normal, hostDotProduct(test, normal) / hostDotProduct(normal, normal)));
        proj = hostScaleVector(hostNormalizeVector(proj), armSize);

        Point point1a = hostAddVector(points[i], proj);
        Point point2a = hostSubtractVector(points[i], proj);

        Vector cross = hostCrossProduct(normal, proj);
        cross = hostScaleVector(hostNormalizeVector(cross), armSize);
        Point point3a = hostAddVector(points[i], cross);
        Point point4a = hostSubtractVector(points[i], cross);

        Point point1b = hostAddVector(point1a, normal);
        Point point2b = hostAddVector(point2a, normal);
        Point point3b = hostAddVector(point3a, normal);
        Point point4b = hostAddVector(point4a, normal);

        Triangle t1, t2, t3, t4, t5, t6, t7, t8;

        t1.a = point1a; t1.b = point3a; t1.c = point3b;
        t2.a = point1a; t2.b = point1b; t2.c = point3b;
        t3.a = point3a; t3.b = point2a; t3.c = point2b;
        t4.a = point3a; t4.b = point3b; t4.c = point2b;
        t5.a = point4a; t5.b = point1a; t5.c = point1b;
        t6.a = point4a; t6.b = point4b; t6.c = point1b;
        t7.a = point2a; t7.b = point4a; t7.c = point4b;
        t8.a = point2a; t8.b = point2b; t8.c = point4b;

        triangles[8 * i    ] = {t1, triangleColor};
        triangles[8 * i + 1] = {t2, triangleColor};
        triangles[8 * i + 2] = {t3, triangleColor};
        triangles[8 * i + 3] = {t4, triangleColor};
        triangles[8 * i + 4] = {t5, triangleColor};
        triangles[8 * i + 5] = {t6, triangleColor};
        triangles[8 * i + 6] = {t7, triangleColor};
        triangles[8 * i + 7] = {t8, triangleColor};
    }

    spheres[numPoints - 1] = {{points[numPoints - 1], 0.1}, {1., 0., 0.}};

    for (int i = 0; i < num_triangles; i++) {
        Triangle t = triangles[i].t;
        t.p = hostFindPlane(t);
        triangles[i].t = t;
    }

    cudaRaytrace(img, triangles, num_triangles, spheres, num_spheres, c);
}

/*
 Returns a point from a list, which must have length at least 3. Used to convert the representation
 from prm_3D.
*/
Point pointFromList(float* list) {
    Point a;
    a.x = list[1];
    a.y = -1 * list[2];
    a.z = list[0];

    return a;
}

// Reads an arm path from the given file. 
Path readPositions(std::string pathToFile) {
    std::ifstream posFile; 
    posFile.open(pathToFile);
    Path path = {NULL, 0, 0};

    if (posFile.is_open()) {
        std::string paramString;
        getline(posFile, paramString);
        size_t delimPos = paramString.find(";");
        int numLinks = stoi(paramString.substr(0, delimPos));
        int numPositions = stoi(paramString.substr(delimPos + 1, paramString.length() - delimPos + 1));
        path.numLinks = numLinks;
        path.numPositions = numPositions;

        Point** positions = (Point **)malloc(numPositions * sizeof(Point *));
        for (int i = 0; i < numPositions; i++) {
            positions[i] = (Point *)malloc((numLinks + 1) * sizeof(Point));
        }

        std::string line;
        size_t majorPos = 0;
        int i = 0;
        while (getline(posFile, line)) {
            int j = 0;
            while ((majorPos = line.find(";")) != std::string::npos) {
                std::string linkString = line.substr(0, majorPos);
                size_t minorPos = 0;
                float* coordList = new float[3];
                int k = 0;
                while ((minorPos = linkString.find(",")) != std::string::npos) {
                    std::string coord = linkString.substr(0, minorPos);
                    coordList[k] = stof(coord);
                    linkString.erase(0, minorPos + 1);
                    k++;
                }

                positions[i][j] = pointFromList(coordList);
                line.erase(0, majorPos + 1);
                j++;
            }
            i++;
        }

        path.points = positions;
        posFile.close();
    }

    return path;
}

int main(int argc, char **argv) {
    int viewDimX = 512;
    int viewDimY = viewDimX;
    
    Color sphereColor = {15.0 / 255.0, 235.0 / 255.0, 40.0 / 255.0};

    // Hard-coded obstacles - identical to prm_3D code
    SphereSurface sphere1 = {{ {1.5, -1.5, 0.0}, 1.0 }, sphereColor};
    SphereSurface sphere2 = {{ {1.5, 1.5, 0.0}, 1.0 }, sphereColor};
    SphereSurface sphere3 = {{ {0.0, 0.0, -1.5}, 1.0 }, sphereColor};
    SphereSurface sphere4 = {{ {-1.5, 0.0, 0.0}, 1.0 }, sphereColor};

    float* pixels = (float*)malloc(viewDimX * viewDimY * 3 * sizeof(float));
    Image img = {pixels, viewDimX, viewDimY, 3, 3}; 

    Camera c = {{3, -0.5, -2}, 1, -40, 0};

    if (argc < 2) {
        std::cerr << "usage: visualize <input> [output]" << std::endl; 
        return 1;
    }

    Path path = readPositions(argv[1]);

    if (path.numPositions == 0) {
        std::cerr << "Valid file not found: " << argv[1] << std::endl;
        return 1;
    }

    int num_triangles = 8 * path.numLinks;
    int num_spheres = path.numLinks + 5;

    TriangleSurface* triangles = (TriangleSurface*)malloc(num_triangles * sizeof(TriangleSurface));
    SphereSurface* spheres = (SphereSurface*)malloc(num_spheres * sizeof(SphereSurface));

    spheres[num_spheres - 4] = sphere1;
    spheres[num_spheres - 3] = sphere2;
    spheres[num_spheres - 2] = sphere3;
    spheres[num_spheres - 1] = sphere4;

    // Random vector defining the arm - for projection onto plane
    Vector arm = randomVector();
   
	std::ofstream myfile; 
	if (argc == 3) {
    	myfile.open(argv[2]);
        
        if (!myfile.is_open()) {
            std::cerr << "Failed to open " << argv[2] << " - quitting" << std::endl;
            return 1;
        }

    	myfile << std::to_string(path.numPositions) << "," << std::to_string(viewDimX) << std::endl;
	} 
	else {
		std::cout << "Running in timing mode" << std::endl;
	}

    for (int n = 0; n < path.numPositions; n++) {
        constructScene(img, c, path.points[n], path.numLinks + 1, spheres, num_spheres, triangles, num_triangles, arm);  
        if (argc == 3) {
            for (int i = 0; i < viewDimX; i++) {
                for (int j = 0; j < viewDimY; j++) {
                    myfile << std::to_string(img.pixels[i * 3 + j * viewDimX * 3       ]) << " "
                            << std::to_string(img.pixels[i * 3 + j * viewDimX * 3 + 1]) << " "
                            << std::to_string(img.pixels[i * 3 + j * viewDimX * 3 + 2]) << " "
                            << std::endl;
                    }
            }
            if (n != path.numPositions - 1 && argc == 3) {
                myfile << "-" << std::endl;
            }
		}
    }

    for (int i = 0; i < path.numPositions; i++) {
        free(path.points[i]);
    }

    free(path.points);
    free(pixels);
    free(spheres);
    free(triangles);

    return 0;
}
