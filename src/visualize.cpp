#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <stdlib.h>
#include <tuple>
#include <cstring>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "visualize.hpp"

#define PI 3.14159265

// Converts degrees to radians (float)
float degToRad(float angle) {
    return angle * PI / 180;
}

// Converts radians to degrees (float)
float radToDeg(float angle) {
    return 180 * angle / PI;
}

// Adds input Vectors a and b to produce output vector c = a + b
Vector addVector(Vector a, Vector b) {
    return Vector {a.x + b.x, a.y + b.y, a.z + b.z};
}

// Subtracts input Vector b from a to produce output vector c = a - b
Vector subtractVector(Vector a, Vector b) {
    return Vector {a.x - b.x, a.y - b.y, a.z - b.z};
}

// Scales input Vector a by a factor of alpha to produce output vector b = alpha * a
Vector scaleVector(Vector a, float alpha) {
    return Vector {a.x * alpha, a.y * alpha, a.z * alpha};
}

// Generates a random float
float randomFloat() {
    return (float)(rand()) / (float)(rand());
}

// Generates a random Vector
Vector randomVector() {
    return {randomFloat(), randomFloat(), randomFloat()};
}

// Returns the dot product between input Vectors a and b
float dotProduct(Vector a, Vector b) {
   return a.x * b.x + a.y * b.y + a.z * b.z;
}

// Computes the cross product of input Vectors a and b, c = a x b, returning c
Vector crossProduct(Vector a, Vector b) {
    return Vector {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

Vector normalizeVector(Vector a) {
    return scaleVector(a, 1/sqrtf(dotProduct(a, a)));
}

// Rotates input Vector a about the y-axis by angle degrees, returning the resulting vector
Vector rotateY(Vector a, float angle) {
    return Vector {a.x * cosf(degToRad(angle)) + a.z * sinf(degToRad(angle)),
                    a.y,
                    -1 * a.x * sinf(degToRad(angle)) + a.z * cosf(degToRad(angle))};
}

// Rotates input Vector a about the x-axis by angle degrees, returning the resulting Vector
Vector rotateX(Vector a, float angle) {
    return Vector {a.x,
                    a.y * cosf(degToRad(angle)) - a.z * sinf(degToRad(angle)),
                    a.y * sinf(degToRad(angle)) + a.z * cosf(degToRad(angle))};
}

// Returns a string representation of a Vector
std::string vectorToString(Vector a) {
    return "(" + std::to_string(a.x) + " " + std::to_string(a.y) + " " + std::to_string(a.z) + ")";
}

// Returns a string representation of a Triangle
std::string triangleToString(Triangle t) {
    return vectorToString(t.a) + "  " + vectorToString(t.b) + "  " + vectorToString(t.c);
}

// Returns a string representation of a Plane
std::string planeToString(Plane p) {
    return "(" + std::to_string(p.a) + " " + std::to_string(p.b) + " " + std::to_string(p.c) + " " 
        + std::to_string(p.d) + ")";
}

// Returns a string representation of a Ray
std::string rayToString(Ray r) {
    return "Origin: " + vectorToString(r.origin) + "  Direction: " + vectorToString(r.m); 
}

// Returns Plane of the input Triangle
Plane findPlane(Triangle t) {
    Vector a = subtractVector(t.b, t.a);
    Vector b = subtractVector(t.c, t.a);

    Vector n = crossProduct(a, b);
    Plane p = {n.x, n.y, n.z, -1 * dotProduct(n, t.a)};

    return p;
}

// Returns ray that lies between the camera and the pixel (pixel_x, pixel_y) in the image.
Ray getRay(int pixel_x, int pixel_y, Camera c, Image img) {
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
 Returns a tuple representing the intersection Point of a Ray and a Sphere that is closer to the 
 camera, with a boolean indicating whether intersection occurred. If not, the intersection Point is 
 null.  
*/
std::tuple<Point, bool> sphereIntersect(Ray r, Sphere s) {
    Vector originToCenter = subtractVector(r.origin, s.center);
    float rayMagnitude = dotProduct(r.m, r.m);
    float disc = powf(dotProduct(r.m, originToCenter), 2) - rayMagnitude * (dotProduct(originToCenter, originToCenter) - powf(s.radius, 2));

    if (disc < 0) {
        Point empty;
        return {empty, false};
    }

    float lmda = (-1 * dotProduct(r.m, originToCenter) - sqrtf(disc)) / rayMagnitude;
    return {addVector(r.origin, scaleVector(r.m, lmda)), true};
}

/*
 Returns a tuple representing the interection point of a Ray and a Triangle's Plane, and the 
 distance along the Ray that the intersection occurs.
*/
std::tuple<Point, float> triangleIntersect(Ray r, Triangle t) {
    float lmda = -1 * (dotProduct({t.p.a, t.p.b, t.p.c}, r.origin) + t.p.d)
                            / dotProduct({t.p.a, t.p.b, t.p.c}, r.m);

    return {{lmda * r.m.x + r.origin.x, lmda * r.m.y + r.origin.y, lmda * r.m.z + r.origin.z}, lmda};
}

/*
 Checks that the x coordinate of the input Point is within the box boundaries of the input Triangle.
*/
bool checkXbounds(Point i, Triangle t) {
    if (i.x < std::min(std::min(t.a.x, t.b.x), t.c.x)) return false;
    if (i.x > std::max(std::max(t.a.x, t.b.x), t.c.x)) return false;
    return true;
}

/*
 Checks that the y coordinate of the input Point is within the box boundaries of the input Triangle.
*/
bool checkYbounds(Point i, Triangle t) {
    if (i.y < std::min(std::min(t.a.y, t.b.y), t.c.y)) return false;
    if (i.y > std::max(std::max(t.a.y, t.b.y), t.c.y)) return false; 
    return true;
}

/*
 Checks that the z coordinate of the input Point is within the box boundaries of the input Triangle.
*/
bool checkZbounds(Point i, Triangle t) {
    if (i.z < std::min(std::min(t.a.z, t.b.z), t.c.z)) return false;
    if (i.z > std::max(std::max(t.a.z, t.b.z), t.c.z)) return false;
    return true;
}


// Checks that Point i and Vector a lie on the same side of the line between Vectors b and c.
bool sameSide(Point i, Vector a, Vector b, Vector c) {
    Vector v = subtractVector(b, c);
    return (dotProduct(crossProduct(v, subtractVector(i, c)), crossProduct(v, subtractVector(a, c))) >= 0);
}

// Determines if the input Point is a valid intersection point on Triangle t. 
bool isValidIntersect(Point i, float lmda, Triangle t) {
    if (lmda < 0) return false;
    if (!checkXbounds(i, t)) return false;
    if (!checkYbounds(i, t)) return false;
    if (!checkZbounds(i, t)) return false;

    return sameSide(i, t.a, t.b, t.c) && sameSide(i, t.b, t.a, t.c) && sameSide(i, t.c, t.a, t.b);
}

// Returns the distance between Points i and j.
float distance(Point i, Point j) {
    return sqrtf(powf(i.x - j.x, 2) + powf(i.y - j.y, 2) + powf(i.z - j.z, 2));
}

// Adjusts the brightness of the input Color to sharpen the contrast of neighboring pixels.
Color adjustBrightness(Color color, float brightness) {
    color.r = powf(color.r * 3 * brightness, 2);
    color.g = powf(color.g * 3 * brightness, 2);
    color.b = powf(color.b * 3 * brightness, 2);
    return color;
}

// Sets the pixel at (i, j) to the input Color with the given input brightness.
void setPixel(int i, int j, Image img, Color color, float brightness) {
    if (brightness != -1) color = adjustBrightness(color, brightness);
    img.pixels[i * 3 + j * img.x_pixels * 3    ] = color.r;
    img.pixels[i * 3 + j * img.x_pixels * 3 + 1] = color.g;
    img.pixels[i * 3 + j * img.x_pixels * 3 + 2] = color.b;
}

/*
 Traces a ray for every pixel in the image, filling it in with the appropriate color according to
 the objects present in the space.
*/ 
void display(Image img, TriangleSurface* triangles, int num_triangles, SphereSurface* spheres, int num_spheres, Camera c) {
    for (int i = 0; i < img.x_pixels; i++) {
        for (int j = 0; j < img.y_pixels; j++) {
            Ray r = getRay(i, j, c, img);

            float min_dist = -1;
            Color color = {1, 1, 1}; 

            for (int k = 0; k < num_triangles; k++) {
                Triangle t = triangles[k].t;
                t.p = findPlane(t);
                std::tuple<Point, float> result = triangleIntersect(r, t);
                Point intersect = std::get<0>(result);
                float lmda = std::get<1>(result);
                float dist = distance(intersect, c.pos);

                if (isValidIntersect(intersect, lmda, t) && (dist < min_dist || min_dist == -1)) {
                    color = triangles[k].color;
                    min_dist = dist;
                }

            } 

            for (int k = 0; k < num_spheres; k++) {
                std::tuple<Point, bool> result = sphereIntersect(r, spheres[k].s);
                if (std::get<1>(result)) {
                    Point intersect = std::get<0>(result);
                    float dist = distance(intersect, c.pos);

                    if (dist < min_dist || min_dist == -1) {
                        color = spheres[k].color;
                        min_dist = dist;
                    }
                }
            }
            
            setPixel(i, img.y_pixels - j - 1, img, color, 1/min_dist);
        }
    }
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

        Vector normal = subtractVector(points[i+1], points[i]);
        Vector test = addVector(points[i], arm);
        Vector proj = subtractVector(test, scaleVector(normal, dotProduct(test, normal) / dotProduct(normal, normal)));
        proj = scaleVector(normalizeVector(proj), armSize);

        Point point1a = addVector(points[i], proj);
        Point point2a = subtractVector(points[i], proj);

        Vector cross = crossProduct(normal, proj);
        cross = scaleVector(normalizeVector(cross), armSize);
        Point point3a = addVector(points[i], cross);
        Point point4a = subtractVector(points[i], cross);

        Point point1b = addVector(point1a, normal);
        Point point2b = addVector(point2a, normal);
        Point point3b = addVector(point3a, normal);
        Point point4b = addVector(point4a, normal);

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
        t.p = findPlane(t);
        triangles[i].t = t;
    }

    display(img, triangles, num_triangles, spheres, num_spheres, c);
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

    SphereSurface sphere1 = {{ {1.5, -1.5, 0.0}, 1.0 }, sphereColor};
    SphereSurface sphere2 = {{ {1.5, 1.5, 0.0}, 1.0 }, sphereColor};
    SphereSurface sphere3 = {{ {0.0, 0.0, -1.5}, 1.0 }, sphereColor};
    SphereSurface sphere4 = {{ {-1.5, 0.0, 0.0}, 1.0 }, sphereColor};

    float* pixels = (float*)malloc(viewDimX * viewDimY * 3 * sizeof(float));
    Image img = {pixels, viewDimX, viewDimY, 3, 3}; 

    Camera c = {{3, -0.5, -2}, 1, -40, 0};

    if (argc < 2) {
        std::cerr << "usage: visualize-cpu <input> [output]" << std::endl; 
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