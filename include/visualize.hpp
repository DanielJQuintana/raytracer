#pragma once

struct Point {
    float x;
    float y;
    float z;
};

struct Plane {
    float a;
    float b;
    float c;
    float d;
};

struct Triangle {
    Point a;
    Point b;
    Point c;
    Plane p;
};

struct Sphere {
    Point center;
    float radius;
};

struct Color {
    float r;
    float g;
    float b;
};

struct TriangleSurface {
    Triangle t;
    Color color;
};

struct SphereSurface {
    Sphere s;
    Color color;
};

typedef Point Vector;

struct Ray {
    Vector m;
    Vector origin;
};

struct Image {
    float* pixels;
    int x_pixels;
    int y_pixels;
    float x_dim;
    float y_dim;
};

struct Camera {
    Point pos;
    float dist;
    float yaw;
    float pitch;
};

struct Path {
    Point** points;
    int numLinks;
    int numPositions;
};

struct ObstacleList {
    Point* obstacles;
    int numObstacles;
    float obstacleSize;
};
