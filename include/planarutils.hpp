#include <iostream>
#include <cmath>
#include <vector>
#include <tuple>

bool PointNearPoint(double d, std::vector<double> pA, std::vector<double> pB);

bool PointNearSegment(double d, std::vector<double> p, std::vector<std::vector<double>> s);

bool SegmentCrossSegment(std::vector<std::vector<double>> sA, std::vector<std::vector<double>> sB);

std::pair<bool, bool> EndpointsNearSegmentInterior(double d, std::vector<std::vector<double>> sA, std::vector<std::vector<double>> sB);

bool SegmentNearSegment(double d, std::vector<std::vector<double>> sA, std::vector<std::vector<double>> sB);

bool SegmentNearSphere(double d, std::vector<std::vector<double>> sA, std::vector<double> sphere);

// bool PointInTriangle(std::vector<double> p, std::vector<std::vector<double>> t);

// bool PointNearTriangle(double d, std::vector<double> p, std::vector<std::vector<double>> t);

// bool SegmentCrossTriangle(std::vector<std::vector<double>> segment, std::vector<std::vector<double>> triangle);

// bool SegmentNearTriangle(double d, std::vector<std::vector<double>> segment, std::vector<std::vector<double>> triangle);
