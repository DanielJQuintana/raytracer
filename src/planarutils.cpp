#include <iostream>
#include <cmath>
#include <vector>
#include <tuple>

#include "planarutils.hpp"

bool PointNearPoint(double d, std::vector<double> pA, std::vector<double> pB) {
    return ((pA[0] - pB[0]) * (pA[0] - pB[0]) + (pA[1] - pB[1]) * (pA[1] - pB[1]) <= d * d);
}

bool PointNearSegment(double d, std::vector<double> p, std::vector<std::vector<double>> s) {
    double vx = s[1][0] - s[0][0];
    double vy = s[1][1] - s[0][1];
    double rx = p[0] - s[0][0];
    double ry = p[1] - s[0][1];

    double rTv = rx * vx + ry * vy;
    double rTr = rx * rx + ry * ry;
    double vTv = vx * vx + vy * vy;

    if (rTv <= 0) {
        return (rTr <= d * d);
    }
    if (rTv >= vTv) {
        return (rTr - 2 * rTv + vTv <= d * d);
    }

    return ((rx * vy - ry * vx) * (rx * vy - ry * vx) <= vTv * d * d);
}

bool SegmentCrossSegment(std::vector<std::vector<double>> sA, std::vector<std::vector<double>> sB) {
    double ux = sA[1][0] - sA[0][0];
    double uy = sA[1][1] - sA[0][1];
    double vx = sB[1][0] - sB[0][0];
    double vy = sB[1][1] - sB[0][1];
    double rx = sB[0][0] - sA[0][0];
    double ry = sB[0][1] - sA[0][1];

    double uXv = ux * vy - uy * vx;
    double rXu = rx * uy - ry * ux;
    double rXv = rx * vy - ry * vx;

    if (uXv > 0) {
        return ((rXu > 0) && (rXu < uXv) && (rXv > 0) && (rXv < uXv));
    } else {
        return ((rXu < 0) && (rXu > uXv) && (rXv < 0) && (rXv > uXv));
    }
}

std::pair<bool, bool> EndpointsNearSegmentInterior(double d, std::vector<std::vector<double>> sA, std::vector<std::vector<double>> sB) {
    double vx = sB[1][0] - sB[0][0];
    double vy = sB[1][1] - sB[0][1];
    double r1x = sA[0][0] - sB[0][0];
    double r1y = sA[0][1] - sB[0][1];
    double r2x = sA[1][0] - sB[0][0];
    double r2y = sA[1][1] - sB[0][1];

    double r1Xv = r1x * vy - r1y * vx;
    double r2Xv = r2x * vy - r2y * vx;

    if (r1Xv * r2Xv <= 0) {
        return std::make_pair(true, false);
    }

    double vTv = vx * vx + vy * vy;
    if (std::abs(r2Xv) < std::abs(r1Xv)) {
        double r2Tv = r2x * vx + r2y * vy;
        return std::make_pair(false, (r2Tv >= 0) && (r2Tv <= vTv) && (r2Xv * r2Xv <= vTv * d * d));
    } else {
        double r1Tv = r1x * vx + r1y * vy;
        return std::make_pair(false, (r1Tv >= 0) && (r1Tv <= vTv) && (r1Xv * r1Xv <= vTv * d * d));
    }
}

bool SegmentNearSegment(double d, std::vector<std::vector<double>> sA, std::vector<std::vector<double>> sB) {
    if (PointNearPoint(d, sA[0], sB[0]) || PointNearPoint(d, sA[0], sB[1]) ||
        PointNearPoint(d, sA[1], sB[0]) || PointNearPoint(d, sA[1], sB[1])) {
        return true;
    }

    bool cross1, near;
    std::tie(cross1, near) = EndpointsNearSegmentInterior(d, sA, sB);
    if (near) {
        return true;
    }

    bool cross2;
    std::tie(cross2, near) = EndpointsNearSegmentInterior(d, sB, sA);
    if (near) {
        return true;
    }

    return (cross1 && cross2);
}

bool SegmentNearSphere(double d, std::vector<std::vector<double>> sA, std::vector<double> sphere) {
    
    std::vector<double> segmentStart = sA[0];
    std::vector<double> segmentEnd = sA[1];
    std::vector<double> sphereCenter = { sphere[0], sphere[1], sphere[2] };
    double sphereRadius = sphere[3];

    // Calculate the direction vector of the segment
    double segmentDirectionX = segmentEnd[0] - segmentStart[0];
    double segmentDirectionY = segmentEnd[1] - segmentStart[1];
    double segmentDirectionZ = segmentEnd[2] - segmentStart[2];

    // Calculate the vector from the segment's start point to the sphere's center
    double vectorX = sphereCenter[0] - segmentStart[0];
    double vectorY = sphereCenter[1] - segmentStart[1];
    double vectorZ = sphereCenter[2] - segmentStart[2];

    // Calculate the projection of the vector onto the segment's direction
    double projection = (vectorX * segmentDirectionX) + (vectorY * segmentDirectionY) + (vectorZ * segmentDirectionZ);

    if (projection <= 0.0) {
        // The sphere is behind the segment's start point
        return false;
    }

    // Calculate the length of the segment
    double segmentLength = std::sqrt(
        std::pow(segmentDirectionX, 2) +
        std::pow(segmentDirectionY, 2) +
        std::pow(segmentDirectionZ, 2)
    );

    if (projection >= segmentLength + d) {
        // The sphere is past the segment's end point
        return false;
    }

    // Calculate the closest point on the segment to the sphere's center
    double closestPointX = segmentStart[0] + (segmentDirectionX * (projection / segmentLength));
    double closestPointY = segmentStart[1] + (segmentDirectionY * (projection / segmentLength));
    double closestPointZ = segmentStart[2] + (segmentDirectionZ * (projection / segmentLength));

    // Calculate the distance between the closest point and the sphere's center
    double distance = std::sqrt(
        std::pow(closestPointX - sphereCenter[0], 2) +
        std::pow(closestPointY - sphereCenter[1], 2) +
        std::pow(closestPointZ - sphereCenter[2], 2)
    );

    if (distance <= sphereRadius + d) {
        // The segment is near the sphere
        return true;
    }

    // The segment is not near the sphere
    return false;
}


// bool PointInTriangle(std::vector<double> p, std::vector<std::vector<double>> t) {
//     double apx = t[0][0] - p[0];
//     double apy = t[0][1] - p[1];
//     double bpx = t[1][0] - p[0];
//     double bpy = t[1][1] - p[1];
//     double cpx = t[2][0] - p[0];
//     double cpy = t[2][1] - p[1];

//     double aXb = apx * bpy - apy * bpx;
//     double bXc = bpx * cpy - bpy * cpx;
//     double cXa = cpx * apy - cpy * apx;

//     if (aXb + bXc + cXa > 0) {
//         return (aXb >= 0) && (bXc >= 0) && (cXa >= 0);
//     } else {
//         return (aXb <= 0) && (bXc <= 0) && (cXa <= 0);
//     }
// }

// bool PointNearTriangle(double d, std::vector<double> p, std::vector<std::vector<double>> t) {
//     if (PointInTriangle(p, t)) {
//         return true;
//     }

//     return (PointNearSegment(d, p, {t[0], t[1]}) ||
//             PointNearSegment(d, p, {t[1], t[2]}) ||
//             PointNearSegment(d, p, {t[2], t[0]}));
// }

// bool SegmentCrossTriangle(std::vector<std::vector<double>> segment, std::vector<std::vector<double>> triangle) {
//     return (PointInTriangle(segment[0], triangle) ||
//             PointInTriangle(segment[1], triangle) ||
//             SegmentCrossSegment(segment, {triangle[0], triangle[1]}) ||
//             SegmentCrossSegment(segment, {triangle[1], triangle[2]}) ||
//             SegmentCrossSegment(segment, {triangle[2], triangle[0]}));
// }

// bool SegmentNearTriangle(double d, std::vector<std::vector<double>> segment, std::vector<std::vector<double>> triangle) {
//     return (PointInTriangle(segment[0], triangle) ||
//             PointInTriangle(segment[1], triangle) ||
//             SegmentNearSegment(d, segment, {triangle[0], triangle[1]}) ||
//             SegmentNearSegment(d, segment, {triangle[1], triangle[2]}) ||
//             SegmentNearSegment(d, segment, {triangle[2], triangle[0]}));
// }
