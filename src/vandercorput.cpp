#include <iostream>
#include <vector>
#include <cmath>
#include <functional>

#include "vandercorput.hpp"

std::vector<double> power2sequence(double fraction) {
    int N = std::ceil(-std::log2(fraction));
    if (N < 1) {
        return std::vector<double>();
    }

    double delta = 0.5;
    std::vector<double> section = {0.5};
    std::vector<double> result = section;

    for (int i = 1; i < N; ++i) {
        delta /= 2.0;
        for (double& x : section) {
            result.push_back(x - delta);
            result.push_back(x + delta);
        }
    }

    return result;
}

std::vector<double> sequence(double fraction) {
    std::vector<double> result;

    std::function<std::vector<double>(std::vector<double>)> resort = [&](std::vector<double> list) {
        int k = list.size() / 2;
        if (k > 0) {
            std::vector<double> top = resort(std::vector<double>(list.begin(), list.begin() + k));
            std::vector<double> bot = resort(std::vector<double>(list.begin() + k + 1, list.end()));
            list[0] = list[k];
            for (int i = 1; i < (int)list.size(); i += 2) {
                list[i] = top[(i - 1) / 2];
            }
            for (int i = 2; i < (int)list.size(); i += 2) {
                list[i] = bot[i / 2 - 1];
            }
        }
        return list;
    };

    int n = std::ceil(1.0 / fraction);
    for (int i = 1; i < n; ++i) {
        result.push_back(static_cast<double>(i) / n);
    }
    result = resort(result);

    return result;
}

// int main() {
//     std::cout << "1.3: ";
//     for (double val : power2sequence(1.3)) {
//         std::cout << val << " ";
//     }
//     std::cout << std::endl;

//     std::cout << "0.7: ";
//     for (double val : power2sequence(0.7)) {
//         std::cout << val << " ";
//     }
//     std::cout << std::endl;

//     std::cout << "0.4: ";
//     for (double val : power2sequence(0.4)) {
//         std::cout << val << " ";
//     }
//     std::cout << std::endl;

//     std::cout << "0.3: ";
//     for (double val : power2sequence(0.3)) {
//         std::cout << val << " ";
//     }
//     std::cout << std::endl;

//     std::cout << "0.2: ";
//     for (double val : power2sequence(0.24)) {
//         std::cout << val << " ";
//     }
//     std::cout << std::endl;

//     std::cout << "0.1: ";
//     for (double val : power2sequence(0.1)) {
//         std::cout << val << " ";
//     }
//     std::cout << std::endl;

//     std::cout << "1.3: ";
//     for (double val : sequence(1.3)) {
//         std::cout << val << " ";
//     }
//     std::cout << std::endl;

//     std::cout << "0.7: ";
//     for (double val : sequence(0.7)) {
//         std::cout << val << " ";
//     }
//     std::cout << std::endl;

//     std::cout << "0.4: ";
//     for (double val : sequence(0.4)) {
//         std::cout << val << " ";
//     }
//     std::cout << std::endl;

//     std::cout << "0.3: ";
//     for (double val : sequence(0.3)) {
//         std::cout << val << " ";
//     }
//     std::cout << std::endl;

//     std::cout << "0.2: ";
//     for (double val : sequence(0.24)) {
//         std::cout << val << " ";
//     }
//     std::cout << std::endl;

//     std::cout << "0.1: ";
//     for (double val : sequence(0.1)) {
//         std::cout << val << " ";
//     }
//     std::cout << std::endl;

//     std::cout << "0.0625: ";
//     for (double val : sequence(0.0625)) {
//         std::cout << val << " ";
//     }
//     std::cout << std::endl;

//     return 0;
// }
