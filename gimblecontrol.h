#include <vector>
#include <list>
#include <cmath>
#include <optional>
#include <cstdio>

namespace gimbal {
constexpr double kPi       = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr float  kGimbalHomeDeg = 45.0f;   // gimbal "linkage" angle when the props are at 0 deg
 
// Wrap an angle to [-pi, pi]
inline double wrapToPi(double x) { return std::remainder(x, 2.0 * kPi); }
}


float GimbleAdj(std::vector<float>&vec, float th2PrevDeg, float desLambda) {

    /*  Four Bar Linkage Mechanism
    L1 is the imaginary const. linkage between the servo rotation axis and the fisheye bearing center 
    L2 and L3 are the real linkages
    G is the imaginary linkage between the fisheye bearing and the gible ball joint connecting to L3

    L1, L2, L3, G = const
    initial Theta = 
    Create a triangle between L3 and G: Law of cosines:
    */

    int gimblehome = 45; //deg of the gimble "linkage" when the props are at 0deg
    float L1 = vec[1];
    float L2 = vec[2];
    float L3 = vec[3];
    float G = vec[4];
    double pi = 3.14159265358979323846;
    // Wrap an angle to [-pi, pi]
    inline double wrapToPi(double x) { 
        return std::remainder(x, 2.0 * pi);
    }

    // Freudenstein Equation:
    float k1 = L1/L2; 
    float k2 = L1/G; 
    float k3 = ((L2*L2) - (L3*L3) + (G*G) + (L1*L1))/(2*L2*G);
    float A = -1*(k2 + std::cos(desLambda * (2*pi/180)));
    float B = -1*(std::sin(desLambda * (2*pi/180)));
    float C = k1 * std::cos(desLambda * (2*pi/180)) + k3;
    double disc = (A*A) + (B*B) - (C*C);

    if (disc < 0.0) return std::nullopt;   // unreachable gimbal angle
 
    const double s  = std::sqrt(disc);
    const double r1 = 2.0 * std::atan2(-B + s, C - A);
    const double r2 = 2.0 * std::atan2(-B - s, C - A);
 
    // Pick whichever root is closer to the previous value (stay on the same branch)
    const double prev = th2PrevDeg * gimbal::kDegToRad;
    const double r = (std::fabs(gimbal::wrapToPi(r1 - prev)) <=
                      std::fabs(gimbal::wrapToPi(r2 - prev))) ? r1 : r2;
 
    return static_cast<float>(gimbal::wrapToPi(r) * gimbal::kRadToDeg);

};