#pragma once
#include <array>
#include <cmath>
#include <limits>

namespace tb3_burger {
using Quaternion = std::array<double, 4>; // x, y, z, w
using Vector = std::array<double, 3>;
using Covariance = std::array<double, 36>;
// Raw MuJoCo IMU site: z=.068 from base_frame. Requested ROS imu_link:
// base_footprint -> base_link .010 + base_link -> imu_link .068 = .078.
constexpr double ImuSourceToTargetZ = .010;
inline Vector cross(const Vector& a, const Vector& b) {
    return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};
}
inline Vector accelerationAtRosImu(const Vector& raw, const Vector& omega, const Vector& alpha) {
    const Vector offset{0,0,ImuSourceToTargetZ};
    const auto tangential=cross(alpha,offset);
    const auto centripetal=cross(omega,cross(omega,offset));
    return {raw[0]+tangential[0]+centripetal[0],raw[1]+tangential[1]+centripetal[1],raw[2]+tangential[2]+centripetal[2]};
}
struct AngularAccelerationEstimator {
    bool ready=false;
    double previous_stamp=0;
    Vector previous{};
    bool update(double stamp, const Vector& omega, Vector& alpha) {
        alpha={0,0,0};
        if (!std::isfinite(stamp) || stamp<0) { ready=false;return false; }
        for (double v:omega) if (!std::isfinite(v)) {ready=false;return false;}
        const double dt=stamp-previous_stamp;
        const bool valid=ready && dt>0 && dt<=.2;
        if (valid) for(int i=0;i<3;++i) alpha[i]=(omega[i]-previous[i])/dt;
        previous=omega;previous_stamp=stamp;ready=true;
        return valid;
    }
};
inline bool normalize(Quaternion& q) {
    double squared = 0;
    for (double v : q) { if (!std::isfinite(v)) return false; squared += v*v; }
    const double norm = std::sqrt(squared);
    if (!std::isfinite(norm) || norm <= std::numeric_limits<double>::epsilon()) return false;
    for (double& v : q) v /= norm;
    return true;
}
inline double yaw(const Quaternion& q) {
    return std::atan2(2*(q[3]*q[2] + q[0]*q[1]), 1-2*(q[1]*q[1]+q[2]*q[2]));
}
inline Vector worldToFootprint(const Vector& v, double heading) {
    const double c = std::cos(heading), s = std::sin(heading);
    return {c*v[0]+s*v[1], -s*v[0]+c*v[1], v[2]};
}
inline Covariance worldToFootprintCovariance(const Covariance& source, double heading) {
    double a[6][6]{};
    const double c = std::cos(heading), s = std::sin(heading);
    for (int offset : {0,3}) {
        a[offset][offset]=c; a[offset][offset+1]=s;
        a[offset+1][offset]=-s; a[offset+1][offset+1]=c;
        a[offset+2][offset+2]=1;
    }
    Covariance result{};
    for(int row=0;row<6;++row) for(int col=0;col<6;++col)
        for(int i=0;i<6;++i) for(int j=0;j<6;++j)
            result[row*6+col] += a[row][i]*source[i*6+j]*a[col][j];
    return result;
}
}
