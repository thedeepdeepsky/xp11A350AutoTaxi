#include "Geo.h"

namespace autotaxi::geo {

double normalize360(double deg) {
    double v = std::fmod(deg, 360.0);
    if (v < 0.0) v += 360.0;
    return v;
}

double diffSignedDeg(double targetDeg, double currentDeg) {
    double d = normalize360(targetDeg) - normalize360(currentDeg);
    while (d > 180.0) d -= 360.0;
    while (d < -180.0) d += 360.0;
    return d;
}

double distanceMeters(double lat1Deg, double lon1Deg, double lat2Deg, double lon2Deg) {
    const double lat1 = deg2rad(lat1Deg);
    const double lat2 = deg2rad(lat2Deg);
    const double dLat = deg2rad(lat2Deg - lat1Deg);
    const double dLon = deg2rad(lon2Deg - lon1Deg);

    const double a = std::sin(dLat / 2.0) * std::sin(dLat / 2.0)
                   + std::cos(lat1) * std::cos(lat2)
                   * std::sin(dLon / 2.0) * std::sin(dLon / 2.0);
    const double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
    return kEarthRadiusM * c;
}

double bearingDeg(double lat1Deg, double lon1Deg, double lat2Deg, double lon2Deg) {
    const double lat1 = deg2rad(lat1Deg);
    const double lat2 = deg2rad(lat2Deg);
    const double dLon = deg2rad(lon2Deg - lon1Deg);

    const double y = std::sin(dLon) * std::cos(lat2);
    const double x = std::cos(lat1) * std::sin(lat2)
                   - std::sin(lat1) * std::cos(lat2) * std::cos(dLon);
    return normalize360(rad2deg(std::atan2(y, x)));
}

double knotsToMps(double knots) {
    return knots * kKnotToMps;
}

void latLonToLocalMeters(double refLatDeg, double refLonDeg,
                         double latDeg, double lonDeg,
                         double& xEastM, double& yNorthM) {
    const double refLat = deg2rad(refLatDeg);
    xEastM = deg2rad(lonDeg - refLonDeg) * kEarthRadiusM * std::cos(refLat);
    yNorthM = deg2rad(latDeg - refLatDeg) * kEarthRadiusM;
}

} // namespace autotaxi::geo
