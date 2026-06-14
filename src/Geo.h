#pragma once
#include <cmath>

namespace autotaxi::geo {

constexpr double kPi = 3.14159265358979323846;
constexpr double kEarthRadiusM = 6371000.0;
constexpr double kNmToM = 1852.0;
constexpr double kKnotToMps = 0.5144444444444445;

inline double deg2rad(double deg) { return deg * kPi / 180.0; }
inline double rad2deg(double rad) { return rad * 180.0 / kPi; }

double normalize360(double deg);
double diffSignedDeg(double targetDeg, double currentDeg);
double distanceMeters(double lat1Deg, double lon1Deg, double lat2Deg, double lon2Deg);
double bearingDeg(double lat1Deg, double lon1Deg, double lat2Deg, double lon2Deg);
double knotsToMps(double knots);
void latLonToLocalMeters(double refLatDeg, double refLonDeg,
                         double latDeg, double lonDeg,
                         double& xEastM, double& yNorthM);

} // namespace autotaxi::geo
