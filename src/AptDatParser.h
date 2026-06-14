#pragma once
#include <string>
#include <unordered_map>
#include <vector>

namespace autotaxi {

struct TaxiNode {
    int id = -1;
    double lat = 0.0;
    double lon = 0.0;
    std::string usage;
    std::string name;
};

struct TaxiEdge {
    int from = -1;
    int to = -1;
    bool bidirectional = true;
    bool runway = false;
    bool groundVehicleOnly = false;
    bool activeZone = false;
    std::string ident;
    std::vector<std::string> activeRunways;
};

struct RunwayEnd {
    std::string name;
    double lat = 0.0;
    double lon = 0.0;
};

struct Runway {
    std::string end1;
    double lat1 = 0.0;
    double lon1 = 0.0;
    std::string end2;
    double lat2 = 0.0;
    double lon2 = 0.0;
};

struct RampStart {
    double lat = 0.0;
    double lon = 0.0;
    double headingTrue = 0.0;
    std::string type;
    std::string aircraftTypes;
    std::string name;
    std::string widthCode;
    std::string operationType;
    std::vector<std::string> airlines;
};

struct AirportData {
    std::string headerId;
    std::string icao;
    std::string name;
    std::string sourceFile;
    int sourcePriority = 999999; // Lower means higher scenery priority.

    std::unordered_map<int, TaxiNode> nodes;
    std::vector<TaxiEdge> edges;
    std::vector<RampStart> ramps;
    std::vector<Runway> runways;

    double centerLat = 0.0;
    double centerLon = 0.0;
    double distanceToCurrentM = 1e100;

    bool valid() const {
        if (nodes.empty()) return false;
        for (const auto& e : edges) {
            if (!e.groundVehicleOnly) return true;
        }
        return false;
    }
};

struct AirportSearchOptions {
    std::string wantedIcao;
    double currentLat = 0.0;
    double currentLon = 0.0;
    double maxDistanceM = 8.0 * 1852.0;
    double sameAirportPriorityToleranceM = 700.0;
};

class AptDatParser {
public:
    static bool parseBestAirportFromFiles(const std::vector<std::string>& aptDatFiles,
                                          const AirportSearchOptions& options,
                                          AirportData& outAirport,
                                          std::vector<std::string>& logLines);

private:
    static bool parseFile(const std::string& path,
                          int sourcePriority,
                          const AirportSearchOptions& options,
                          AirportData& outAirport,
                          std::vector<std::string>& logLines,
                          bool& exactIcaoFound);
};

} // namespace autotaxi
