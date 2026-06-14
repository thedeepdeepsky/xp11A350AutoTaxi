#pragma once
#include "AptDatParser.h"
#include "Config.h"

#include "XPLMDataAccess.h"
#include "XPLMUtilities.h"

#include <functional>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

namespace autotaxi {

enum class DestinationKind {
    Ramp,
    Runway,
    Node,
    Coordinate
};

struct DestinationChoice {
    DestinationKind kind = DestinationKind::Ramp;
    std::string label;       // UI text, e.g. "Gate A10" or "Runway 34R".
    std::string detail;      // Secondary info for UI.
    std::string group;       // RUNWAY / GATE / RAMP / GPS / NODE.
    std::string rampName;
    std::string runwayEnd;
    int nodeId = -1;
    double lat = std::numeric_limits<double>::quiet_NaN();
    double lon = std::numeric_limits<double>::quiet_NaN();
    bool heavyCompatible = true; // false for width codes smaller than ICAO E when known.
};

struct StatusSnapshot {
    bool initialized = false;
    bool airportLoaded = false;
    bool active = false;
    bool arrived = false;
    bool routeReady = false;
    bool controlsReady = false;

    std::string airportId;
    std::string airportName;
    std::string sourceFile;
    int sourcePriority = -1;
    int aptDatCandidateCount = 0;
    double airportDistanceM = 0.0;

    double aircraftLat = 0.0;
    double aircraftLon = 0.0;
    double aircraftHeadingDeg = 0.0;
    double aircraftGroundSpeedKts = 0.0;

    int nearestNodeId = -1;
    std::string nearestNodeName;
    double nearestNodeDistanceM = 0.0;
    std::string nearestRampName;
    std::string nearestRampType;
    double nearestRampDistanceM = 0.0;
    std::string nearestEdgeIdent;
    bool nearestEdgeRunway = false;
    double nearestEdgeDistanceM = 0.0;
    std::string positionLabel;

    int taxiNodeCount = 0;
    int taxiEdgeCount = 0;
    int rampCount = 0;
    int runwayEndCount = 0;
    int routeNodeCount = 0;
    int waypointIndex = 0;

    double routeDistanceM = 0.0;
    double routeEtaSec = 0.0;
    double routeProgressPct = 0.0;
    double nextWaypointDistanceM = 0.0;
    std::string routeSummary;
    std::string routeSafety;
    std::string nextInstruction;

    double lastSteerCmd = 0.0;
    double lastThrottleCmd = 0.0;
    double lastTargetSpeedKts = 0.0;
    std::string throttleMode;
    double lastLeftBrakeCmd = 0.0;
    double lastRightBrakeCmd = 0.0;
    double lastFctlRudderCmd = 0.0;
    double lastFctlRollCmd = 0.0;
    double stuckSeconds = 0.0;
    double lastCrossTrackErrorM = 0.0;
    double lastPathHeadingErrorDeg = 0.0;
    double lastPidOutputDeg = 0.0;
    int routeSegmentIndex = 0;

    int selectedDestinationIndex = -1;
    int destinationCount = 0;
    std::string selectedDestinationLabel;
    std::string selectedDestinationDetail;
    std::string lastMessage;
};

class AutoTaxiSystem {
public:
    using Logger = std::function<void(const std::string&)>;

    AutoTaxiSystem(std::string xplaneRoot, std::string pluginRoot, Logger logger);

    bool initialize();
    bool reload();
    bool toggle();
    bool planSelectedDestination();
    bool startSelectedDestination();
    void stopByUser();
    float flightLoop(float elapsedSeconds);

    bool hasAirport() const;
    bool isActive() const;
    bool hasRoute() const;
    const std::vector<DestinationChoice>& destinationChoices() const;
    int selectedDestinationIndex() const;
    bool selectDestinationIndex(int index);
    StatusSnapshot snapshot() const;

private:
    struct AdjEdge {
        int from = -1;
        int to = -1;
        double costM = 0.0;
        std::string ident;
        bool runway = false;
        bool activeZone = false;
    };

    struct EdgeHit {
        bool valid = false;
        int from = -1;
        int to = -1;
        double distanceM = std::numeric_limits<double>::infinity();
        double along01 = 0.0;
        std::string ident;
        bool runway = false;
        bool activeZone = false;
    };

    struct RouteTrack {
        bool valid = false;
        size_t segmentIndex = 0;
        double segmentLengthM = 0.0;
        double along01 = 0.0;
        double distanceM = std::numeric_limits<double>::infinity();
        double signedCrossTrackM = 0.0;
        double pathHeadingDeg = 0.0;
        double lookaheadXEastM = 0.0;
        double lookaheadYNorthM = 0.0;
        double desiredTrackDeg = 0.0;
        double distanceToSegmentEndM = 0.0;
    };

    std::string xplaneRoot_;
    std::string pluginRoot_;
    std::string iniPath_;
    Logger log_;

    AutoTaxiConfig cfg_;
    AirportData airport_;
    std::vector<int> route_;
    size_t waypointIndex_ = 0;
    size_t routeSegmentIndex_ = 0;
    std::vector<DestinationChoice> destinations_;
    int selectedDestinationIndex_ = -1;

    bool initialized_ = false;
    bool active_ = false;
    bool arrived_ = false;
    double smoothedSteer_ = 0.0;
    double pidIntegralDegSec_ = 0.0;
    double pidPrevHeadingErrDeg_ = 0.0;
    double pidDerivativeFilteredDegPerSec_ = 0.0;
    bool pidPrevValid_ = false;
    double lastCrossTrackErrorM_ = 0.0;
    double lastPathHeadingErrorDeg_ = 0.0;
    double lastPidOutputDeg_ = 0.0;
    double prevSignedCrossTrackM_ = 0.0;
    double filteredXteRateMps_ = 0.0;
    bool prevXteValid_ = false;
    int offRouteReplanCooldown_ = 0;
    int aptDatCandidateCount_ = 0;

    double routeDistanceM_ = 0.0;
    std::string routeSummary_;
    std::string routeSafety_;
    int lastRouteDestNode_ = -1;
    mutable std::string lastMessage_;

    XPLMDataRef drLat_ = nullptr;
    XPLMDataRef drLon_ = nullptr;
    XPLMDataRef drPsi_ = nullptr;
    XPLMDataRef drGroundSpeed_ = nullptr;
    XPLMDataRef drOnGroundAny_ = nullptr;
    XPLMDataRef drAircraftIcao_ = nullptr;
    XPLMDataRef drAircraftDesc_ = nullptr;

    XPLMDataRef drSteer_ = nullptr;
    XPLMDataRef drFctlRudder_ = nullptr;
    XPLMDataRef drFctlRoll_ = nullptr;
    XPLMDataRef drFctlRudderLegacy_ = nullptr;
    XPLMDataRef drFctlRollLegacy_ = nullptr;
    XPLMDataRef drFctlFlapRatio_ = nullptr;
    XPLMDataRef drFctlFlapRequest_ = nullptr;
    XPLMDataRef drFctlSlatRatio_ = nullptr;
    XPLMDataRef drThrottleAll_ = nullptr;
    XPLMDataRef drThrottleArray_ = nullptr;
    XPLMDataRef drLeftBrake_ = nullptr;
    XPLMDataRef drRightBrake_ = nullptr;
    XPLMDataRef drLeftBrakeRatio_ = nullptr;
    XPLMDataRef drRightBrakeRatio_ = nullptr;
    XPLMDataRef drParkingBrake_ = nullptr;
    XPLMDataRef drParkingBrakeLegacy_ = nullptr;

    XPLMCommandRef cmdBrakeRelease_ = nullptr;
    XPLMCommandRef cmdBrakesRegular_ = nullptr;
    XPLMCommandRef cmdBrakesMax_ = nullptr;
    XPLMCommandRef cmdThrottleUp_ = nullptr;
    XPLMCommandRef cmdThrottleDown_ = nullptr;
    XPLMCommandRef cmdThrottleIdle_ = nullptr;

    double activeTimeSec_ = 0.0;
    double stuckSeconds_ = 0.0;
    double lastSteerCmd_ = 0.0;
    double lastThrottleCmd_ = 0.0;
    double lastTargetSpeedKts_ = 0.0;
    std::string throttleMode_;
    double lastLeftBrakeCmd_ = 0.0;
    double lastRightBrakeCmd_ = 0.0;
    double lastFctlRudderCmd_ = 0.0;
    double lastFctlRollCmd_ = 0.0;
    bool secondaryFctlBaselineCaptured_ = false;
    double savedFlapRatio_ = 0.0;
    double savedFlapRequest_ = 0.0;
    double savedSlatRatio_ = 0.0;
    bool secondaryFctlAssistHasCommanded_ = false;
    double commandAssistTimer_ = 0.0;
    double prevFinalDistM_ = std::numeric_limits<double>::infinity();
    double finalDistIncreasingSec_ = 0.0;

    bool resolveDatarefs();
    void resolveControlDatarefs();
    void resolveCommandRefs();
    bool controlsReady() const;
    bool isAircraftAllowed() const;

    std::vector<std::string> collectAptDatCandidates() const;
    bool loadAirport();
    void rebuildDestinationChoices();
    int nodeForDestination(const DestinationChoice& choice) const;
    int nodeForRunwayEnd(const std::string& runwayEnd, double fallbackLat, double fallbackLon) const;
    bool buildRoute(double currentLat, double currentLon, bool userVisibleLog);
    int nearestNode(double lat, double lon, double* outDistanceM = nullptr) const;
    EdgeHit nearestEdge(double lat, double lon) const;
    int destinationNode() const;
    int fallbackDestinationNode(int startNode) const;
    bool runAStar(int startNode, int destNode, std::vector<int>& outRoute) const;
    std::vector<AdjEdge> adjacencyForNode(int nodeId) const;
    const TaxiEdge* findEdgeBetween(int from, int to) const;
    double routeRemainingDistanceM(double lat, double lon) const;
    void pruneTerminalRouteForArrival(bool userVisibleLog);
    size_t lookAheadWaypointIndex(double lat, double lon) const;
    double upcomingTurnDeg(size_t atIndex) const;
    RouteTrack computeRouteTrack(double lat, double lon, size_t segmentIndex, double lookaheadM) const;
    RouteTrack currentRouteTrack(double lat, double lon, double lookaheadM);
    void resetPidController();
    void computeRouteMetrics();
    bool startSafetyCheck(double lat, double lon) const;

    void updateControl(double dt);
    void writeControls(double steerRatio, double throttle, double leftBrake, double rightBrake);
    void writeFctlCoupling(double steerRatio);
    void captureFctlSecondaryBaseline();
    void writeFctlSecondaryAssist(double tightTurnBlend);
    void releaseFctlControls();
    void releaseFctlSecondaryAssist();
    void writeParkingBrakesReleased();
    void releaseBrakeCommands();
    void pulseThrottleAssist(double throttle, double groundSpeedMps, double dt);
    void releaseControls(bool releaseParkingBrake);
    void arriveAndHold();

    std::string readStringDataRef(XPLMDataRef ref) const;
    double getDataf(XPLMDataRef ref, double def = 0.0) const;
    void log(const std::string& s) const;
};

} // namespace autotaxi
