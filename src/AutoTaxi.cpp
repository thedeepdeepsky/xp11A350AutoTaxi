#include "AutoTaxi.h"
#include "Geo.h"

#include "XPLMUtilities.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <queue>
#include <set>
#include <sstream>
#include <unordered_set>

namespace fs = std::filesystem;

namespace autotaxi {
namespace {

template<typename T>
T clamp(T v, T lo, T hi) {
    return std::max(lo, std::min(hi, v));
}

bool finiteLatLon(double lat, double lon) {
    return std::isfinite(lat) && std::isfinite(lon)
        && lat >= -90.0 && lat <= 90.0
        && lon >= -180.0 && lon <= 180.0;
}

std::string normalizePathString(const fs::path& p) {
    return p.lexically_normal().generic_string();
}

bool startsWith(const std::string& s, const std::string& prefix) {
    if (s.size() < prefix.size()) return false;
    return std::equal(prefix.begin(), prefix.end(), s.begin());
}

double widthRank(const std::string& widthCode) {
    if (widthCode.empty()) return 999.0;
    const char c = static_cast<char>(std::toupper(static_cast<unsigned char>(widthCode[0])));
    if (c < 'A' || c > 'F') return 999.0;
    return static_cast<double>(c - 'A');
}

bool isHeavyCompatibleWidth(const std::string& widthCode) {
    if (widthCode.empty()) return true; // Unknown: do not hide, just treat as usable.
    return widthRank(widthCode) >= widthRank("E");
}

std::string fmt1(double v) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(1) << v;
    return oss.str();
}

std::string fmt0(double v) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(0) << v;
    return oss.str();
}

std::string baseName(const std::string& path) {
    try {
        return fs::path(path).filename().generic_string();
    } catch (...) {
        return path;
    }
}

std::string destinationSortKey(const DestinationChoice& d) {
    int group = 9;
    switch (d.kind) {
        case DestinationKind::Runway: group = 0; break;
        case DestinationKind::Ramp: group = d.heavyCompatible ? 1 : 2; break;
        case DestinationKind::Node: group = 3; break;
        case DestinationKind::Coordinate: group = 4; break;
    }
    return std::to_string(group) + "|" + toUpper(d.label);
}

std::string airportDisplayId(const AirportData& ap) {
    if (!ap.icao.empty()) return ap.icao;
    return ap.headerId;
}

double dot2(double ax, double ay, double bx, double by) {
    return ax * bx + ay * by;
}

} // namespace

AutoTaxiSystem::AutoTaxiSystem(std::string xplaneRoot, std::string pluginRoot, Logger logger)
    : xplaneRoot_(std::move(xplaneRoot)),
      pluginRoot_(std::move(pluginRoot)),
      iniPath_(normalizePathString(fs::path(pluginRoot_) / "A350AutoTaxi.ini")),
      log_(std::move(logger)) {}

bool AutoTaxiSystem::initialize() {
    initialized_ = resolveDatarefs();
    std::string err;
    cfg_ = loadAutoTaxiConfig(iniPath_, err);
    if (!err.empty()) {
        log("[A350AutoTaxi] Config warning: " + err + " ; using defaults where needed.");
    }
    resolveControlDatarefs();
    resolveCommandRefs();
    return initialized_;
}

bool AutoTaxiSystem::resolveDatarefs() {
    drLat_ = XPLMFindDataRef("sim/flightmodel/position/latitude");
    drLon_ = XPLMFindDataRef("sim/flightmodel/position/longitude");
    drPsi_ = XPLMFindDataRef("sim/flightmodel/position/psi");
    drGroundSpeed_ = XPLMFindDataRef("sim/flightmodel/position/groundspeed");
    drOnGroundAny_ = XPLMFindDataRef("sim/flightmodel/failures/onground_any");

    drAircraftIcao_ = XPLMFindDataRef("sim/aircraft/view/acf_ICAO");
    drAircraftDesc_ = XPLMFindDataRef("sim/aircraft/view/acf_descrip");

    const bool ok = drLat_ && drLon_ && drPsi_ && drGroundSpeed_;
    if (!ok) log("[A350AutoTaxi] ERROR: mandatory position datarefs not found.");
    return ok;
}

void AutoTaxiSystem::resolveControlDatarefs() {
    drSteer_ = XPLMFindDataRef(cfg_.steerDataref.c_str());
    drFctlRudder_ = cfg_.fctlRudderDataref.empty() ? nullptr : XPLMFindDataRef(cfg_.fctlRudderDataref.c_str());
    drFctlRoll_ = cfg_.fctlRollDataref.empty() ? nullptr : XPLMFindDataRef(cfg_.fctlRollDataref.c_str());
    drFctlRudderLegacy_ = cfg_.fctlRudderLegacyDataref.empty() ? nullptr : XPLMFindDataRef(cfg_.fctlRudderLegacyDataref.c_str());
    drFctlRollLegacy_ = cfg_.fctlRollLegacyDataref.empty() ? nullptr : XPLMFindDataRef(cfg_.fctlRollLegacyDataref.c_str());
    drFctlFlapRatio_ = cfg_.fctlFlapRatioDataref.empty() ? nullptr : XPLMFindDataRef(cfg_.fctlFlapRatioDataref.c_str());
    drFctlFlapRequest_ = cfg_.fctlFlapRequestDataref.empty() ? nullptr : XPLMFindDataRef(cfg_.fctlFlapRequestDataref.c_str());
    drFctlSlatRatio_ = cfg_.fctlSlatRatioDataref.empty() ? nullptr : XPLMFindDataRef(cfg_.fctlSlatRatioDataref.c_str());
    drThrottleAll_ = XPLMFindDataRef(cfg_.throttleAllDataref.c_str());
    drThrottleArray_ = XPLMFindDataRef(cfg_.throttleArrayDataref.c_str());
    drLeftBrake_ = XPLMFindDataRef(cfg_.leftBrakeDataref.c_str());
    drRightBrake_ = XPLMFindDataRef(cfg_.rightBrakeDataref.c_str());
    drLeftBrakeRatio_ = XPLMFindDataRef("sim/cockpit2/controls/left_brake_ratio");
    drRightBrakeRatio_ = XPLMFindDataRef("sim/cockpit2/controls/right_brake_ratio");
    drParkingBrake_ = XPLMFindDataRef(cfg_.parkingBrakeDataref.c_str());
    drParkingBrakeLegacy_ = XPLMFindDataRef("sim/flightmodel/controls/parkbrake");
}

void AutoTaxiSystem::resolveCommandRefs() {
    cmdBrakeRelease_ = XPLMFindCommand("sim/flight_controls/brakes_release");
    cmdBrakesRegular_ = XPLMFindCommand("sim/flight_controls/brakes_regular");
    cmdBrakesMax_ = XPLMFindCommand("sim/flight_controls/brakes_max");
    cmdThrottleUp_ = XPLMFindCommand("sim/engines/throttle_up");
    cmdThrottleDown_ = XPLMFindCommand("sim/engines/throttle_down");
    cmdThrottleIdle_ = XPLMFindCommand("sim/engines/throttle_idle");
}


bool AutoTaxiSystem::controlsReady() const {
    auto writable = [](XPLMDataRef ref) {
        return ref && XPLMCanWriteDataRef(ref);
    };

    const bool throttleOk = writable(drThrottleAll_) || writable(drThrottleArray_) || cmdThrottleUp_;
    const bool brakeOk = (writable(drLeftBrake_) && writable(drRightBrake_)) ||
                         (writable(drLeftBrakeRatio_) && writable(drRightBrakeRatio_)) ||
                         cmdBrakeRelease_;

    return writable(drSteer_) && throttleOk && brakeOk;
}

bool AutoTaxiSystem::reload() {
    std::string err;
    cfg_ = loadAutoTaxiConfig(iniPath_, err);
    if (!err.empty()) log("[A350AutoTaxi] Config warning: " + err + " ; using defaults where needed.");

    const bool wasActive = active_;
    resolveControlDatarefs();
    resolveCommandRefs();
    if (wasActive) releaseControls(true);

    auto checkWrite = [&](XPLMDataRef ref, const std::string& name) {
        if (!ref) {
            log("[A350AutoTaxi] WARNING: dataref not found: " + name);
        } else if (!XPLMCanWriteDataRef(ref)) {
            log("[A350AutoTaxi] WARNING: dataref is not writable in this aircraft: " + name);
        }
    };

    checkWrite(drSteer_, cfg_.steerDataref);
    if (cfg_.fctlTurnCoupling) {
        if (!cfg_.fctlRudderDataref.empty()) checkWrite(drFctlRudder_, cfg_.fctlRudderDataref);
        if (!cfg_.fctlRollDataref.empty()) checkWrite(drFctlRoll_, cfg_.fctlRollDataref);
        if (!cfg_.fctlRudderLegacyDataref.empty()) checkWrite(drFctlRudderLegacy_, cfg_.fctlRudderLegacyDataref);
        if (!cfg_.fctlRollLegacyDataref.empty()) checkWrite(drFctlRollLegacy_, cfg_.fctlRollLegacyDataref);
        if (cfg_.fctlSecondaryAssist) {
            if (!cfg_.fctlFlapRatioDataref.empty()) checkWrite(drFctlFlapRatio_, cfg_.fctlFlapRatioDataref);
            if (!cfg_.fctlFlapRequestDataref.empty()) checkWrite(drFctlFlapRequest_, cfg_.fctlFlapRequestDataref);
            if (!cfg_.fctlSlatRatioDataref.empty()) checkWrite(drFctlSlatRatio_, cfg_.fctlSlatRatioDataref);
        }
    }
    checkWrite(drThrottleAll_, cfg_.throttleAllDataref);
    checkWrite(drThrottleArray_, cfg_.throttleArrayDataref);
    checkWrite(drLeftBrake_, cfg_.leftBrakeDataref);
    checkWrite(drRightBrake_, cfg_.rightBrakeDataref);
    checkWrite(drLeftBrakeRatio_, "sim/cockpit2/controls/left_brake_ratio");
    checkWrite(drRightBrakeRatio_, "sim/cockpit2/controls/right_brake_ratio");
    checkWrite(drParkingBrake_, cfg_.parkingBrakeDataref);
    checkWrite(drParkingBrakeLegacy_, "sim/flightmodel/controls/parkbrake");

    route_.clear();
    waypointIndex_ = 0;
    routeSegmentIndex_ = 0;
    resetPidController();
    routeDistanceM_ = 0.0;
    routeSummary_.clear();
    routeSafety_.clear();
    lastRouteDestNode_ = -1;
    arrived_ = false;
    active_ = false;
    offRouteReplanCooldown_ = 0;

    return loadAirport();
}

bool AutoTaxiSystem::toggle() {
    if (active_) {
        stopByUser();
        return true;
    }
    return startSelectedDestination();
}

bool AutoTaxiSystem::planSelectedDestination() {
    if (!initialized_ && !initialize()) return false;

    if (!hasAirport()) {
        if (!reload()) {
            log("[A350AutoTaxi] Cannot plan: no airport taxi network loaded.");
            return false;
        }
    } else {
        std::string err;
        cfg_ = loadAutoTaxiConfig(iniPath_, err);
        if (!err.empty()) log("[A350AutoTaxi] Config warning: " + err + " ; using defaults where needed.");
    }

    const double lat = getDataf(drLat_);
    const double lon = getDataf(drLon_);
    const bool ok = buildRoute(lat, lon, true);
    if (!ok) log("[A350AutoTaxi] Cannot plan: no route from current position to selected destination.");
    return ok;
}

bool AutoTaxiSystem::startSelectedDestination() {
    if (active_) return true;

    if (!initialized_ && !initialize()) return false;

    if (!isAircraftAllowed()) {
        log("[A350AutoTaxi] Refused: current aircraft does not look like A350. Set allow_any_aircraft=true to test.");
        return false;
    }

    resolveControlDatarefs();
    if (!controlsReady()) {
        log("[A350AutoTaxi] Refused: steering/throttle/brake datarefs are not writable. Fix A350AutoTaxi.ini datarefs first.");
        return false;
    }

    if (!hasAirport()) {
        if (!reload()) {
            log("[A350AutoTaxi] Cannot start: no airport taxi network loaded.");
            return false;
        }
        resolveControlDatarefs();
    }

    const double lat = getDataf(drLat_);
    const double lon = getDataf(drLon_);
    if (!startSafetyCheck(lat, lon)) return false;

    if (!buildRoute(lat, lon, true)) {
        log("[A350AutoTaxi] Cannot start: no route from current position to selected destination.");
        return false;
    }

    writeParkingBrakesReleased();
    releaseBrakeCommands();
    secondaryFctlBaselineCaptured_ = false;
    secondaryFctlAssistHasCommanded_ = false;
    captureFctlSecondaryBaseline();

    active_ = true;
    arrived_ = false;
    activeTimeSec_ = 0.0;
    stuckSeconds_ = 0.0;
    smoothedSteer_ = 0.0;
    resetPidController();
    offRouteReplanCooldown_ = 0;
    commandAssistTimer_ = 0.0;

    std::string destLabel = "configured/fallback destination";
    if (selectedDestinationIndex_ >= 0 && selectedDestinationIndex_ < static_cast<int>(destinations_.size())) {
        destLabel = destinations_[selectedDestinationIndex_].label;
    }

    log("[A350AutoTaxi] GSX-mode AutoTaxi ACTIVE airport=" + airportDisplayId(airport_) +
        " destination=" + destLabel +
        " distance_m=" + fmt0(routeDistanceM_) +
        " route=" + routeSummary_);
    return true;
}

void AutoTaxiSystem::stopByUser() {
    active_ = false;
    arrived_ = false;
    route_.clear();
    waypointIndex_ = 0;
    routeSegmentIndex_ = 0;
    resetPidController();
    routeDistanceM_ = 0.0;
    routeSummary_.clear();
    routeSafety_.clear();
    lastRouteDestNode_ = -1;
    releaseControls(true);
    log("[A350AutoTaxi] AutoTaxi stopped by user; controls released.");
}

float AutoTaxiSystem::flightLoop(float elapsedSeconds) {
    if (!initialized_) return 0.5f;
    if (!active_) return 0.25f;

    const double dt = clamp<double>(elapsedSeconds <= 0.0f ? 0.05 : elapsedSeconds, 0.01, 0.20);
    activeTimeSec_ += dt;
    updateControl(dt);
    return 0.05f;
}

bool AutoTaxiSystem::hasAirport() const {
    return airport_.valid();
}

bool AutoTaxiSystem::isActive() const {
    return active_;
}

bool AutoTaxiSystem::hasRoute() const {
    return !route_.empty();
}

const std::vector<DestinationChoice>& AutoTaxiSystem::destinationChoices() const {
    return destinations_;
}

int AutoTaxiSystem::selectedDestinationIndex() const {
    return selectedDestinationIndex_;
}

bool AutoTaxiSystem::selectDestinationIndex(int index) {
    if (index < 0 || index >= static_cast<int>(destinations_.size())) return false;
    selectedDestinationIndex_ = index;
    route_.clear();
    waypointIndex_ = 0;
    routeSegmentIndex_ = 0;
    resetPidController();
    routeDistanceM_ = 0.0;
    routeSummary_.clear();
    routeSafety_.clear();
    lastRouteDestNode_ = -1;
    arrived_ = false;
    log("[A350AutoTaxi] Selected destination: " + destinations_[index].label);
    return true;
}

StatusSnapshot AutoTaxiSystem::snapshot() const {
    StatusSnapshot st;
    st.initialized = initialized_;
    st.airportLoaded = airport_.valid();
    st.active = active_;
    st.arrived = arrived_;
    st.routeReady = !route_.empty();
    st.controlsReady = controlsReady();
    st.lastSteerCmd = lastSteerCmd_;
    st.lastThrottleCmd = lastThrottleCmd_;
    st.lastTargetSpeedKts = lastTargetSpeedKts_;
    st.throttleMode = throttleMode_;
    st.lastLeftBrakeCmd = lastLeftBrakeCmd_;
    st.lastRightBrakeCmd = lastRightBrakeCmd_;
    st.lastFctlRudderCmd = lastFctlRudderCmd_;
    st.lastFctlRollCmd = lastFctlRollCmd_;
    st.stuckSeconds = stuckSeconds_;
    st.lastCrossTrackErrorM = lastCrossTrackErrorM_;
    st.lastPathHeadingErrorDeg = lastPathHeadingErrorDeg_;
    st.lastPidOutputDeg = lastPidOutputDeg_;
    st.routeSegmentIndex = static_cast<int>(routeSegmentIndex_);
    st.lastMessage = lastMessage_;

    st.airportId = airportDisplayId(airport_);
    st.airportName = airport_.name;
    st.sourceFile = airport_.sourceFile;
    st.sourcePriority = airport_.sourcePriority;
    st.aptDatCandidateCount = aptDatCandidateCount_;
    st.airportDistanceM = airport_.distanceToCurrentM;
    st.taxiNodeCount = static_cast<int>(airport_.nodes.size());
    st.taxiEdgeCount = static_cast<int>(airport_.edges.size());
    st.rampCount = static_cast<int>(airport_.ramps.size());
    st.runwayEndCount = static_cast<int>(airport_.runways.size() * 2);
    st.routeNodeCount = static_cast<int>(route_.size());
    st.waypointIndex = static_cast<int>(waypointIndex_);
    st.destinationCount = static_cast<int>(destinations_.size());
    st.routeDistanceM = routeDistanceM_;
    st.routeEtaSec = routeDistanceM_ > 1.0 ? routeDistanceM_ / std::max(0.1, geo::knotsToMps(cfg_.taxiSpeedKts)) : 0.0;
    st.routeSummary = routeSummary_;
    st.routeSafety = routeSafety_;

    if (selectedDestinationIndex_ >= 0 && selectedDestinationIndex_ < static_cast<int>(destinations_.size())) {
        st.selectedDestinationLabel = destinations_[selectedDestinationIndex_].label;
        st.selectedDestinationDetail = destinations_[selectedDestinationIndex_].detail;
    }

    st.aircraftLat = getDataf(drLat_);
    st.aircraftLon = getDataf(drLon_);
    st.aircraftHeadingDeg = getDataf(drPsi_);
    st.aircraftGroundSpeedKts = getDataf(drGroundSpeed_) / geo::kKnotToMps;

    if (airport_.valid()) {
        double bestNodeD = std::numeric_limits<double>::infinity();
        st.nearestNodeId = nearestNode(st.aircraftLat, st.aircraftLon, &bestNodeD);
        st.nearestNodeDistanceM = bestNodeD;
        if (st.nearestNodeId >= 0) {
            const auto it = airport_.nodes.find(st.nearestNodeId);
            if (it != airport_.nodes.end()) st.nearestNodeName = it->second.name;
        }

        double bestRampD = std::numeric_limits<double>::infinity();
        for (const auto& r : airport_.ramps) {
            const double d = geo::distanceMeters(st.aircraftLat, st.aircraftLon, r.lat, r.lon);
            if (d < bestRampD) {
                bestRampD = d;
                st.nearestRampName = r.name;
                st.nearestRampType = r.type;
                st.nearestRampDistanceM = d;
            }
        }

        const EdgeHit edge = nearestEdge(st.aircraftLat, st.aircraftLon);
        if (edge.valid) {
            st.nearestEdgeIdent = edge.ident.empty() ? (edge.runway ? "runway edge" : "taxiway") : edge.ident;
            st.nearestEdgeRunway = edge.runway;
            st.nearestEdgeDistanceM = edge.distanceM;
        }

        if (!st.nearestRampName.empty() && st.nearestRampDistanceM < cfg_.gatePositionRadiusM) {
            st.positionLabel = st.nearestRampType + " " + st.nearestRampName + " (" + fmt1(st.nearestRampDistanceM) + " m)";
        } else if (edge.valid && edge.distanceM < cfg_.taxiwayPositionRadiusM) {
            st.positionLabel = std::string(edge.runway ? "Runway edge " : "Taxiway ") +
                               (st.nearestEdgeIdent.empty() ? "unnamed" : st.nearestEdgeIdent) +
                               " (" + fmt1(edge.distanceM) + " m)";
        } else if (st.nearestNodeId >= 0) {
            st.positionLabel = "Taxi node " + std::to_string(st.nearestNodeId);
            if (!st.nearestNodeName.empty()) st.positionLabel += " / " + st.nearestNodeName;
            st.positionLabel += " (" + fmt1(st.nearestNodeDistanceM) + " m)";
        }

        if (!route_.empty() && waypointIndex_ < route_.size()) {
            const auto& target = airport_.nodes.at(route_[waypointIndex_]);
            st.nextWaypointDistanceM = geo::distanceMeters(st.aircraftLat, st.aircraftLon, target.lat, target.lon);
            const double remaining = routeRemainingDistanceM(st.aircraftLat, st.aircraftLon);
            if (routeDistanceM_ > 1.0) {
                st.routeProgressPct = clamp(100.0 * (routeDistanceM_ - remaining) / routeDistanceM_, 0.0, 100.0);
            }

            const int prevIdx = waypointIndex_ == 0 ? 0 : static_cast<int>(waypointIndex_) - 1;
            if (prevIdx >= 0 && waypointIndex_ < route_.size()) {
                const TaxiEdge* leg = findEdgeBetween(route_[prevIdx], route_[waypointIndex_]);
                const std::string legName = leg && !leg->ident.empty() ? leg->ident : "next node";
                st.nextInstruction = "Follow " + legName + " -> node " + std::to_string(route_[waypointIndex_]) +
                                     " (" + fmt0(st.nextWaypointDistanceM) + " m)";
            }
        }
    }

    return st;
}

bool AutoTaxiSystem::isAircraftAllowed() const {
    if (cfg_.allowAnyAircraft) return true;

    const auto acIcao = toUpper(readStringDataRef(drAircraftIcao_));
    const auto acDesc = toUpper(readStringDataRef(drAircraftDesc_));

    for (auto allowed : cfg_.allowedAircraftIcao) {
        allowed = toUpper(trim(allowed));
        if (allowed.empty()) continue;
        if (acIcao == allowed) return true;
        if (acDesc.find(allowed) != std::string::npos) return true;
    }

    if (acDesc.find("A350") != std::string::npos) return true;
    return false;
}

std::vector<std::string> AutoTaxiSystem::collectAptDatCandidates() const {
    std::vector<std::string> files;
    std::unordered_set<std::string> seen;

    auto addFile = [&](const fs::path& p) {
        const auto s = normalizePathString(p);
        if (seen.find(s) != seen.end()) return;
        if (fs::exists(p) && fs::is_regular_file(p)) {
            files.push_back(s);
            seen.insert(s);
        }
    };

    if (!cfg_.aptDatPath.empty()) {
        fs::path p(cfg_.aptDatPath);
        if (p.is_relative()) p = fs::path(xplaneRoot_) / p;
        addFile(p);
    }

    const fs::path sceneryIni = fs::path(xplaneRoot_) / "Custom Scenery" / "scenery_packs.ini";
    std::ifstream in(sceneryIni);
    if (in) {
        std::string line;
        while (std::getline(in, line)) {
            line = trim(line);
            if (line.empty() || line[0] == '#') continue;
            if (startsWith(line, "SCENERY_PACK_DISABLED")) continue;
            if (!startsWith(line, "SCENERY_PACK")) continue;

            const auto pos = line.find(' ');
            if (pos == std::string::npos) continue;
            auto pack = trim(line.substr(pos + 1));
            if (pack.empty()) continue;

            fs::path packPath(pack);
            if (packPath.is_relative()) packPath = fs::path(xplaneRoot_) / packPath;
            addFile(packPath / "Earth nav data" / "apt.dat");
        }
    }

    // Explicit fallback for installations whose scenery_packs.ini was not generated yet.
    addFile(fs::path(xplaneRoot_) / "Custom Scenery" / "Global Airports" / "Earth nav data" / "apt.dat");
    addFile(fs::path(xplaneRoot_) / "Resources" / "default scenery" / "default apt dat" / "Earth nav data" / "apt.dat");

    return files;
}

bool AutoTaxiSystem::loadAirport() {
    const auto files = collectAptDatCandidates();
    aptDatCandidateCount_ = static_cast<int>(files.size());
    if (files.empty()) {
        log("[A350AutoTaxi] No apt.dat candidate files found.");
        return false;
    }

    AirportSearchOptions opt;
    opt.wantedIcao = cfg_.icao;
    opt.currentLat = getDataf(drLat_);
    opt.currentLon = getDataf(drLon_);
    opt.maxDistanceM = cfg_.airportSearchRadiusNm * geo::kNmToM;
    opt.sameAirportPriorityToleranceM = cfg_.sameAirportPriorityToleranceM;

    std::vector<std::string> logs;
    AirportData ap;
    log("[A350AutoTaxi] apt.dat scan candidates=" + std::to_string(files.size()) +
        " root=" + xplaneRoot_);
    const bool ok = AptDatParser::parseBestAirportFromFiles(files, opt, ap, logs);
    for (const auto& s : logs) log(s);

    if (!ok) {
        if (!cfg_.icao.empty()) {
            log("[A350AutoTaxi] No usable airport taxi network found for ICAO=" + cfg_.icao);
        } else {
            log("[A350AutoTaxi] No usable airport taxi network found within " +
                fmt1(cfg_.airportSearchRadiusNm) + " NM.");
        }
        return false;
    }

    std::string previousLabel;
    if (selectedDestinationIndex_ >= 0 && selectedDestinationIndex_ < static_cast<int>(destinations_.size())) {
        previousLabel = destinations_[selectedDestinationIndex_].label;
    }

    airport_ = std::move(ap);
    rebuildDestinationChoices();

    selectedDestinationIndex_ = destinations_.empty() ? -1 : 0;
    if (!previousLabel.empty()) {
        for (int i = 0; i < static_cast<int>(destinations_.size()); ++i) {
            if (destinations_[i].label == previousLabel) {
                selectedDestinationIndex_ = i;
                break;
            }
        }
    }

    log("[A350AutoTaxi] Airport loaded: " + airportDisplayId(airport_) +
        " priority=" + std::to_string(airport_.sourcePriority) +
        " runways=" + std::to_string(airport_.runways.size() * 2) +
        " ramps=" + std::to_string(airport_.ramps.size()) +
        " destinations=" + std::to_string(destinations_.size()) +
        " source=" + airport_.sourceFile);
    return true;
}

void AutoTaxiSystem::rebuildDestinationChoices() {
    destinations_.clear();

    for (const auto& rw : airport_.runways) {
        DestinationChoice a;
        a.kind = DestinationKind::Runway;
        a.group = "RUNWAY";
        a.label = "Runway " + rw.end1;
        a.detail = "hold-short / active-zone node";
        a.runwayEnd = rw.end1;
        a.lat = rw.lat1;
        a.lon = rw.lon1;
        a.nodeId = nodeForRunwayEnd(rw.end1, rw.lat1, rw.lon1);
        destinations_.push_back(a);

        DestinationChoice b;
        b.kind = DestinationKind::Runway;
        b.group = "RUNWAY";
        b.label = "Runway " + rw.end2;
        b.detail = "hold-short / active-zone node";
        b.runwayEnd = rw.end2;
        b.lat = rw.lat2;
        b.lon = rw.lon2;
        b.nodeId = nodeForRunwayEnd(rw.end2, rw.lat2, rw.lon2);
        destinations_.push_back(b);
    }

    for (const auto& r : airport_.ramps) {
        DestinationChoice d;
        d.kind = DestinationKind::Ramp;
        const std::string typeUp = toUpper(r.type);
        const bool isGate = typeUp.find("GATE") != std::string::npos;
        d.group = isGate ? "GATE" : "RAMP";
        if (isGate) {
            d.label = "Gate " + r.name;
        } else if (!r.type.empty()) {
            d.label = r.type + " " + r.name;
        } else {
            d.label = "Ramp " + r.name;
        }
        d.heavyCompatible = isHeavyCompatibleWidth(r.widthCode);
        d.detail = r.aircraftTypes;
        if (!r.widthCode.empty()) d.detail += (d.detail.empty() ? "" : " | ") + std::string("width ") + r.widthCode;
        if (!r.operationType.empty()) d.detail += (d.detail.empty() ? "" : " | ") + r.operationType;
        if (!d.heavyCompatible) d.detail += (d.detail.empty() ? "" : " | ") + std::string("small for A350");
        d.rampName = r.name;
        d.lat = r.lat;
        d.lon = r.lon;
        d.nodeId = nearestNode(r.lat, r.lon);
        destinations_.push_back(d);
    }

    if (finiteLatLon(cfg_.destinationLat, cfg_.destinationLon)) {
        DestinationChoice d;
        d.kind = DestinationKind::Coordinate;
        d.group = "GPS";
        d.label = "Configured coordinate";
        d.detail = "destination_lat / destination_lon from A350AutoTaxi.ini";
        d.lat = cfg_.destinationLat;
        d.lon = cfg_.destinationLon;
        d.nodeId = nearestNode(d.lat, d.lon);
        destinations_.push_back(d);
    }

    std::sort(destinations_.begin(), destinations_.end(), [](const DestinationChoice& a, const DestinationChoice& b) {
        return destinationSortKey(a) < destinationSortKey(b);
    });
}

int AutoTaxiSystem::nodeForRunwayEnd(const std::string& runwayEnd, double fallbackLat, double fallbackLon) const {
    int best = -1;
    double bestD = std::numeric_limits<double>::infinity();
    const std::string wanted = toUpper(runwayEnd);

    auto considerNode = [&](int id) {
        const auto it = airport_.nodes.find(id);
        if (it == airport_.nodes.end()) return;
        const double d = geo::distanceMeters(fallbackLat, fallbackLon, it->second.lat, it->second.lon);
        if (d < bestD) {
            bestD = d;
            best = id;
        }
    };

    for (const auto& e : airport_.edges) {
        if (!e.activeZone || e.groundVehicleOnly) continue;
        bool mentionsRunway = false;
        for (const auto& r : e.activeRunways) {
            const auto ru = toUpper(r);
            if (ru == wanted || ru.find(wanted) != std::string::npos || wanted.find(ru) != std::string::npos) {
                mentionsRunway = true;
                break;
            }
        }
        if (!mentionsRunway) continue;
        considerNode(e.from);
        considerNode(e.to);
    }

    if (best >= 0) return best;
    return nearestNode(fallbackLat, fallbackLon);
}

int AutoTaxiSystem::nodeForDestination(const DestinationChoice& choice) const {
    if (choice.nodeId >= 0 && airport_.nodes.find(choice.nodeId) != airport_.nodes.end()) return choice.nodeId;
    if (choice.kind == DestinationKind::Runway) {
        return nodeForRunwayEnd(choice.runwayEnd, choice.lat, choice.lon);
    }
    if (finiteLatLon(choice.lat, choice.lon)) return nearestNode(choice.lat, choice.lon);
    return -1;
}

int AutoTaxiSystem::nearestNode(double lat, double lon, double* outDistanceM) const {
    int best = -1;
    double bestD = std::numeric_limits<double>::infinity();
    for (const auto& kv : airport_.nodes) {
        const double d = geo::distanceMeters(lat, lon, kv.second.lat, kv.second.lon);
        if (d < bestD) {
            bestD = d;
            best = kv.first;
        }
    }
    if (outDistanceM) *outDistanceM = bestD;
    return best;
}

AutoTaxiSystem::EdgeHit AutoTaxiSystem::nearestEdge(double lat, double lon) const {
    EdgeHit best;
    if (!finiteLatLon(lat, lon)) return best;

    for (const auto& e : airport_.edges) {
        if (e.groundVehicleOnly) continue;
        const auto ia = airport_.nodes.find(e.from);
        const auto ib = airport_.nodes.find(e.to);
        if (ia == airport_.nodes.end() || ib == airport_.nodes.end()) continue;

        double ax = 0.0, ay = 0.0, bx = 0.0, by = 0.0;
        geo::latLonToLocalMeters(lat, lon, ia->second.lat, ia->second.lon, ax, ay);
        geo::latLonToLocalMeters(lat, lon, ib->second.lat, ib->second.lon, bx, by);
        const double vx = bx - ax;
        const double vy = by - ay;
        const double len2 = vx * vx + vy * vy;
        if (len2 < 1.0) continue;
        const double t = clamp(-dot2(ax, ay, vx, vy) / len2, 0.0, 1.0);
        const double cx = ax + t * vx;
        const double cy = ay + t * vy;
        const double d = std::hypot(cx, cy);
        if (d < best.distanceM) {
            best.valid = true;
            best.from = e.from;
            best.to = e.to;
            best.distanceM = d;
            best.along01 = t;
            best.ident = e.ident;
            best.runway = e.runway;
            best.activeZone = e.activeZone;
        }
    }
    return best;
}

int AutoTaxiSystem::destinationNode() const {
    if (selectedDestinationIndex_ >= 0 && selectedDestinationIndex_ < static_cast<int>(destinations_.size())) {
        const int n = nodeForDestination(destinations_[selectedDestinationIndex_]);
        if (n >= 0) return n;
        log("[A350AutoTaxi] selected destination cannot be mapped to taxi network: " +
            destinations_[selectedDestinationIndex_].label);
    }

    if (cfg_.destinationNode >= 0) {
        if (airport_.nodes.find(cfg_.destinationNode) != airport_.nodes.end()) return cfg_.destinationNode;
        log("[A350AutoTaxi] destination_node not found in airport network: " + std::to_string(cfg_.destinationNode));
    }

    if (!cfg_.destinationRamp.empty()) {
        const RampStart* bestRamp = nullptr;
        for (const auto& r : airport_.ramps) {
            if (iequals(r.name, cfg_.destinationRamp)) {
                bestRamp = &r;
                break;
            }
        }
        if (!bestRamp) {
            for (const auto& r : airport_.ramps) {
                if (icontains(r.name, cfg_.destinationRamp)) {
                    bestRamp = &r;
                    break;
                }
            }
        }
        if (bestRamp) {
            if (!isHeavyCompatibleWidth(bestRamp->widthCode)) {
                log("[A350AutoTaxi] WARNING: destination ramp " + bestRamp->name +
                    " has ICAO width code " + bestRamp->widthCode + ", smaller than A350 code E.");
            }
            const int n = nearestNode(bestRamp->lat, bestRamp->lon);
            if (n >= 0) return n;
        }
        log("[A350AutoTaxi] destination_ramp not found: " + cfg_.destinationRamp);
    }

    if (finiteLatLon(cfg_.destinationLat, cfg_.destinationLon)) {
        const int n = nearestNode(cfg_.destinationLat, cfg_.destinationLon);
        if (n >= 0) return n;
    }

    return -1;
}

int AutoTaxiSystem::fallbackDestinationNode(int startNode) const {
    int best = -1;
    double bestD = -1.0;

    auto consider = [&](const TaxiNode& n) {
        if (n.id == startNode) return;
        const auto& start = airport_.nodes.at(startNode);
        const double d = geo::distanceMeters(start.lat, start.lon, n.lat, n.lon);
        if (d > bestD) {
            bestD = d;
            best = n.id;
        }
    };

    for (const auto& kv : airport_.nodes) {
        const auto usage = toUpper(kv.second.usage);
        if (usage == "DEST" || usage == "END" || usage == "BOTH") consider(kv.second);
    }
    if (best >= 0) return best;
    for (const auto& kv : airport_.nodes) consider(kv.second);
    return best;
}

bool AutoTaxiSystem::buildRoute(double currentLat, double currentLon, bool userVisibleLog) {
    if (!airport_.valid()) return false;

    double startDistance = 0.0;
    const int start = nearestNode(currentLat, currentLon, &startDistance);
    int dest = destinationNode();

    if (start < 0) return false;
    if (dest < 0 || dest == start) {
        dest = fallbackDestinationNode(start);
        if (userVisibleLog) log("[A350AutoTaxi] Destination unavailable; fallback node=" + std::to_string(dest));
    }
    if (dest < 0 || dest == start) return false;

    std::vector<int> route;
    if (!runAStar(start, dest, route)) return false;

    route_ = std::move(route);
    routeSegmentIndex_ = 0;
    waypointIndex_ = route_.size() > 1 ? 1 : 0;
    lastRouteDestNode_ = dest;
    pruneTerminalRouteForArrival(userVisibleLog);
    resetPidController();
    computeRouteMetrics();

    if (userVisibleLog) {
        log("[A350AutoTaxi] Route planned: start=" + std::to_string(start) +
            " dest=" + std::to_string(dest) +
            " nodes=" + std::to_string(route_.size()) +
            " distance_m=" + fmt0(routeDistanceM_) +
            " start_node_dist_m=" + fmt0(startDistance) +
            " route=" + routeSummary_);
    }
    return true;
}

std::vector<AutoTaxiSystem::AdjEdge> AutoTaxiSystem::adjacencyForNode(int nodeId) const {
    std::vector<AdjEdge> out;
    const auto fromIt = airport_.nodes.find(nodeId);
    if (fromIt == airport_.nodes.end()) return out;

    for (const auto& e : airport_.edges) {
        if (e.groundVehicleOnly) continue;
        int to = -1;
        if (e.from == nodeId) to = e.to;
        else if (e.bidirectional && e.to == nodeId) to = e.from;
        else continue;
        const auto toIt = airport_.nodes.find(to);
        if (toIt == airport_.nodes.end()) continue;

        double cost = geo::distanceMeters(fromIt->second.lat, fromIt->second.lon, toIt->second.lat, toIt->second.lon);
        if (cfg_.avoidRunwayEdges && e.runway) cost += cfg_.runwayPenaltyM;
        if (e.activeZone) cost += cfg_.activeZonePenaltyM;
        out.push_back({nodeId, to, cost, e.ident, e.runway, e.activeZone});
    }
    return out;
}

bool AutoTaxiSystem::runAStar(int startNode, int destNode, std::vector<int>& outRoute) const {
    const auto destIt = airport_.nodes.find(destNode);
    if (destIt == airport_.nodes.end()) return false;

    struct Item {
        int node;
        double f;
        double g;
        bool operator>(const Item& other) const { return f > other.f; }
    };

    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open;
    std::unordered_map<int, double> gScore;
    std::unordered_map<int, int> prev;
    std::unordered_map<int, std::string> prevIdent;

    auto heuristic = [&](int node) {
        const auto it = airport_.nodes.find(node);
        if (it == airport_.nodes.end()) return 0.0;
        return geo::distanceMeters(it->second.lat, it->second.lon, destIt->second.lat, destIt->second.lon);
    };

    gScore[startNode] = 0.0;
    open.push({startNode, heuristic(startNode), 0.0});

    while (!open.empty()) {
        const Item cur = open.top();
        open.pop();
        const auto known = gScore.find(cur.node);
        if (known == gScore.end() || cur.g > known->second + 1e-6) continue;
        if (cur.node == destNode) break;

        for (const auto& e : adjacencyForNode(cur.node)) {
            double step = e.costM;
            const auto prevIdIt = prevIdent.find(cur.node);
            if (prevIdIt != prevIdent.end() && !prevIdIt->second.empty() && !e.ident.empty() &&
                !iequals(prevIdIt->second, e.ident)) {
                step += cfg_.turnPenaltyM;
            }
            const double tentative = cur.g + step;
            auto it = gScore.find(e.to);
            if (it == gScore.end() || tentative < it->second) {
                gScore[e.to] = tentative;
                prev[e.to] = cur.node;
                prevIdent[e.to] = e.ident;
                open.push({e.to, tentative + heuristic(e.to), tentative});
            }
        }
    }

    if (gScore.find(destNode) == gScore.end()) return false;

    std::vector<int> rev;
    int n = destNode;
    rev.push_back(n);
    while (n != startNode) {
        auto it = prev.find(n);
        if (it == prev.end()) return false;
        n = it->second;
        rev.push_back(n);
    }
    std::reverse(rev.begin(), rev.end());
    outRoute = std::move(rev);
    return true;
}

const TaxiEdge* AutoTaxiSystem::findEdgeBetween(int from, int to) const {
    const TaxiEdge* reverseCandidate = nullptr;
    for (const auto& e : airport_.edges) {
        if (e.groundVehicleOnly) continue;
        if (e.from == from && e.to == to) return &e;
        if (e.bidirectional && e.from == to && e.to == from) reverseCandidate = &e;
    }
    return reverseCandidate;
}

double AutoTaxiSystem::routeRemainingDistanceM(double lat, double lon) const {
    if (route_.empty() || waypointIndex_ >= route_.size()) return 0.0;
    const auto& curTarget = airport_.nodes.at(route_[waypointIndex_]);
    double dist = geo::distanceMeters(lat, lon, curTarget.lat, curTarget.lon);
    for (size_t i = waypointIndex_; i + 1 < route_.size(); ++i) {
        const auto& a = airport_.nodes.at(route_[i]);
        const auto& b = airport_.nodes.at(route_[i + 1]);
        dist += geo::distanceMeters(a.lat, a.lon, b.lat, b.lon);
    }
    return dist;
}

void AutoTaxiSystem::pruneTerminalRouteForArrival(bool userVisibleLog) {
    if (!cfg_.terminalArrivalGuard || route_.size() < 3) return;

    int removed = 0;
    const int maxRemoved = 4;
    while (route_.size() >= 3 && removed < maxRemoved) {
        const size_t n = route_.size();
        const auto aIt = airport_.nodes.find(route_[n - 3]);
        const auto bIt = airport_.nodes.find(route_[n - 2]);
        const auto cIt = airport_.nodes.find(route_[n - 1]);
        if (aIt == airport_.nodes.end() || bIt == airport_.nodes.end() || cIt == airport_.nodes.end()) break;

        const double lenAB = geo::distanceMeters(aIt->second.lat, aIt->second.lon, bIt->second.lat, bIt->second.lon);
        const double lenBC = geo::distanceMeters(bIt->second.lat, bIt->second.lon, cIt->second.lat, cIt->second.lon);
        const double inHdg = geo::bearingDeg(aIt->second.lat, aIt->second.lon, bIt->second.lat, bIt->second.lon);
        const double outHdg = geo::bearingDeg(bIt->second.lat, bIt->second.lon, cIt->second.lat, cIt->second.lon);
        const double turn = std::abs(geo::diffSignedDeg(outHdg, inHdg));

        const bool tinyFinalHook = lenBC <= std::max(3.0, cfg_.terminalMinFinalLegM);
        const bool sharpShortFinal = turn >= std::max(30.0, cfg_.terminalSharpTurnDeg) &&
            lenBC <= std::max(cfg_.terminalMinFinalLegM, cfg_.terminalSharpTurnShortLegM);
        const bool closeNodeCluster = lenAB <= std::max(3.0, cfg_.terminalMinFinalLegM) &&
            lenBC <= std::max(3.0, cfg_.terminalSharpTurnShortLegM) && turn >= 60.0;

        if (!tinyFinalHook && !sharpShortFinal && !closeNodeCluster) break;

        const int removedNode = route_.back();
        route_.pop_back();
        ++removed;
        if (userVisibleLog) {
            log("[A350AutoTaxi] Terminal guard pruned tiny/sharp arrival hook node=" +
                std::to_string(removedNode) + " final_leg=" + fmt0(lenBC) +
                "m turn=" + fmt0(turn) + "deg. Will stop at previous route node to avoid circling.");
        }
    }

    if (removed > 0) {
        routeSegmentIndex_ = 0;
        waypointIndex_ = route_.size() > 1 ? 1 : 0;
    }
}

size_t AutoTaxiSystem::lookAheadWaypointIndex(double lat, double lon) const {
    if (route_.empty() || waypointIndex_ >= route_.size()) return waypointIndex_;
    double acc = 0.0;
    size_t idx = waypointIndex_;
    double lastLat = lat;
    double lastLon = lon;
    while (idx < route_.size()) {
        const auto& n = airport_.nodes.at(route_[idx]);
        acc += geo::distanceMeters(lastLat, lastLon, n.lat, n.lon);
        if (acc >= cfg_.lookaheadDistanceM) return idx;
        lastLat = n.lat;
        lastLon = n.lon;
        if (idx + 1 >= route_.size()) return idx;
        ++idx;
    }
    return route_.size() - 1;
}

double AutoTaxiSystem::upcomingTurnDeg(size_t atIndex) const {
    if (route_.size() < 3 || atIndex == 0 || atIndex + 1 >= route_.size()) return 0.0;
    const auto& a = airport_.nodes.at(route_[atIndex - 1]);
    const auto& b = airport_.nodes.at(route_[atIndex]);
    const auto& c = airport_.nodes.at(route_[atIndex + 1]);
    const double in = geo::bearingDeg(a.lat, a.lon, b.lat, b.lon);
    const double out = geo::bearingDeg(b.lat, b.lon, c.lat, c.lon);
    return std::abs(geo::diffSignedDeg(out, in));
}


AutoTaxiSystem::RouteTrack AutoTaxiSystem::computeRouteTrack(double lat, double lon,
                                                             size_t segmentIndex,
                                                             double lookaheadM) const {
    RouteTrack tr;
    if (route_.size() < 2 || segmentIndex + 1 >= route_.size()) return tr;

    const auto aIt = airport_.nodes.find(route_[segmentIndex]);
    const auto bIt = airport_.nodes.find(route_[segmentIndex + 1]);
    if (aIt == airport_.nodes.end() || bIt == airport_.nodes.end()) return tr;

    double ax = 0.0, ay = 0.0, bx = 0.0, by = 0.0;
    geo::latLonToLocalMeters(lat, lon, aIt->second.lat, aIt->second.lon, ax, ay);
    geo::latLonToLocalMeters(lat, lon, bIt->second.lat, bIt->second.lon, bx, by);

    const double vx = bx - ax;
    const double vy = by - ay;
    const double len2 = vx * vx + vy * vy;
    if (len2 < 1.0) return tr;

    const double len = std::sqrt(len2);
    const double ux = vx / len;
    const double uy = vy / len;
    const double t = clamp(-dot2(ax, ay, vx, vy) / len2, 0.0, 1.0);
    const double cx = ax + t * vx;
    const double cy = ay + t * vy;

    // Positive means aircraft is left of the path direction. We use it for display/debug;
    // the actual control uses the look-ahead point, so no sign convention can flip steering.
    const double startToAircraftX = -ax;
    const double startToAircraftY = -ay;
    const double signedCrossTrack = ux * startToAircraftY - uy * startToAircraftX;

    double targetX = cx;
    double targetY = cy;
    double remain = std::max(5.0, lookaheadM);
    size_t idx = segmentIndex;
    double along = t;

    while (idx + 1 < route_.size()) {
        const auto fromIt = airport_.nodes.find(route_[idx]);
        const auto toIt = airport_.nodes.find(route_[idx + 1]);
        if (fromIt == airport_.nodes.end() || toIt == airport_.nodes.end()) break;

        double x1 = 0.0, y1 = 0.0, x2 = 0.0, y2 = 0.0;
        geo::latLonToLocalMeters(lat, lon, fromIt->second.lat, fromIt->second.lon, x1, y1);
        geo::latLonToLocalMeters(lat, lon, toIt->second.lat, toIt->second.lon, x2, y2);
        const double sx = x2 - x1;
        const double sy = y2 - y1;
        const double segLen = std::hypot(sx, sy);
        if (segLen < 1.0) {
            ++idx;
            along = 0.0;
            continue;
        }

        const double available = (1.0 - along) * segLen;
        if (remain <= available || idx + 2 >= route_.size()) {
            const double targetT = clamp(along + remain / segLen, 0.0, 1.0);
            targetX = x1 + targetT * sx;
            targetY = y1 + targetT * sy;
            break;
        }

        remain -= available;
        ++idx;
        along = 0.0;
        targetX = x2;
        targetY = y2;
    }

    tr.valid = true;
    tr.segmentIndex = segmentIndex;
    tr.segmentLengthM = len;
    tr.along01 = t;
    tr.distanceM = std::hypot(cx, cy);
    tr.signedCrossTrackM = signedCrossTrack;
    tr.pathHeadingDeg = geo::bearingDeg(aIt->second.lat, aIt->second.lon, bIt->second.lat, bIt->second.lon);
    tr.lookaheadXEastM = targetX;
    tr.lookaheadYNorthM = targetY;
    tr.desiredTrackDeg = geo::normalize360(geo::rad2deg(std::atan2(targetX, targetY)));
    tr.distanceToSegmentEndM = std::hypot(bx, by);
    return tr;
}

AutoTaxiSystem::RouteTrack AutoTaxiSystem::currentRouteTrack(double lat, double lon, double lookaheadM) {
    RouteTrack cur = computeRouteTrack(lat, lon, routeSegmentIndex_, lookaheadM);
    if (!cur.valid) return cur;

    const double advanceAlong = clamp(cfg_.routeSegmentAdvanceAlong, 0.55, 0.99);

    // Monotonic leg selection: do not chase the closest node. We stay on the current
    // planned leg until the aircraft is genuinely at the end of that leg, then advance.
    // This removes the +/-1 steering flip that happens when the aircraft spawns near a node.
    while (routeSegmentIndex_ + 1 < route_.size() - 1) {
        const bool closeToEnd = cur.distanceToSegmentEndM < std::max(6.0, cfg_.waypointRadiusM);
        const bool pastEnd = cur.along01 >= advanceAlong;
        if (!closeToEnd && !pastEnd) break;
        ++routeSegmentIndex_;
        cur = computeRouteTrack(lat, lon, routeSegmentIndex_, lookaheadM);
        if (!cur.valid) break;
    }

    // Only allow snapping forward when a later planned segment is clearly better. We never
    // snap backwards; the goal is to converge back onto the original route, not oscillate
    // around the nearest taxi node.
    const int scanAhead = clamp(static_cast<int>(std::round(cfg_.routeSegmentScanAhead)), 0, 12);
    RouteTrack best = cur;
    for (int k = 1; k <= scanAhead; ++k) {
        const size_t idx = routeSegmentIndex_ + static_cast<size_t>(k);
        if (idx + 1 >= route_.size()) break;
        RouteTrack cand = computeRouteTrack(lat, lon, idx, lookaheadM);
        if (!cand.valid) continue;
        const bool muchCloser = cand.distanceM + 8.0 < best.distanceM;
        const bool currentNearlyDone = cur.along01 > 0.75 || cur.distanceToSegmentEndM < cfg_.waypointRadiusM * 1.5;
        if (muchCloser && currentNearlyDone) best = cand;
    }
    if (best.valid && best.segmentIndex > routeSegmentIndex_) {
        routeSegmentIndex_ = best.segmentIndex;
        cur = best;
    }

    waypointIndex_ = std::min(routeSegmentIndex_ + 1, route_.size() - 1);
    return cur;
}

void AutoTaxiSystem::resetPidController() {
    pidIntegralDegSec_ = 0.0;
    pidPrevHeadingErrDeg_ = 0.0;
    pidDerivativeFilteredDegPerSec_ = 0.0;
    pidPrevValid_ = false;
    lastCrossTrackErrorM_ = 0.0;
    lastPathHeadingErrorDeg_ = 0.0;
    lastPidOutputDeg_ = 0.0;
    prevSignedCrossTrackM_ = 0.0;
    filteredXteRateMps_ = 0.0;
    prevXteValid_ = false;
    prevFinalDistM_ = std::numeric_limits<double>::infinity();
    finalDistIncreasingSec_ = 0.0;
}

void AutoTaxiSystem::computeRouteMetrics() {
    routeDistanceM_ = 0.0;
    routeSummary_.clear();
    routeSafety_.clear();

    if (route_.size() < 2) return;

    std::vector<std::string> names;
    bool containsRunway = false;
    bool containsActiveZone = false;
    std::string lastName;

    for (size_t i = 0; i + 1 < route_.size(); ++i) {
        const auto& a = airport_.nodes.at(route_[i]);
        const auto& b = airport_.nodes.at(route_[i + 1]);
        routeDistanceM_ += geo::distanceMeters(a.lat, a.lon, b.lat, b.lon);
        const TaxiEdge* e = findEdgeBetween(route_[i], route_[i + 1]);
        if (!e) continue;
        containsRunway = containsRunway || e->runway;
        containsActiveZone = containsActiveZone || e->activeZone;
        std::string name = e->ident.empty() ? (e->runway ? "RWY" : "TWY") : e->ident;
        if (!name.empty() && !iequals(name, lastName)) {
            names.push_back(name);
            lastName = name;
        }
    }

    const int maxNames = std::max(3, cfg_.routeSummaryMaxNames);
    for (int i = 0; i < static_cast<int>(names.size()) && i < maxNames; ++i) {
        if (!routeSummary_.empty()) routeSummary_ += " -> ";
        routeSummary_ += names[i];
    }
    if (static_cast<int>(names.size()) > maxNames) routeSummary_ += " -> ...";
    if (routeSummary_.empty()) routeSummary_ = "taxi network nodes";

    if (containsRunway) routeSafety_ = "WARNING: route contains runway segment";
    else if (containsActiveZone) routeSafety_ = "Caution: route reaches runway active-zone/hold-short area";
    else routeSafety_ = "Taxiways only; runway edges avoided";
}

bool AutoTaxiSystem::startSafetyCheck(double lat, double lon) const {
    if (drOnGroundAny_) {
        const int onGround = XPLMGetDatai(drOnGroundAny_);
        if (onGround == 0) {
            log("[A350AutoTaxi] Refused: aircraft is not on ground.");
            return false;
        }
    }

    const double gsKts = getDataf(drGroundSpeed_) / geo::kKnotToMps;
    if (gsKts > cfg_.maxStartSpeedKts) {
        log("[A350AutoTaxi] Refused: aircraft already moving too fast (" + fmt1(gsKts) + " kt).");
        return false;
    }

    double nodeDist = 0.0;
    nearestNode(lat, lon, &nodeDist);
    const EdgeHit edge = nearestEdge(lat, lon);
    const double bestDist = edge.valid ? std::min(nodeDist, edge.distanceM) : nodeDist;
    if (bestDist > cfg_.maxStartNodeDistanceM) {
        log("[A350AutoTaxi] Refused: aircraft is " + fmt0(bestDist) +
            " m from taxi network; reposition closer to a gate/taxiway first.");
        return false;
    }
    return true;
}

void AutoTaxiSystem::updateControl(double dt) {
    if (route_.empty() || route_.size() < 2 || routeSegmentIndex_ + 1 >= route_.size()) {
        arriveAndHold();
        return;
    }

    if (drOnGroundAny_) {
        const int onGround = XPLMGetDatai(drOnGroundAny_);
        if (onGround == 0) {
            writeControls(0.0, 0.0, cfg_.maxBrake, cfg_.maxBrake);
            log("[A350AutoTaxi] Emergency hold: aircraft is not on ground.");
            return;
        }
    }

    const double lat = getDataf(drLat_);
    const double lon = getDataf(drLon_);
    const double psi = getDataf(drPsi_);
    const double gsMps = std::max(0.0, getDataf(drGroundSpeed_));
    const double gsKts = gsMps / geo::kKnotToMps;

    if (offRouteReplanCooldown_ > 0) --offRouteReplanCooldown_;
    if (cfg_.autoReplanIfOffRoute && offRouteReplanCooldown_ == 0) {
        const EdgeHit edge = nearestEdge(lat, lon);
        if (edge.valid && edge.distanceM > cfg_.offRouteReplanDistanceM) {
            offRouteReplanCooldown_ = 40;
            if (buildRoute(lat, lon, false)) {
                log("[A350AutoTaxi] Auto re-planned route after off-route deviation " + fmt0(edge.distanceM) + " m.");
            } else {
                writeControls(0.0, 0.0, cfg_.maxBrake, cfg_.maxBrake);
                active_ = false;
                log("[A350AutoTaxi] Safety stop: off-route and no replacement route found.");
                return;
            }
        }
    }

    // Dynamic lookahead: longer at higher ground speed, still clamped for terminal/gate areas.
    // The first pass finds the current planned leg; a second optional pass extends the
    // preview when an upcoming corner is close. This is deliberate "AP style" corner
    // anticipation: the controller starts aiming into the next leg before the nose reaches
    // the taxi-node, instead of waiting until the aircraft has overshot and then weaving
    // back across the route.
    const double baseLookaheadM = clamp(cfg_.lookaheadDistanceM + gsKts * cfg_.lookaheadSpeedGainMPerKt,
                                        std::max(10.0, cfg_.lookaheadMinDistanceM),
                                        std::max(cfg_.lookaheadMinDistanceM + 1.0, cfg_.lookaheadMaxDistanceM));
    double lookaheadM = baseLookaheadM;
    RouteTrack track = currentRouteTrack(lat, lon, lookaheadM);
    if (!track.valid) {
        writeControls(0.0, 0.0, cfg_.maxBrake, cfg_.maxBrake);
        active_ = false;
        log("[A350AutoTaxi] Safety stop: route tracking segment invalid.");
        return;
    }

    // Terminal guard: do not chase exact micro-nodes near a runway hold-short/arrival
    // point. If the aircraft is inside the final operational bubble, or has just
    // passed the destination while still close, stop and hold instead of continuing
    // to command tight 360-degree turns around the last node.
    const bool finalSegment = routeSegmentIndex_ + 2 >= route_.size();
    const auto& finalNode = airport_.nodes.at(route_.back());
    const double finalDistM = geo::distanceMeters(lat, lon, finalNode.lat, finalNode.lon);
    const double terminalRemainingM = routeRemainingDistanceM(lat, lon);

    // Runway destination line-up geometry. A runway destination should not simply
    // stop at the nearest hold-short/active-zone node; once close enough, switch to
    // runway-centreline capture and align the aircraft heading with the selected
    // runway direction. Positive signed XTE means the aircraft is left of runway
    // centreline in the selected runway heading direction.
    bool runwayAlignGeoValid = false;
    bool runwayAlignActive = false;
    double runwayAlignBlend = 0.0;
    double runwayHeadingDeg = 0.0;
    double runwayHeadingErrDeg = 0.0;
    double runwaySignedXteM = 0.0;
    double runwayDistToThresholdM = std::numeric_limits<double>::infinity();
    if (cfg_.runwayAlignmentMode && selectedDestinationIndex_ >= 0 &&
        selectedDestinationIndex_ < static_cast<int>(destinations_.size())) {
        const DestinationChoice& dest = destinations_[selectedDestinationIndex_];
        if (dest.kind == DestinationKind::Runway) {
            const std::string wantedEnd = toUpper(dest.runwayEnd);
            for (const auto& rw : airport_.runways) {
                double thrLat = 0.0, thrLon = 0.0, farLat = 0.0, farLon = 0.0;
                if (toUpper(rw.end1) == wantedEnd) {
                    thrLat = rw.lat1; thrLon = rw.lon1; farLat = rw.lat2; farLon = rw.lon2;
                } else if (toUpper(rw.end2) == wantedEnd) {
                    thrLat = rw.lat2; thrLon = rw.lon2; farLat = rw.lat1; farLon = rw.lon1;
                } else {
                    continue;
                }

                double tx = 0.0, ty = 0.0, fx = 0.0, fy = 0.0;
                geo::latLonToLocalMeters(lat, lon, thrLat, thrLon, tx, ty);
                geo::latLonToLocalMeters(lat, lon, farLat, farLon, fx, fy);
                const double vx = fx - tx;
                const double vy = fy - ty;
                const double len = std::hypot(vx, vy);
                if (len < 100.0) break;
                const double ux = vx / len;
                const double uy = vy / len;
                const double aircraftFromThresholdX = -tx;
                const double aircraftFromThresholdY = -ty;
                runwaySignedXteM = ux * aircraftFromThresholdY - uy * aircraftFromThresholdX;
                runwayHeadingDeg = geo::bearingDeg(thrLat, thrLon, farLat, farLon);
                runwayHeadingErrDeg = geo::diffSignedDeg(runwayHeadingDeg, psi);
                runwayDistToThresholdM = geo::distanceMeters(lat, lon, thrLat, thrLon);
                runwayAlignGeoValid = true;
                break;
            }
        }
    }

    if (runwayAlignGeoValid) {
        const double captureDist = std::max(50.0, cfg_.runwayAlignCaptureDistanceM);
        const double fullDist = clamp(cfg_.runwayAlignFullDistanceM, 20.0, captureDist - 1.0);
        const double distBasis = std::min(terminalRemainingM, runwayDistToThresholdM);
        if (distBasis <= captureDist) {
            runwayAlignActive = true;
            const double rawBlend = clamp((captureDist - distBasis) / std::max(1.0, captureDist - fullDist), 0.0, 1.0);
            runwayAlignBlend = clamp(0.35 + 0.65 * rawBlend, 0.0, 1.0);
        }
    }

    const double terminalStopRadiusM = std::max(cfg_.finalStopRadiusM, cfg_.terminalArrivalRadiusM);
    const double terminalNoTightDistanceM = std::max(terminalStopRadiusM, cfg_.terminalNoTightTurnDistanceM);
    const bool terminalGuardActive = cfg_.terminalArrivalGuard &&
        (finalSegment || terminalRemainingM <= terminalNoTightDistanceM);

    if (cfg_.terminalArrivalGuard && std::isfinite(prevFinalDistM_) &&
        finalDistM <= std::max(terminalStopRadiusM, cfg_.terminalOvershootRadiusM) &&
        finalDistM > prevFinalDistM_ + 0.20) {
        finalDistIncreasingSec_ += dt;
    } else {
        finalDistIncreasingSec_ = std::max(0.0, finalDistIncreasingSec_ - dt * 2.0);
    }
    prevFinalDistM_ = finalDistM;

    const bool terminalInsideBubble = cfg_.terminalArrivalGuard && finalDistM <= terminalStopRadiusM;
    const bool terminalRouteComplete = cfg_.terminalArrivalGuard &&
        terminalRemainingM <= terminalStopRadiusM &&
        std::abs(track.signedCrossTrackM) <= std::max(2.0, cfg_.terminalXteStopM);
    const bool terminalOvershotClose = cfg_.terminalArrivalGuard &&
        finalDistIncreasingSec_ >= std::max(0.1, cfg_.terminalOvershootSeconds) &&
        finalDistM <= std::max(terminalStopRadiusM, cfg_.terminalOvershootRadiusM) &&
        gsKts <= 6.0;

    const bool runwayAlignmentComplete = !runwayAlignActive ||
        (std::abs(runwaySignedXteM) <= std::max(0.5, cfg_.runwayAlignCenterToleranceM) &&
         std::abs(runwayHeadingErrDeg) <= std::max(0.5, cfg_.runwayAlignHeadingToleranceDeg));

    if ((terminalInsideBubble || terminalRouteComplete || terminalOvershotClose) && runwayAlignmentComplete) {
        arriveAndHold();
        return;
    }

    const size_t upcomingTurnIndex = std::min(routeSegmentIndex_ + 1, route_.size() - 1);
    const double upcomingTurnAngleDeg = upcomingTurnDeg(upcomingTurnIndex);

    bool outboundHeadingValid = false;
    double outboundHeadingDeg = 0.0;
    if (routeSegmentIndex_ + 2 < route_.size()) {
        const auto cornerIt = airport_.nodes.find(route_[routeSegmentIndex_ + 1]);
        const auto outIt = airport_.nodes.find(route_[routeSegmentIndex_ + 2]);
        if (cornerIt != airport_.nodes.end() && outIt != airport_.nodes.end()) {
            outboundHeadingDeg = geo::bearingDeg(cornerIt->second.lat, cornerIt->second.lon,
                                                 outIt->second.lat, outIt->second.lon);
            outboundHeadingValid = true;
        }
    }

    double upcomingTurnSignedDeg = 0.0;
    double upcomingTurnSign = 0.0;
    if (outboundHeadingValid) {
        upcomingTurnSignedDeg = geo::diffSignedDeg(outboundHeadingDeg, track.pathHeadingDeg);
        if (std::abs(upcomingTurnSignedDeg) > 1.0) {
            upcomingTurnSign = std::copysign(1.0, upcomingTurnSignedDeg);
        }
    }
    const double outboundHeadingErrForTurn = outboundHeadingValid ? geo::diffSignedDeg(outboundHeadingDeg, psi) : 0.0;

    // Force-full-turn arming. This bypasses the slow blend/rate-limit path for
    // large upcoming corners; the steer command will be snapped directly to full
    // tiller later, until the nose is close enough to outbound heading to roll out.
    const bool forceFullTurnWindow = !terminalGuardActive && cfg_.tightTurnForceFullSteer &&
        outboundHeadingValid && upcomingTurnAngleDeg >= std::max(1.0, cfg_.tightTurnForceFullSteerAngleDeg) &&
        track.distanceToSegmentEndM <= std::max(20.0, cfg_.tightTurnForceFullSteerDistanceM);
    const bool forceFullTurnRollout = forceFullTurnWindow &&
        std::abs(outboundHeadingErrForTurn) <= std::max(3.0, cfg_.tightTurnForceFullReleaseHeadingDeg);
    const bool forceFullTurnActive = forceFullTurnWindow && !forceFullTurnRollout;

    // Early hard-turn takeover. The direct track-course controller intentionally holds
    // the current taxiway centerline on straight legs, but before a 70-100 degree corner
    // it can delay the transition into the one-pass turn program. This blend starts
    // *before* the normal tight-turn window, fades out track-course authority, and allows
    // the steering snap logic to arm while there is still distance left to make the turn.
    double earlyTurnBlend = 0.0;
    if (!terminalGuardActive && cfg_.earlyTurnTakeover && outboundHeadingValid &&
        upcomingTurnAngleDeg >= std::max(1.0, cfg_.earlyTurnTakeoverAngleDeg) &&
        track.distanceToSegmentEndM <= std::max(20.0, cfg_.earlyTurnTakeoverDistanceM)) {
        const double startDist = std::max(20.0, cfg_.earlyTurnTakeoverDistanceM);
        const double fullDist = clamp(cfg_.earlyTurnFullDistanceM, 5.0, startDist - 1.0);
        const double distT = clamp((startDist - track.distanceToSegmentEndM) /
                                   std::max(1.0, startDist - fullDist), 0.0, 1.0);
        const double turnT = clamp((upcomingTurnAngleDeg - cfg_.earlyTurnTakeoverAngleDeg) /
                                   std::max(1.0, 95.0 - cfg_.earlyTurnTakeoverAngleDeg), 0.0, 1.0);
        const double raw = distT * turnT;
        if (raw > 0.0) {
            earlyTurnBlend = clamp(std::max(clamp(cfg_.earlyTurnMinBlend, 0.0, 1.0), raw), 0.0, 1.0);
        }
    }
    if (forceFullTurnActive) {
        // Make route-track fade and tight-turn speed/brake scheduling engage immediately.
        earlyTurnBlend = 1.0;
    }

    double tightTurnBlend = 0.0;
    if (!terminalGuardActive && cfg_.tightTurnMode &&
        upcomingTurnAngleDeg >= std::max(1.0, cfg_.tightTurnAngleDeg) &&
        (track.distanceToSegmentEndM <= std::max(10.0, cfg_.tightTurnTriggerDistanceM) || earlyTurnBlend > 0.0)) {
        double normalTightBlend = 0.0;
        if (track.distanceToSegmentEndM <= std::max(10.0, cfg_.tightTurnTriggerDistanceM)) {
            const double distT = 1.0 - clamp(track.distanceToSegmentEndM /
                                             std::max(10.0, cfg_.tightTurnTriggerDistanceM), 0.0, 1.0);
            const double turnT = clamp((upcomingTurnAngleDeg - cfg_.tightTurnAngleDeg) /
                                       std::max(1.0, 95.0 - cfg_.tightTurnAngleDeg), 0.0, 1.0);
            normalTightBlend = clamp(0.25 + 0.75 * distT * turnT, 0.0, 1.0);
        }
        tightTurnBlend = std::max(normalTightBlend, earlyTurnBlend);

        // Tight-radius cornering: aim just beyond the corner apex instead of using a
        // long speed-based lookahead far down the next leg. This reduces the drawn arc
        // radius while still starting the turn before reaching the node.
        const double apexLead = std::max(5.0, cfg_.tightTurnLookaheadAfterApexM);
        const double tightLookahead = std::max(cfg_.tightTurnMinLookaheadM,
                                               track.distanceToSegmentEndM + apexLead);
        lookaheadM = baseLookaheadM + tightTurnBlend * (tightLookahead - baseLookaheadM);
        track = currentRouteTrack(lat, lon, lookaheadM);
        if (!track.valid) {
            writeControls(0.0, 0.0, cfg_.maxBrake, cfg_.maxBrake);
            active_ = false;
            log("[A350AutoTaxi] Safety stop: tight-turn route tracking segment invalid.");
            return;
        }
    } else if (!terminalGuardActive && cfg_.turnAnticipation &&
        upcomingTurnAngleDeg >= std::max(1.0, cfg_.turnAnticipationMinAngleDeg) &&
        track.distanceToSegmentEndM <= std::max(10.0, cfg_.turnAnticipationDistanceM)) {
        const double distT = 1.0 - clamp(track.distanceToSegmentEndM /
                                        std::max(10.0, cfg_.turnAnticipationDistanceM), 0.0, 1.0);
        const double turnT = clamp((upcomingTurnAngleDeg - cfg_.turnAnticipationMinAngleDeg) /
                                   std::max(1.0, 95.0 - cfg_.turnAnticipationMinAngleDeg), 0.0, 1.0);
        const double extraLookahead = std::max(0.0, cfg_.turnAnticipationExtraLookaheadM) * distT * turnT;
        const double maxAnticipatedLookahead = std::max(cfg_.lookaheadMaxDistanceM,
                                                        cfg_.lookaheadMaxDistanceM + cfg_.turnAnticipationExtraLookaheadM);
        lookaheadM = clamp(baseLookaheadM + extraLookahead, baseLookaheadM, maxAnticipatedLookahead);
        if (lookaheadM > baseLookaheadM + 0.5) {
            track = currentRouteTrack(lat, lon, lookaheadM);
            if (!track.valid) {
                writeControls(0.0, 0.0, cfg_.maxBrake, cfg_.maxBrake);
                active_ = false;
                log("[A350AutoTaxi] Safety stop: anticipated route tracking segment invalid.");
                return;
            }
        }
    }

    const bool hardOnePassTurn = !terminalGuardActive && cfg_.tightTurnOnePassMode &&
        (tightTurnBlend > 0.05 || earlyTurnBlend > 0.0) &&
        upcomingTurnAngleDeg >= std::max(1.0, cfg_.tightTurnOnePassAngleDeg);

    // Path-following heading error. This points to a look-ahead point on the *planned route*,
    // not to whichever node happens to be nearest.
    //
    // Fast route-capture mode: if the aircraft starts off the planned line, or has drifted
    // several metres away from it, increase cross-track bias and steering authority just
    // like an AP localizer-capture mode. As the cross-track error falls back below the
    // near-route band, fade back to the smooth PID gains. This fixes the slow recovery +
    // repeated S-turns that happened when the controller was always using the same gentle
    // cross-track gain.
    const double xteM = track.signedCrossTrackM;
    const double xteAbsM = std::abs(xteM);
    const double xtePrevAbsM = prevXteValid_ ? std::abs(prevSignedCrossTrackM_) : xteAbsM;
    const bool crossedRouteCenterline = prevXteValid_ && (xteM * prevSignedCrossTrackM_ < 0.0);
    const bool justStartedCapture = activeTimeSec_ <= std::max(0.0, cfg_.fastCaptureStartSeconds);

    // Predictive XTE lead: do not wait for the displayed XTE to cross zero before
    // changing steering direction. We estimate lateral motion from the signed XTE
    // rate, then command with a short look-ahead XTE. Example: if XTE is -5 m but
    // increasing rapidly toward zero, predictedXteM may already be positive, so
    // the controller starts counter-steering before the centerline is crossed.
    double xteRateMps = 0.0;
    if (prevXteValid_ && dt > 0.02) {
        const double rawRate = clamp((xteM - prevSignedCrossTrackM_) / dt, -8.0, 8.0);
        const double filterSec = std::max(0.03, cfg_.predictiveXteRateFilterSec);
        const double alpha = clamp(dt / filterSec, 0.0, 1.0);
        filteredXteRateMps_ += alpha * (rawRate - filteredXteRateMps_);
        xteRateMps = filteredXteRateMps_;
    } else {
        filteredXteRateMps_ = 0.0;
    }

    // Geometry-derived XTE rate. During tiny line corrections the measured XTE can
    // change only a few centimetres per frame, so the old predictive lead often did
    // not reverse steering until the displayed XTE had already crossed zero. The
    // aircraft heading relative to the planned taxi leg predicts the future crossing
    // earlier: if the nose is still aimed across the line, start unwinding/counter-
    // steering now, even when the measured XTE delta is small.
    const double pathHeadingErrForLead = geo::diffSignedDeg(track.pathHeadingDeg, psi);
    const double geomXteRateMps = gsMps * std::sin(geo::deg2rad(pathHeadingErrForLead));

    double xteForControlM = xteM;
    if (cfg_.predictiveXteLead && xteAbsM >= std::max(0.0, cfg_.predictiveXteMinActiveM)) {
        const double leadSec = std::max(0.0, cfg_.predictiveXteLeadTimeSec);
        const double maxLeadM = std::max(0.5, cfg_.predictiveXteMaxLeadM);
        const double predicted = clamp(xteM + xteRateMps * leadSec, -maxLeadM, maxLeadM);
        const bool willCrossSoon = prevXteValid_ && (predicted * xteM < 0.0);
        const bool movingTowardCenter = prevXteValid_ && std::abs(predicted) < xteAbsM;
        const bool movingAwayFromCenter = prevXteValid_ && std::abs(predicted) > xteAbsM;

        if (willCrossSoon || movingTowardCenter || movingAwayFromCenter) {
            xteForControlM = predicted;
        }
    }

    // XTE corridor / sway guard. Instead of trying to drive every capture all the
    // way to exactly 0 m immediately, first capture into a +/- corridor, then
    // transition to low-gain fine tracking. This prevents the controller from
    // applying a large yaw command at XTE 3-5 m and then S-turning across the line.
    double xteCorridorBlend = 0.0;       // 0 outside corridor, 1 in fine-tune zone
    double xteCorridorOuterBlend = 0.0;  // outside +/- corridor but still managed
    double xteCorridorSteerDampBlend = 0.0;
    double xteCorridorTrackMaxInterceptDeg = cfg_.trackCourseMaxInterceptDeg;
    const bool xteCorridorAvailable = cfg_.xteCorridorControl && !terminalGuardActive && !hardOnePassTurn;
    if (xteCorridorAvailable) {
        const double corridorM = std::max(0.5, cfg_.xteCorridorM);
        const double fineM = clamp(cfg_.xteFineTuneM, 0.05, corridorM);
        const double rateWeight = clamp(cfg_.microAnticipateGeomRateWeight, 0.0, 1.0);
        const double blendedRate = xteRateMps * (1.0 - rateWeight) + geomXteRateMps * rateWeight;
        const double leadSec = std::max(0.0, cfg_.xteCorridorLeadSec);
        const double leadXte = clamp(xteM + blendedRate * leadSec, -corridorM * 2.0, corridorM * 2.0);
        const bool headingTowardCenter = (xteM * geomXteRateMps) < 0.0;
        const double sign = (xteM == 0.0) ? 0.0 : std::copysign(1.0, xteM);

        if (xteAbsM <= corridorM) {
            const double bandT = clamp((corridorM - xteAbsM) / std::max(0.10, corridorM - fineM), 0.0, 1.0);
            const double fineT = clamp((fineM - xteAbsM) / std::max(0.10, fineM), 0.0, 1.0);
            xteCorridorBlend = std::max(bandT, fineT);
            xteCorridorSteerDampBlend = clamp(0.25 + 0.75 * bandT, 0.0, 1.0);

            // Fine tracking uses a softened effective XTE. If the nose is already
            // moving toward the centerline, the lead value may be smaller or even
            // opposite sign; using it starts the rollout before crossing 0 m.
            const double innerGain = clamp(cfg_.xteCorridorInnerGain, 0.05, 1.0);
            double softened = sign * (xteAbsM <= fineM
                ? xteAbsM * innerGain
                : fineM * innerGain + (xteAbsM - fineM) * (0.55 + 0.45 * innerGain));
            if (headingTowardCenter && (std::abs(leadXte) < std::abs(softened) || leadXte * xteM < 0.0)) {
                softened = leadXte;
            }
            xteForControlM = softened;
            xteCorridorTrackMaxInterceptDeg = std::min(xteCorridorTrackMaxInterceptDeg,
                std::max(3.0, cfg_.xteCorridorMaxInterceptDeg));
        } else {
            // Outside the corridor, do not aim for an immediate centerline crossing.
            // Aim for the corridor boundary plus a reduced excess term so the first
            // capture normally lands inside +/-5 m rather than overshooting through it.
            const double excess = xteAbsM - corridorM;
            const double outerGain = clamp(cfg_.xteCorridorOuterGain, 0.10, 1.0);
            const double managedXte = sign * (corridorM + excess * outerGain);
            xteForControlM = managedXte;
            xteCorridorOuterBlend = clamp((xteAbsM - corridorM) / std::max(1.0, corridorM), 0.0, 1.0);
            xteCorridorTrackMaxInterceptDeg = std::min(xteCorridorTrackMaxInterceptDeg,
                std::max(cfg_.xteCorridorMaxInterceptDeg, cfg_.xteCorridorOuterMaxInterceptDeg));
        }
    }

    double microLeadBlend = 0.0;
    double microReturnBlend = 0.0;
    double microReturnSign = 0.0;
    if (cfg_.microAnticipateReturn && gsKts >= std::max(0.0, cfg_.microAnticipateMinSpeedKts) &&
        xteAbsM <= std::max(0.5, cfg_.microAnticipateBandM) &&
        std::abs(pathHeadingErrForLead) >= std::max(0.0, cfg_.microAnticipateMinHeadingDeg)) {
        const double rateWeight = clamp(cfg_.microAnticipateGeomRateWeight, 0.0, 1.0);
        const double blendedRate = xteRateMps * (1.0 - rateWeight) + geomXteRateMps * rateWeight;
        const double microLeadSec = std::max(0.0, cfg_.microAnticipateLeadSec);
        const double microMaxLeadM = std::max(0.5, cfg_.microAnticipateMaxLeadM);
        const double microPredicted = clamp(xteM + blendedRate * microLeadSec, -microMaxLeadM, microMaxLeadM);

        const bool headingTowardCenter = (xteM * geomXteRateMps) < 0.0;
        const bool predictionCrossesLine = (xteM * microPredicted) < 0.0;
        const bool predictionCloserToLine = std::abs(microPredicted) + 0.05 < xteAbsM;
        const bool alreadyNearLine = xteAbsM <= std::max(0.25, cfg_.nearRouteDampingM);

        if (headingTowardCenter || predictionCrossesLine || (alreadyNearLine && predictionCloserToLine)) {
            // Use the more forward-looking XTE estimate. This makes small course
            // corrections behave like AP localizer rollout: start returning the
            // tiller before the centerline is crossed instead of after.
            xteForControlM = microPredicted;
            const double band = std::max(0.5, cfg_.microAnticipateBandM);
            microLeadBlend = clamp(1.0 - xteAbsM / band, 0.0, 1.0);
        }

        // Stronger pre-zero return logic. In practice, only biasing xteForControlM can
        // still be too weak because the look-ahead point may remain on the same side of
        // the line. If the aircraft is close to the planned route and is still steering
        // toward the centerline, begin commanding the opposite tiller direction before
        // XTE reaches zero. This is the missing AP-style rollout behavior for tiny
        // heading corrections.
        if (cfg_.microAnticipateDirectSteer && !terminalGuardActive && !hardOnePassTurn && tightTurnBlend < 0.12 &&
            xteAbsM > 0.05 && xteAbsM <= std::max(0.5, cfg_.microAnticipateDirectBandM)) {
            const double directBand = std::max(0.5, cfg_.microAnticipateDirectBandM);
            const double closureRate = (xteM * blendedRate < 0.0) ? std::abs(blendedRate) : 0.0;
            const double timeToCross = closureRate > 0.03 ? xteAbsM / closureRate : 999.0;
            const double timeBlend = clamp(1.0 - timeToCross / std::max(0.5, microLeadSec), 0.0, 1.0);
            const double bandBlend = std::pow(clamp(1.0 - xteAbsM / directBand, 0.0, 1.0), 0.70);
            const bool tillerStillTowardLine = (smoothedSteer_ * xteM) > std::max(0.0, cfg_.microAnticipateTowardSteerThreshold);

            if (headingTowardCenter || predictionCrossesLine || tillerStillTowardLine) {
                microReturnBlend = clamp(std::max({microLeadBlend, timeBlend, bandBlend}), 0.0, 1.0);
                // If XTE is positive, the aircraft is on one side and the old correction
                // usually has the same sign as XTE. Return/rollout steering must therefore
                // be opposite sign, before the sign of XTE changes.
                microReturnSign = -std::copysign(1.0, xteM);
            }
        }
    }

    double captureBlend = 0.0;
    if (cfg_.fastRouteCapture) {
        const double soft = std::max(0.25, cfg_.fastCaptureThresholdM);
        const double hard = std::max(soft + 0.25, cfg_.fastCaptureHardThresholdM);
        captureBlend = clamp((xteAbsM - soft) / (hard - soft), 0.0, 1.0);
        if (justStartedCapture && xteAbsM > std::max(0.5, cfg_.nearRouteDampingM)) {
            const double startT = clamp((cfg_.fastCaptureStartSeconds - activeTimeSec_) /
                                        std::max(1.0, cfg_.fastCaptureStartSeconds), 0.0, 1.0);
            captureBlend = std::max(captureBlend, 0.45 * startT);
        }

        // Once inside the desired XTE corridor, fade out the aggressive capture mode.
        // This is what prevents the aircraft from correcting hard at XTE 3-5 m,
        // crossing the line, then correcting hard the other way. Outside the corridor
        // capture remains available, but the corridor logic above manages the first
        // intercept to land inside +/- xte_corridor_m.
        if (xteCorridorAvailable) {
            const double corridorM = std::max(0.5, cfg_.xteCorridorM);
            const double fade = clamp(cfg_.xteCorridorCaptureFade, 0.0, 1.0);
            if (xteAbsM <= corridorM) {
                captureBlend *= fade;
            } else if (xteAbsM < corridorM + 2.0) {
                const double t = clamp((xteAbsM - corridorM) / 2.0, 0.0, 1.0);
                captureBlend *= fade + (1.0 - fade) * t;
            }
        }
    }

    // When we have just crossed the route centerline and the remaining error is small,
    // suppress the aggressive capture blend and reset derivative/integral memory. This
    // is the key anti-oscillation step: do not keep commanding a high-gain correction
    // after the aircraft is already back over the planned taxi line.
    if (crossedRouteCenterline && xteAbsM < std::max(1.0, cfg_.nearRouteDampingM) * 1.8) {
        captureBlend *= 0.20;
        pidIntegralDegSec_ = 0.0;
        pidDerivativeFilteredDegPerSec_ = 0.0;
        pidPrevValid_ = false;
    }

    const double normalGain = std::max(0.0, cfg_.pidCrossTrackGainDegPerM);
    const double fastGain = std::max(normalGain, cfg_.fastCaptureGainDegPerM);
    double xteGain = normalGain + captureBlend * (fastGain - normalGain);
    if (microLeadBlend > 0.0) {
        xteGain *= 1.0 + microLeadBlend * std::max(0.0, cfg_.microAnticipateBiasBoost - 1.0);
    }
    if (xteCorridorAvailable && xteCorridorBlend > 0.0) {
        const double innerScale = clamp(cfg_.xteCorridorInnerGain, 0.05, 1.0);
        xteGain *= (1.0 - xteCorridorBlend) + xteCorridorBlend * innerScale;
    }
    const double normalMaxBias = std::max(0.0, cfg_.pidCrossTrackMaxDeg);
    const double fastMaxBias = std::max(normalMaxBias, cfg_.fastCaptureMaxBiasDeg);
    double maxXteBiasDeg = normalMaxBias + captureBlend * (fastMaxBias - normalMaxBias);
    if (xteCorridorAvailable && xteCorridorBlend > 0.0) {
        maxXteBiasDeg = std::min(maxXteBiasDeg, std::max(2.0, cfg_.xteCorridorMaxInterceptDeg));
    } else if (xteCorridorAvailable && xteCorridorOuterBlend > 0.0) {
        maxXteBiasDeg = std::min(maxXteBiasDeg, std::max(cfg_.xteCorridorMaxInterceptDeg, cfg_.xteCorridorOuterMaxInterceptDeg));
    }
    const double xteBiasDeg = clamp(xteForControlM * xteGain, -maxXteBiasDeg, maxXteBiasDeg);

    const double lookaheadHeadingErr =
        geo::diffSignedDeg(geo::normalize360(track.desiredTrackDeg + xteBiasDeg), psi);

    // Direct track-course capture. The older controller mainly aimed at a look-ahead
    // point; on a long straight this can wait until XTE is already several metres away
    // before producing meaningful tiller. Here we command the current taxiway course
    // plus a Stanley/L1-style intercept angle from XTE. This starts steering back as
    // soon as XTE exists. When the aircraft is already cutting toward the centerline,
    // we deliberately soften or reverse the effective XTE before it reaches zero, so
    // the tiller starts returning before the map value crosses 0.0 m.
    double courseHeadingErr = lookaheadHeadingErr;
    double trackCoursePrezeroBlend = 0.0;
    if (cfg_.trackCourseControl) {
        double courseXteM = xteForControlM;
        const double courseBand = std::max(0.5, cfg_.trackCoursePrezeroBandM);
        const double courseLeadSec = std::max(0.0, cfg_.trackCoursePrezeroLeadSec);
        const double rateWeight = clamp(cfg_.microAnticipateGeomRateWeight, 0.0, 1.0);
        const double blendedRate = xteRateMps * (1.0 - rateWeight) + geomXteRateMps * rateWeight;
        const bool headingTowardCenter = (xteM * geomXteRateMps) < 0.0;
        const bool closeEnoughForRollout = xteAbsM <= courseBand;
        const bool meaningfulHeading =
            std::abs(pathHeadingErrForLead) >= std::max(0.0, cfg_.trackCoursePrezeroMinHeadingDeg);

        if (closeEnoughForRollout && headingTowardCenter && meaningfulHeading) {
            const double bandT = clamp(1.0 - xteAbsM / courseBand, 0.0, 1.0);
            const double absScale = std::pow(clamp(xteAbsM / courseBand, 0.0, 1.0),
                                             std::max(0.35, cfg_.trackCoursePrezeroPower));
            const double softened = std::copysign(xteAbsM * absScale, xteM);

            double predicted = xteM + blendedRate * courseLeadSec;
            const double maxLead = std::max(courseBand, cfg_.predictiveXteMaxLeadM);
            predicted = clamp(predicted, -maxLead, maxLead);

            const double headingT = clamp(std::abs(pathHeadingErrForLead) / 8.0, 0.0, 1.0);
            const double counterBias = std::copysign(std::max(0.0, cfg_.trackCoursePrezeroCounterM) *
                                                     bandT * headingT, xteM);
            const double prezero = predicted - counterBias;

            // Pick the most conservative-to-center value: it must never be larger in
            // magnitude than current XTE when we are already moving toward the line.
            courseXteM = softened;
            if (std::abs(prezero) < std::abs(courseXteM) || prezero * xteM < 0.0) {
                courseXteM = prezero;
            }
            trackCoursePrezeroBlend = bandT;
            xteForControlM = courseXteM;
        }

        const double interceptDistM = std::max(6.0, cfg_.trackCourseLookaheadM +
            gsKts * std::max(0.0, cfg_.trackCourseSpeedGainMPerKt));
        const double interceptLimitDeg = std::max(2.0, xteCorridorTrackMaxInterceptDeg);
        const double interceptDeg = clamp(
            geo::rad2deg(std::atan2(courseXteM * std::max(0.05, cfg_.trackCourseGain), interceptDistM)),
            -interceptLimitDeg,
             interceptLimitDeg);
        courseHeadingErr = geo::diffSignedDeg(geo::normalize360(track.pathHeadingDeg + interceptDeg), psi);
    }

    double routeTrackFadeOut = clamp(tightTurnBlend * clamp(cfg_.trackCourseTightTurnFade, 0.0, 1.0), 0.0, 1.0);
    if (earlyTurnBlend > 0.0) {
        routeTrackFadeOut = std::max(routeTrackFadeOut,
            clamp(earlyTurnBlend * clamp(cfg_.earlyTurnTrackCourseFade, 0.0, 1.0), 0.0, 1.0));
    }
    const double courseFade = 1.0 - routeTrackFadeOut;
    const double courseBlend = clamp(cfg_.trackCourseBlend, 0.0, 1.0) * courseFade;
    double headingErr = lookaheadHeadingErr * (1.0 - courseBlend) + courseHeadingErr * courseBlend;
    if (earlyTurnBlend > 0.0 && outboundHeadingValid) {
        const double outboundErrForTakeover = geo::diffSignedDeg(outboundHeadingDeg, psi);
        const double takeoverBlend = clamp(earlyTurnBlend * clamp(cfg_.earlyTurnHeadingBlend, 0.0, 1.0), 0.0, 1.0);
        headingErr = headingErr * (1.0 - takeoverBlend) + outboundErrForTakeover * takeoverBlend;
    }
    if (runwayAlignActive && runwayAlignGeoValid) {
        // Centreline + heading alignment for runway destinations. If the aircraft is
        // left of the selected runway centreline, bias the desired heading right of
        // runway heading; if it is right, bias left. As the aircraft gets closer to
        // the threshold/route end this overrides taxi-route tracking.
        const double centreInterceptDeg = clamp(-runwaySignedXteM * std::max(0.0, cfg_.runwayAlignCenterGainDegPerM),
                                                -std::max(1.0, cfg_.runwayAlignMaxInterceptDeg),
                                                 std::max(1.0, cfg_.runwayAlignMaxInterceptDeg));
        const double desiredRunwayDeg = geo::normalize360(runwayHeadingDeg + centreInterceptDeg);
        const double runwayErr = geo::diffSignedDeg(desiredRunwayDeg, psi);
        const double blend = clamp(runwayAlignBlend, 0.0, 1.0);
        headingErr = headingErr * (1.0 - blend) + runwayErr * blend;
    }
    headingErr = clamp(headingErr, -180.0, 180.0);
    if ((microLeadBlend > 0.0 || trackCoursePrezeroBlend > 0.0) &&
        std::abs(headingErr) < std::max(0.0, cfg_.microAnticipateDeadbandDeg)) {
        headingErr = 0.0;
    }
    const double outboundHeadingErr = outboundHeadingErrForTurn;
    const double pathHeadingErr = pathHeadingErrForLead;
    lastCrossTrackErrorM_ = xteM;
    lastPathHeadingErrorDeg_ = pathHeadingErr;

    double targetSpeedMps = geo::knotsToMps(cfg_.taxiSpeedKts);
    const double turnDeg = upcomingTurnDeg(waypointIndex_);
    if (turnDeg > cfg_.turnSlowdownThresholdDeg) {
        targetSpeedMps = std::min(targetSpeedMps, geo::knotsToMps(cfg_.sharpTurnSpeedKts));
    }
    if (tightTurnBlend > 0.0) {
        const double tightSpeed = geo::knotsToMps(std::max(2.0, cfg_.tightTurnSpeedKts));
        targetSpeedMps = tightSpeed + (1.0 - tightTurnBlend) * (targetSpeedMps - tightSpeed);
    } else if (cfg_.turnAnticipation && upcomingTurnAngleDeg > cfg_.turnAnticipationMinAngleDeg &&
        track.distanceToSegmentEndM < cfg_.turnAnticipationDistanceM) {
        const double distT = 1.0 - clamp(track.distanceToSegmentEndM /
                                        std::max(10.0, cfg_.turnAnticipationDistanceM), 0.0, 1.0);
        const double turnT = clamp((upcomingTurnAngleDeg - cfg_.turnAnticipationMinAngleDeg) /
                                   std::max(1.0, 95.0 - cfg_.turnAnticipationMinAngleDeg), 0.0, 1.0);
        const double anticipatedSpeed = geo::knotsToMps(std::max(2.5, cfg_.turnAnticipationSlowdownKts));
        const double blend = clamp(distT * turnT, 0.0, 1.0);
        targetSpeedMps = std::min(targetSpeedMps, anticipatedSpeed + (1.0 - blend) * (targetSpeedMps - anticipatedSpeed));
    }
    if (std::abs(headingErr) > 35.0 || std::abs(track.signedCrossTrackM) > 18.0) {
        targetSpeedMps = std::min(targetSpeedMps, geo::knotsToMps(cfg_.slowSpeedKts));
    }
    if (terminalGuardActive) {
        const double slowDist = std::max(terminalStopRadiusM + 1.0, cfg_.terminalSlowdownDistanceM);
        const double t = clamp(terminalRemainingM / slowDist, 0.0, 1.0);
        const double terminalSpeed = geo::knotsToMps(std::max(0.8, cfg_.terminalSpeedKts));
        targetSpeedMps = std::min(targetSpeedMps, terminalSpeed + t * (targetSpeedMps - terminalSpeed));
    } else if (finalSegment && finalDistM < 160.0) {
        const double t = clamp(finalDistM / 160.0, 0.0, 1.0);
        const double slow = geo::knotsToMps(cfg_.finalSpeedKts);
        targetSpeedMps = slow + t * (targetSpeedMps - slow);
    }
    if (runwayAlignActive && runwayAlignGeoValid) {
        const double alignSpeed = geo::knotsToMps(std::max(1.0, cfg_.runwayAlignSpeedKts));
        const double blend = clamp(runwayAlignBlend, 0.0, 1.0);
        targetSpeedMps = targetSpeedMps * (1.0 - blend) + alignSpeed * blend;
    }

    // AP-like PID yaw/path controller. Output is a commanded correction angle, then mapped
    // to tiller/steering ratio with speed scheduling and rate limiting.
    double pidOutputDeg = headingErr;
    if (cfg_.pidRouteTracking) {
        const double captureLimit = std::max(8.0, cfg_.maxTrackCaptureAngleDeg);
        const double limitedErr = clamp(headingErr, -captureLimit, captureLimit);
        pidIntegralDegSec_ += limitedErr * dt;
        const double intLim = std::max(1.0, cfg_.pidIntegralLimitDegSec);
        pidIntegralDegSec_ = clamp(pidIntegralDegSec_, -intLim, intLim);

        double derivative = 0.0;
        if (pidPrevValid_) {
            const double rawDerivative = clamp(geo::diffSignedDeg(limitedErr, pidPrevHeadingErrDeg_) /
                                               std::max(0.02, dt), -90.0, 90.0);
            const double filterSec = std::max(0.02, cfg_.pidDerivativeFilterSec);
            const double alpha = clamp(dt / filterSec, 0.0, 1.0);
            pidDerivativeFilteredDegPerSec_ += alpha * (rawDerivative - pidDerivativeFilteredDegPerSec_);
            derivative = pidDerivativeFilteredDegPerSec_;
        } else {
            pidDerivativeFilteredDegPerSec_ = 0.0;
        }
        pidPrevHeadingErrDeg_ = limitedErr;
        pidPrevValid_ = true;

        const double corridorKpScale = xteCorridorAvailable
            ? ((1.0 - xteCorridorBlend) + xteCorridorBlend * clamp(cfg_.xteCorridorKpScale, 0.10, 1.0))
            : 1.0;
        const double corridorKdScale = xteCorridorAvailable
            ? (1.0 + xteCorridorBlend * std::max(0.0, cfg_.xteCorridorKdBoost - 1.0))
            : 1.0;
        const double kp = cfg_.pidHeadingKp * corridorKpScale *
            (1.0 + captureBlend * std::max(0.0, cfg_.fastCaptureKpBoost - 1.0)) *
            (1.0 + tightTurnBlend * std::max(0.0, cfg_.tightTurnKpBoost - 1.0));
        const double kd = cfg_.pidHeadingKd * corridorKdScale *
            (1.0 + captureBlend * std::max(0.0, cfg_.fastCaptureKdBoost - 1.0)) *
            (1.0 + tightTurnBlend * std::max(0.0, cfg_.tightTurnKdBoost - 1.0));

        pidOutputDeg = kp * limitedErr
                     + cfg_.pidHeadingKi * pidIntegralDegSec_
                     + kd * derivative;
        pidOutputDeg = clamp(pidOutputDeg, -captureLimit, captureLimit);
    }

    // Micro-course pre-zero rollout. This is intentionally applied after PID so it
    // can override a tiny same-direction correction. Without this, the displayed XTE
    // can cross zero first and only then will the PID sign reverse.
    if (microReturnBlend > 0.0 && microReturnSign != 0.0) {
        const double blend = clamp(microReturnBlend * std::max(0.0, cfg_.microAnticipateDirectBlend), 0.0, 1.0);
        const double returnPidDeg = microReturnSign * std::max(0.0, cfg_.microAnticipatePidReturnDeg) *
                                    (0.35 + 0.65 * microReturnBlend);
        const bool pidStillTowardLine = (pidOutputDeg * xteM) > 0.0;
        if (pidStillTowardLine) {
            pidOutputDeg = pidOutputDeg * (1.0 - blend) + returnPidDeg * blend;
        } else if (pidOutputDeg * microReturnSign > 0.0 && std::abs(pidOutputDeg) < std::abs(returnPidDeg)) {
            pidOutputDeg = pidOutputDeg * (1.0 - 0.65 * blend) + returnPidDeg * (0.65 * blend);
        }
        pidIntegralDegSec_ *= std::max(0.0, 1.0 - 0.85 * blend);
    }

    // Corridor pre-zero rollout / anti-sway. If we are already inside the desired
    // XTE corridor and the nose is cutting toward the centerline, do not keep a
    // large same-direction yaw command. Blend toward a small opposite command before
    // XTE reaches 0 m. This is separate from the older micro-return logic because
    // it also triggers at 3-5 m XTE, not only very close to the centerline.
    if (xteCorridorAvailable && xteCorridorSteerDampBlend > 0.0 && xteAbsM > 0.05) {
        const bool headingTowardCenter = (xteM * geomXteRateMps) < 0.0;
        const bool pidTowardCenter = (pidOutputDeg * xteM) > 0.0;
        if (headingTowardCenter && pidTowardCenter) {
            const double blend = clamp(xteCorridorSteerDampBlend, 0.0, 1.0);
            const double returnSign = -std::copysign(1.0, xteM);
            const double returnPidDeg = returnSign * std::max(0.0, cfg_.xteCorridorPrezeroReturnDeg) * blend;
            pidOutputDeg = pidOutputDeg * (1.0 - 0.70 * blend) + returnPidDeg * (0.70 * blend);
            pidIntegralDegSec_ *= std::max(0.0, 1.0 - 0.90 * blend);
        }
        const double cap = std::max(3.0, cfg_.xteCorridorMaxInterceptDeg) +
            (1.0 - xteCorridorBlend) * 6.0;
        pidOutputDeg = clamp(pidOutputDeg, -cap, cap);
    }

    // One-pass 90-degree turn rollout. For a tight 70-100 degree corner we want an
    // immediate, high tiller command at entry, but we do not want to keep that high
    // command after the nose is nearly aligned with the outbound taxiway. This caps
    // the remaining commanded angle as the aircraft approaches the next-leg heading.
    if (hardOnePassTurn && outboundHeadingValid) {
        const double rolloutBand = std::max(4.0, cfg_.tightTurnRolloutHeadingDeg);
        const double absOutErr = std::abs(outboundHeadingErr);
        const bool stillTurningTowardOutbound = pidOutputDeg * outboundHeadingErr > 0.0;
        if (absOutErr < rolloutBand) {
            const double allowedPid = std::max(std::max(0.0, cfg_.tightTurnRolloutMinPidDeg),
                                               absOutErr * std::max(0.1, cfg_.tightTurnRolloutGain));
            if (stillTurningTowardOutbound) {
                pidOutputDeg = std::copysign(std::min(std::abs(pidOutputDeg), allowedPid), pidOutputDeg);
            } else if (absOutErr < rolloutBand * 0.50) {
                pidOutputDeg = clamp(pidOutputDeg, -allowedPid, allowedPid);
            }
        }
    }

    lastPidOutputDeg_ = pidOutputDeg;

    double targetSteer = 0.0;
    const double activeSteerDeadbandDeg = (microLeadBlend > 0.0 || trackCoursePrezeroBlend > 0.0)
        ? std::min(cfg_.steerDeadbandDeg, std::max(0.0, cfg_.microAnticipateDeadbandDeg))
        : cfg_.steerDeadbandDeg;
    if (std::abs(pidOutputDeg) >= activeSteerDeadbandDeg) {
        const double smoothFullDeflection = std::max(10.0, cfg_.steerFullDeflectionDeg);
        const double captureFullDeflection = std::max(10.0, cfg_.fastCaptureSteerFullDeflectionDeg);
        double fullDeflection = smoothFullDeflection + captureBlend * (captureFullDeflection - smoothFullDeflection);
        const double tightFullDeflection = std::max(8.0, cfg_.tightTurnSteerFullDeflectionDeg);
        fullDeflection += tightTurnBlend * (tightFullDeflection - fullDeflection);
        double normalized = clamp(pidOutputDeg / fullDeflection, -1.0, 1.0);
        const double exponent = std::max(0.50, cfg_.steerCurveExponent);
        normalized = std::copysign(std::pow(std::abs(normalized), exponent), normalized);

        const double lowCap = clamp(cfg_.lowSpeedMaxSteerRatio, 0.05, 1.0);
        const double highCap = clamp(cfg_.highSpeedMaxSteerRatio, 0.03, lowCap);
        const double maxCap = clamp(cfg_.maxSteerRatio, 0.05, 1.0);
        const double highKts = std::max(3.0, cfg_.highSpeedSteerKts);
        const double speedT = clamp((gsKts - 2.0) / std::max(1.0, highKts - 2.0), 0.0, 1.0);
        const double normalScheduledCap = std::min(maxCap, lowCap + speedT * (highCap - lowCap));
        const double tightScheduledCap = normalScheduledCap + tightTurnBlend * (maxCap - normalScheduledCap);
        targetSteer = clamp(normalized, -tightScheduledCap, tightScheduledCap);
    } else {
        // If the error is inside deadband, bleed the integrator very slowly instead of
        // abruptly snapping the steer command through zero.
        pidIntegralDegSec_ *= std::max(0.0, 1.0 - dt * 0.25);
    }

    // Direct target-steer pre-zero return. This is the practical fix for the symptom
    // you observed: during tiny course corrections the normal path/PID output can keep
    // the same sign until XTE has already crossed zero. Here we alter the requested
    // steer itself while XTE is still on the original side, so the tiller starts
    // unwinding/counter-steering before the line is crossed.
    if (cfg_.microAnticipateDirectSteer && microReturnBlend > 0.0 && microReturnSign != 0.0) {
        const double blend = clamp(microReturnBlend * std::max(0.0, cfg_.microAnticipateDirectBlend), 0.0, 1.0);
        const double counterMax = clamp(cfg_.microAnticipateCounterSteerRatio, 0.0, 1.0);
        const double counterMin = clamp(cfg_.microAnticipateCounterMinRatio, 0.0, counterMax);
        const double counterMag = counterMin + (counterMax - counterMin) * microReturnBlend;
        const double counterTarget = microReturnSign * counterMag;
        const bool targetStillTowardLine = (targetSteer * xteM) > 0.0;
        if (targetStillTowardLine) {
            targetSteer = targetSteer * (1.0 - blend) + counterTarget * blend;
        } else if (targetSteer * microReturnSign > 0.0 && std::abs(targetSteer) < counterMag) {
            targetSteer = targetSteer * (1.0 - 0.75 * blend) + counterTarget * (0.75 * blend);
        }
    }

    if (xteCorridorAvailable && xteCorridorSteerDampBlend > 0.0 && tightTurnBlend < 0.08 && earlyTurnBlend <= 0.0) {
        const double blend = clamp(xteCorridorSteerDampBlend, 0.0, 1.0);
        const double scale = clamp(cfg_.swayDampingCommandScale, 0.20, 1.0);
        const bool targetTowardCenter = (targetSteer * xteM) > 0.0;
        if (targetTowardCenter) {
            targetSteer *= (1.0 - blend) + blend * scale;
        }
    }

    const double captureSmoothingPerSec = cfg_.steerSmoothingPerSec +
        captureBlend * (cfg_.fastCaptureSteerSmoothingPerSec - cfg_.steerSmoothingPerSec);
    const double tightSmoothingPerSec = cfg_.steerSmoothingPerSec +
        tightTurnBlend * (cfg_.tightTurnSteerSmoothingPerSec - cfg_.steerSmoothingPerSec);
    double smoothingPerSec = std::max({cfg_.steerSmoothingPerSec,
                                       captureSmoothingPerSec,
                                       tightSmoothingPerSec});

    const bool corridorFineMode = xteCorridorAvailable && xteAbsM <= std::max(0.5, cfg_.xteCorridorM) &&
        tightTurnBlend < 0.08 && earlyTurnBlend <= 0.0;
    const bool fastSteerDemand = cfg_.steerFastResponse &&
        (tightTurnBlend > 0.05 || earlyTurnBlend > 0.0 || captureBlend > 0.10 ||
         (!corridorFineMode && microLeadBlend > 0.10) ||
         (!corridorFineMode && microReturnBlend > 0.05) ||
         (!corridorFineMode && trackCoursePrezeroBlend > 0.05) ||
         (!corridorFineMode && std::abs(pidOutputDeg) >= std::max(1.0, cfg_.steerFastResponseErrorDeg)) ||
         (!corridorFineMode && std::abs(xteForControlM) >= std::max(0.1, cfg_.steerFastResponseXteM)));
    if (fastSteerDemand) {
        smoothingPerSec = std::max(smoothingPerSec, std::max(0.5, cfg_.steerFastResponseRatePerSec));
    }
    if (corridorFineMode && xteCorridorSteerDampBlend > 0.0) {
        const double blend = clamp(xteCorridorSteerDampBlend, 0.0, 1.0);
        const double fineRate = std::max(0.20, cfg_.swayDampingSteerRatePerSec);
        smoothingPerSec = smoothingPerSec * (1.0 - blend) + fineRate * blend;
    }
    if (hardOnePassTurn) {
        smoothingPerSec = std::max(smoothingPerSec, std::max(1.0, cfg_.tightTurnSteerSmoothingPerSec));
    }

    // Absolute force-full-turn command. This is intentionally stronger than the
    // existing snap floor: for large turns it sets the smoothed command immediately
    // to full tiller, so there is no slow ramp through 0.2/0.4/0.6 first.
    if (forceFullTurnActive && outboundHeadingValid) {
        double turnSide = std::abs(outboundHeadingErr) > 2.0
            ? std::copysign(1.0, outboundHeadingErr)
            : upcomingTurnSign;
        if (turnSide != 0.0) {
            const double ratio = clamp(cfg_.tightTurnForceFullSteerRatio, 0.05, 1.0);
            targetSteer = turnSide * ratio;
            smoothedSteer_ = targetSteer;
            smoothingPerSec = std::max(smoothingPerSec, 120.0);
            pidIntegralDegSec_ = 0.0;
        }
    }

    // Early hard-turn snap floor. This is intentionally before the regular snap code:
    // if track-course/route-following has kept the target too small while a big corner
    // is approaching, force a turn-side steering request while still far enough away.
    if (cfg_.earlyTurnTakeover && earlyTurnBlend > 0.0 && hardOnePassTurn && outboundHeadingValid) {
        const double rolloutBand = std::max(4.0, cfg_.tightTurnRolloutHeadingDeg);
        const bool rolloutSoon = std::abs(outboundHeadingErr) < rolloutBand;
        double turnSide = std::abs(outboundHeadingErr) > 2.0
            ? std::copysign(1.0, outboundHeadingErr)
            : upcomingTurnSign;
        if (!rolloutSoon && turnSide != 0.0) {
            const double snapTo = clamp(cfg_.earlyTurnSnapToRatio, 0.0, 1.0);
            const double minFloor = clamp(cfg_.earlyTurnSnapMinTargetRatio, 0.0, snapTo);
            const double snapFloor = clamp(minFloor * earlyTurnBlend, 0.0, snapTo);
            if (snapFloor > 0.0 && (targetSteer * turnSide < snapFloor)) {
                targetSteer = turnSide * std::max(std::abs(targetSteer), snapFloor);
            }
        }
    }

    // Optional snap-in for tight turns: if we are close to a real taxiway corner and the
    // requested tiller already has a clear sign, do not spend a full second ramping from
    // zero. This is what reduces the initial wide arc before the A350 actually turns.
    if (cfg_.tightTurnSnapSteer && (tightTurnBlend >= std::max(0.0, cfg_.tightTurnSnapBlend) || earlyTurnBlend > 0.0)) {
        double snapTo = clamp(cfg_.tightTurnSnapToRatio, 0.0, 1.0);
        double snapMinTarget = std::max(0.0, cfg_.tightTurnSnapMinTargetRatio);
        double snapFloor = 0.0;
        if (hardOnePassTurn) {
            snapTo = std::max(snapTo, clamp(cfg_.tightTurnHardSnapToRatio, 0.0, 1.0));
            snapFloor = clamp(cfg_.tightTurnHardSnapMinTargetRatio, 0.0, snapTo);
        }

        // Do not hard-snap during rollout; once the outbound heading is nearly captured,
        // let the PID/rollout cap unwind steering quickly instead of forcing more tiller.
        const bool rolloutSoon = hardOnePassTurn && outboundHeadingValid &&
            std::abs(outboundHeadingErr) < std::max(4.0, cfg_.tightTurnRolloutHeadingDeg);

        if (!rolloutSoon && std::abs(targetSteer) >= snapMinTarget) {
            double snapAbs = std::min(std::abs(targetSteer), snapTo);
            if (hardOnePassTurn) snapAbs = std::max(snapAbs, snapFloor);
            const double snapValue = std::copysign(snapAbs, targetSteer);
            const bool sameDirection = smoothedSteer_ * targetSteer >= 0.0;
            if (sameDirection && std::abs(smoothedSteer_) < std::abs(snapValue)) {
                smoothedSteer_ = snapValue;
            }
        }
    }

    const double maxStep = std::max(0.05, smoothingPerSec) * dt;
    smoothedSteer_ += clamp(targetSteer - smoothedSteer_, -maxStep, maxStep);

    double throttle = 0.0;
    double baseBrake = 0.0;
    const double speedErr = targetSpeedMps - gsMps;
    const double targetSpeedKts = targetSpeedMps / geo::kKnotToMps;
    const double speedErrKts = targetSpeedKts - gsKts;

    // Dynamic throttle governor. Unlike the earlier fixed-step throttle, this keeps
    // adjusting thrust every frame to hold the scheduled taxi speed. It backs out of
    // thrust in tight turns, aggressive path capture and terminal/hold-short arrival,
    // then adds thrust again when the aircraft falls below target speed.
    const double captureSeverity = clamp(std::abs(pidOutputDeg) / std::max(1.0, cfg_.maxTrackCaptureAngleDeg), 0.0, 1.0);
    const double terminalSeverity = terminalGuardActive
        ? clamp((terminalNoTightDistanceM - terminalRemainingM) / std::max(1.0, terminalNoTightDistanceM), 0.0, 1.0)
        : 0.0;
    double throttleAuthorityScale = 1.0;
    throttleAuthorityScale *= 1.0 - clamp(cfg_.throttleCaptureReduction, 0.0, 0.95) * captureSeverity;
    throttleAuthorityScale *= 1.0 - clamp(cfg_.throttleTurnReduction, 0.0, 0.95) * tightTurnBlend;
    throttleAuthorityScale *= 1.0 - clamp(cfg_.throttleTerminalReduction, 0.0, 0.95) * terminalSeverity;
    throttleAuthorityScale = clamp(throttleAuthorityScale, 0.12, 1.0);

    std::string throttleMode = "idle";
    if (cfg_.dynamicThrottleControl) {
        double desiredThrottle = 0.0;
        const bool headingAllowsThrust = std::abs(headingErr) < 125.0;
        if (headingAllowsThrust && targetSpeedMps > 0.4) {
            if (speedErrKts > 0.15) {
                desiredThrottle = cfg_.throttleIdle + cfg_.throttleSpeedKp * speedErrKts;
                if (gsKts > 0.8) desiredThrottle = std::max(desiredThrottle, cfg_.throttleMinMoving);
                desiredThrottle *= throttleAuthorityScale;
                throttleMode = "speed+";
            } else if (speedErrKts > -0.25 && gsKts > 0.8) {
                desiredThrottle = cfg_.throttleTrim * throttleAuthorityScale;
                throttleMode = "hold";
            }
        }

        if (gsMps < geo::knotsToMps(1.2) && headingAllowsThrust && targetSpeedMps > 0.5) {
            desiredThrottle = std::max(desiredThrottle, std::min(cfg_.creepThrottle, cfg_.maxThrottle) *
                                                       (0.65 + 0.35 * throttleAuthorityScale));
            throttleMode = "creep";
        }

        desiredThrottle = clamp(desiredThrottle, 0.0, std::max(0.0, cfg_.maxThrottle));
        const double rate = desiredThrottle > lastThrottleCmd_
            ? std::max(0.01, cfg_.throttleIncreaseRatePerSec)
            : std::max(0.01, cfg_.throttleDecreaseRatePerSec);
        throttle = lastThrottleCmd_ + clamp(desiredThrottle - lastThrottleCmd_, -rate * dt, rate * dt);
        if (desiredThrottle <= 0.001 && throttle < 0.010) throttle = 0.0;
    } else {
        // Legacy fallback. Kept for easy comparison while tuning.
        const double throttleScale = 1.0 - 0.45 * captureSeverity;
        if (speedErr > 0.35 && std::abs(headingErr) < 85.0) {
            throttle = cfg_.maxThrottle * throttleScale * clamp(speedErr / std::max(0.5, targetSpeedMps), 0.20, 1.0);
            throttleMode = "legacy";
        }
        if (gsMps < geo::knotsToMps(1.2) && std::abs(headingErr) < 150.0 && targetSpeedMps > 0.5) {
            throttle = std::max(throttle, std::min(cfg_.creepThrottle, cfg_.maxThrottle));
            throttleMode = "creep";
        }
    }

    if (gsMps < geo::knotsToMps(0.7) && throttle > 0.0 && !finalSegment) {
        stuckSeconds_ += dt;
    } else {
        stuckSeconds_ = 0.0;
    }

    const bool stuckBoostActive = stuckSeconds_ >= cfg_.stuckBoostSeconds && !finalSegment;
    if (stuckBoostActive) {
        throttle = std::max(throttle, cfg_.breakawayThrottle);
        throttleMode = "breakaway";
        baseBrake = 0.0;
        if (cfg_.commandAssist) releaseBrakeCommands();
    }

    const double overspeedKts = -speedErrKts;
    if (overspeedKts > std::max(0.0, cfg_.throttleOverspeedBrakeDeadbandKts)) {
        throttle = 0.0;
        throttleMode = "brake";
        baseBrake = clamp(overspeedKts * cfg_.brakeGain * 0.55, 0.0, cfg_.maxBrake);
    }

    if (speedErr < -0.20) {
        baseBrake = std::max(baseBrake, clamp((-speedErr) * cfg_.brakeGain, 0.0, cfg_.maxBrake));
    }

    if ((std::abs(pidOutputDeg) > 55.0 || std::abs(track.signedCrossTrackM) > 35.0) && gsMps > geo::knotsToMps(5.0)) {
        throttle = 0.0;
        throttleMode = "capture-brake";
        baseBrake = std::max(baseBrake, 0.35);
    }

    lastTargetSpeedKts_ = targetSpeedKts;
    throttleMode_ = throttleMode;

    double leftBrake = baseBrake;
    double rightBrake = baseBrake;

    // Stronger differential brake is allowed now, but only once rolling. It is blended by
    // PID output so it helps large corrections without creating a parked-aircraft spin lock.
    const double diffBrakeMinKts = 2.5 + tightTurnBlend *
        (std::max(0.2, cfg_.tightTurnDifferentialBrakeMinKts) - 2.5);
    const double diffBrakeMax = cfg_.differentialBrakeMax + tightTurnBlend *
        (std::max(cfg_.differentialBrakeMax, cfg_.tightTurnDifferentialBrakeMax) - cfg_.differentialBrakeMax);
    if (!terminalGuardActive && !stuckBoostActive && gsMps > geo::knotsToMps(diffBrakeMinKts) && gsMps < geo::knotsToMps(10.0) &&
        std::abs(pidOutputDeg) > cfg_.differentialBrakeThresholdDeg) {
        const double diff = diffBrakeMax *
            clamp((std::abs(pidOutputDeg) - cfg_.differentialBrakeThresholdDeg) / 45.0, 0.0, 1.0);
        if (pidOutputDeg > 0.0) rightBrake = clamp(rightBrake + diff, 0.0, cfg_.maxBrake);
        else leftBrake = clamp(leftBrake + diff, 0.0, cfg_.maxBrake);
    }

    if (stuckBoostActive) {
        leftBrake = 0.0;
        rightBrake = 0.0;
    }

    prevSignedCrossTrackM_ = xteM;
    prevXteValid_ = true;

    writeFctlSecondaryAssist(tightTurnBlend);
    writeControls(smoothedSteer_, throttle, leftBrake, rightBrake);
    pulseThrottleAssist(throttle, gsMps, dt);
}

void AutoTaxiSystem::captureFctlSecondaryBaseline() {
    if (secondaryFctlBaselineCaptured_) return;
    if (drFctlFlapRatio_) savedFlapRatio_ = getDataf(drFctlFlapRatio_, 0.0);
    if (drFctlFlapRequest_) savedFlapRequest_ = getDataf(drFctlFlapRequest_, savedFlapRatio_);
    if (drFctlSlatRatio_) savedSlatRatio_ = getDataf(drFctlSlatRatio_, savedFlapRatio_);
    secondaryFctlBaselineCaptured_ = true;
}

void AutoTaxiSystem::writeFctlSecondaryAssist(double tightTurnBlend) {
    if (!active_ || !cfg_.fctlTurnCoupling || !cfg_.fctlSecondaryAssist) return;
    if (tightTurnBlend < std::max(0.0, cfg_.fctlSecondaryMinTightBlend)) return;

    captureFctlSecondaryBaseline();
    const double target = clamp(cfg_.fctlSecondaryFlapRatio, 0.0, 1.0);
    const double flapTarget = std::max(savedFlapRatio_, target);
    const double requestTarget = std::max(savedFlapRequest_, target);
    const double slatTarget = std::max(savedSlatRatio_, target);

    auto writeIfWritable = [](XPLMDataRef ref, double value) {
        if (!ref || !XPLMCanWriteDataRef(ref)) return;
        XPLMSetDataf(ref, static_cast<float>(value));
    };

    writeIfWritable(drFctlFlapRatio_, flapTarget);
    writeIfWritable(drFctlFlapRequest_, requestTarget);
    writeIfWritable(drFctlSlatRatio_, slatTarget);
    secondaryFctlAssistHasCommanded_ = true;
}

void AutoTaxiSystem::writeFctlCoupling(double steerRatio) {
    lastFctlRudderCmd_ = 0.0;
    lastFctlRollCmd_ = 0.0;

    if (!cfg_.fctlTurnCoupling || !active_) {
        return;
    }

    const double rudderCmd = clamp(steerRatio * cfg_.fctlRudderGain, -cfg_.fctlMaxRatio, cfg_.fctlMaxRatio);
    const double rollCmd = clamp(steerRatio * cfg_.fctlRollGain, -cfg_.fctlMaxRatio, cfg_.fctlMaxRatio);
    const bool rudderSharesPrimarySteer = (drFctlRudder_ && drFctlRudder_ == drSteer_) ||
                                          (drFctlRudderLegacy_ && drFctlRudderLegacy_ == drSteer_);
    lastFctlRudderCmd_ = rudderSharesPrimarySteer ? steerRatio : rudderCmd;
    lastFctlRollCmd_ = rollCmd;

    auto writeUnique = [&](XPLMDataRef ref, double value) {
        if (!ref || !XPLMCanWriteDataRef(ref)) return;
        // Do not overwrite the primary steering dataref with a smaller F/CTL visual value.
        // This matters when steer_dataref and fctl_rudder_dataref are the same.
        if (ref == drSteer_) return;
        XPLMSetDataf(ref, static_cast<float>(value));
    };

    writeUnique(drFctlRudder_, rudderCmd);
    writeUnique(drFctlRudderLegacy_, rudderCmd);
    writeUnique(drFctlRoll_, rollCmd);
    writeUnique(drFctlRollLegacy_, rollCmd);
}

void AutoTaxiSystem::releaseFctlControls() {
    lastFctlRudderCmd_ = 0.0;
    lastFctlRollCmd_ = 0.0;

    auto zeroUnique = [&](XPLMDataRef ref) {
        if (!ref || !XPLMCanWriteDataRef(ref)) return;
        if (ref == drSteer_) return;
        XPLMSetDataf(ref, 0.0f);
    };

    zeroUnique(drFctlRudder_);
    zeroUnique(drFctlRudderLegacy_);
    zeroUnique(drFctlRoll_);
    zeroUnique(drFctlRollLegacy_);
}

void AutoTaxiSystem::releaseFctlSecondaryAssist() {
    if (!cfg_.fctlSecondaryAssist || !cfg_.fctlSecondaryRestoreOnRelease ||
        !secondaryFctlBaselineCaptured_ || !secondaryFctlAssistHasCommanded_) {
        return;
    }

    auto writeIfWritable = [](XPLMDataRef ref, double value) {
        if (!ref || !XPLMCanWriteDataRef(ref)) return;
        XPLMSetDataf(ref, static_cast<float>(value));
    };

    writeIfWritable(drFctlFlapRatio_, savedFlapRatio_);
    writeIfWritable(drFctlFlapRequest_, savedFlapRequest_);
    writeIfWritable(drFctlSlatRatio_, savedSlatRatio_);
    secondaryFctlAssistHasCommanded_ = false;
}

void AutoTaxiSystem::writeParkingBrakesReleased() {
    if (drParkingBrake_ && XPLMCanWriteDataRef(drParkingBrake_)) {
        XPLMSetDataf(drParkingBrake_, 0.0f);
    }
    if (drParkingBrakeLegacy_ && XPLMCanWriteDataRef(drParkingBrakeLegacy_)) {
        XPLMSetDataf(drParkingBrakeLegacy_, 0.0f);
    }
}

void AutoTaxiSystem::releaseBrakeCommands() {
    if (!cfg_.commandAssist) return;
    // These commands are optional. Some X-Plane installations/aircraft expose only one of them.
    // XPLMFindCommand returns nullptr if the command is not present, so this is safe.
    if (cmdBrakeRelease_) XPLMCommandOnce(cmdBrakeRelease_);
}

void AutoTaxiSystem::pulseThrottleAssist(double throttle, double groundSpeedMps, double dt) {
    if (!cfg_.commandAssist || !cmdThrottleUp_) return;
    if (throttle < 0.15 || groundSpeedMps > geo::knotsToMps(1.5)) {
        commandAssistTimer_ = 0.0;
        return;
    }

    commandAssistTimer_ += dt;
    if (commandAssistTimer_ >= 0.35) {
        commandAssistTimer_ = 0.0;
        // Some complex aircraft overwrite throttle datarefs every frame. A slow command
        // pulse acts like the pilot nudging the throttles forward and is safer than
        // forcing large dataref values.
        XPLMCommandOnce(cmdThrottleUp_);
    }
}

void AutoTaxiSystem::writeControls(double steerRatio, double throttle, double leftBrake, double rightBrake) {
    steerRatio = clamp(steerRatio, -1.0, 1.0);
    const double throttleLimit = (active_ && stuckSeconds_ >= cfg_.stuckBoostSeconds)
        ? std::max(cfg_.maxThrottle, cfg_.breakawayThrottle)
        : cfg_.maxThrottle;
    throttle = clamp(throttle, 0.0, throttleLimit);
    leftBrake = clamp(leftBrake, 0.0, cfg_.maxBrake);
    rightBrake = clamp(rightBrake, 0.0, cfg_.maxBrake);

    lastSteerCmd_ = steerRatio;
    lastThrottleCmd_ = throttle;
    lastLeftBrakeCmd_ = leftBrake;
    lastRightBrakeCmd_ = rightBrake;

    if (drSteer_ && XPLMCanWriteDataRef(drSteer_)) {
        XPLMSetDataf(drSteer_, static_cast<float>(steerRatio));
    }
    writeFctlCoupling(steerRatio);
    // Keep parking brakes released while active; some aircraft re-apply/hold them after load.
    if (active_) writeParkingBrakesReleased();

    if (drThrottleAll_ && XPLMCanWriteDataRef(drThrottleAll_)) {
        XPLMSetDataf(drThrottleAll_, static_cast<float>(throttle));
    }
    if (drThrottleArray_ && XPLMCanWriteDataRef(drThrottleArray_)) {
        float throttles[8] = {};
        int count = XPLMGetDatavf(drThrottleArray_, throttles, 0, 8);
        if (count <= 0 || count > 8) count = 2;
        for (int i = 0; i < count; ++i) throttles[i] = static_cast<float>(throttle);
        XPLMSetDatavf(drThrottleArray_, throttles, 0, count);
    }

    if (drLeftBrake_ && XPLMCanWriteDataRef(drLeftBrake_)) {
        XPLMSetDataf(drLeftBrake_, static_cast<float>(leftBrake));
    }
    if (drRightBrake_ && XPLMCanWriteDataRef(drRightBrake_)) {
        XPLMSetDataf(drRightBrake_, static_cast<float>(rightBrake));
    }
    if (drLeftBrakeRatio_ && XPLMCanWriteDataRef(drLeftBrakeRatio_)) {
        XPLMSetDataf(drLeftBrakeRatio_, static_cast<float>(leftBrake));
    }
    if (drRightBrakeRatio_ && XPLMCanWriteDataRef(drRightBrakeRatio_)) {
        XPLMSetDataf(drRightBrakeRatio_, static_cast<float>(rightBrake));
    }
}

void AutoTaxiSystem::releaseControls(bool releaseParkingBrake) {
    smoothedSteer_ = 0.0;
    stuckSeconds_ = 0.0;
    lastSteerCmd_ = 0.0;
    lastThrottleCmd_ = 0.0;
    lastLeftBrakeCmd_ = 0.0;
    lastRightBrakeCmd_ = 0.0;
    releaseFctlSecondaryAssist();
    releaseFctlControls();
    secondaryFctlBaselineCaptured_ = false;
    releaseBrakeCommands();
    if (cmdThrottleIdle_) XPLMCommandOnce(cmdThrottleIdle_);
    if (drSteer_ && XPLMCanWriteDataRef(drSteer_)) XPLMSetDataf(drSteer_, 0.0f);
    if (drThrottleAll_ && XPLMCanWriteDataRef(drThrottleAll_)) XPLMSetDataf(drThrottleAll_, 0.0f);
    if (drThrottleArray_ && XPLMCanWriteDataRef(drThrottleArray_)) {
        float throttles[8] = {};
        int count = XPLMGetDatavf(drThrottleArray_, throttles, 0, 8);
        if (count <= 0 || count > 8) count = 2;
        for (int i = 0; i < count; ++i) throttles[i] = 0.0f;
        XPLMSetDatavf(drThrottleArray_, throttles, 0, count);
    }
    if (drLeftBrake_ && XPLMCanWriteDataRef(drLeftBrake_)) XPLMSetDataf(drLeftBrake_, 0.0f);
    if (drRightBrake_ && XPLMCanWriteDataRef(drRightBrake_)) XPLMSetDataf(drRightBrake_, 0.0f);
    if (drLeftBrakeRatio_ && XPLMCanWriteDataRef(drLeftBrakeRatio_)) XPLMSetDataf(drLeftBrakeRatio_, 0.0f);
    if (drRightBrakeRatio_ && XPLMCanWriteDataRef(drRightBrakeRatio_)) XPLMSetDataf(drRightBrakeRatio_, 0.0f);
    if (releaseParkingBrake) writeParkingBrakesReleased();
}

void AutoTaxiSystem::arriveAndHold() {
    if (arrived_) return;

    writeControls(0.0, 0.0, cfg_.maxBrake, cfg_.maxBrake);
    if (cmdThrottleIdle_) XPLMCommandOnce(cmdThrottleIdle_);
    releaseFctlSecondaryAssist();
    if (cfg_.holdParkingBrakeAtDestination && drParkingBrake_ && XPLMCanWriteDataRef(drParkingBrake_)) {
        XPLMSetDataf(drParkingBrake_, 1.0f);
    }

    active_ = false;
    arrived_ = true;
    log("[A350AutoTaxi] Arrived at destination. AutoTaxi OFF, brakes holding.");
}

std::string AutoTaxiSystem::readStringDataRef(XPLMDataRef ref) const {
    if (!ref) return {};
    char buf[512] = {};
    const int n = XPLMGetDatab(ref, buf, 0, static_cast<int>(sizeof(buf) - 1));
    if (n <= 0) return {};
    buf[sizeof(buf) - 1] = '\0';
    return std::string(buf);
}

double AutoTaxiSystem::getDataf(XPLMDataRef ref, double def) const {
    if (!ref) return def;
    return static_cast<double>(XPLMGetDataf(ref));
}

void AutoTaxiSystem::log(const std::string& s) const {
    lastMessage_ = s;
    if (log_) log_(s);
}

} // namespace autotaxi
