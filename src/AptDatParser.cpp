#include "AptDatParser.h"
#include "Config.h"
#include "Geo.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace autotaxi {
namespace {

std::vector<std::string> splitWs(const std::string& line) {
    std::vector<std::string> out;
    std::istringstream iss(line);
    std::string tok;
    while (iss >> tok) out.push_back(tok);
    return out;
}

bool isAirportHeader(int code) {
    return code == 1 || code == 16 || code == 17;
}

bool isCommentOrBlank(const std::string& line) {
    const auto s = trim(line);
    return s.empty() || s[0] == '#';
}

int parseRowCode(const std::vector<std::string>& tok) {
    if (tok.empty()) return -1;
    try {
        return std::stoi(tok[0]);
    } catch (...) {
        return -1;
    }
}

double toDouble(const std::string& s, double def = 0.0) {
    try { return std::stod(s); } catch (...) { return def; }
}

int toInt(const std::string& s, int def = -1) {
    try { return std::stoi(s); } catch (...) { return def; }
}

bool looksLikeLatLon(const std::string& latText, const std::string& lonText) {
    const double lat = toDouble(latText, 999.0);
    const double lon = toDouble(lonText, 999.0);
    return lat >= -90.0 && lat <= 90.0 && lon >= -180.0 && lon <= 180.0;
}

void parseRunway100(const std::vector<std::string>& tok, AirportData& current) {
    // apt.dat 11.00 land runway row. In common 1100 files, the two runway-end
    // names and threshold coordinates are at 8/9/10 and 17/18/19. The guarded
    // fallback scan below keeps third-party exports from breaking the parser.
    Runway rw;
    if (tok.size() >= 20 && looksLikeLatLon(tok[9], tok[10]) && looksLikeLatLon(tok[18], tok[19])) {
        rw.end1 = tok[8];
        rw.lat1 = toDouble(tok[9]);
        rw.lon1 = toDouble(tok[10]);
        rw.end2 = tok[17];
        rw.lat2 = toDouble(tok[18]);
        rw.lon2 = toDouble(tok[19]);
    } else {
        std::vector<RunwayEnd> ends;
        for (size_t i = 1; i + 2 < tok.size(); ++i) {
            if (!looksLikeLatLon(tok[i + 1], tok[i + 2])) continue;
            const std::string name = tok[i];
            const double lat = toDouble(tok[i + 1]);
            const double lon = toDouble(tok[i + 2]);
            if (name.empty()) continue;
            ends.push_back({name, lat, lon});
            i += 2;
            if (ends.size() >= 2) break;
        }
        if (ends.size() >= 2) {
            rw.end1 = ends[0].name;
            rw.lat1 = ends[0].lat;
            rw.lon1 = ends[0].lon;
            rw.end2 = ends[1].name;
            rw.lat2 = ends[1].lat;
            rw.lon2 = ends[1].lon;
        }
    }

    if (!rw.end1.empty() && !rw.end2.empty()) current.runways.push_back(rw);
}

void recomputeCenter(AirportData& ap) {
    double sumLat = 0.0;
    double sumLon = 0.0;
    int count = 0;

    for (const auto& kv : ap.nodes) {
        sumLat += kv.second.lat;
        sumLon += kv.second.lon;
        ++count;
    }
    for (const auto& r : ap.ramps) {
        sumLat += r.lat;
        sumLon += r.lon;
        ++count;
    }
    for (const auto& rw : ap.runways) {
        sumLat += rw.lat1 + rw.lat2;
        sumLon += rw.lon1 + rw.lon2;
        count += 2;
    }

    if (count > 0) {
        ap.centerLat = sumLat / count;
        ap.centerLon = sumLon / count;
    }
}

std::string displayId(const AirportData& ap) {
    if (!ap.icao.empty()) return ap.icao;
    return ap.headerId;
}

bool airportMatches(const AirportData& ap, const AirportSearchOptions& opt) {
    if (!ap.valid()) return false;

    if (!opt.wantedIcao.empty()) {
        return iequals(ap.icao, opt.wantedIcao) || iequals(ap.headerId, opt.wantedIcao);
    }

    return ap.distanceToCurrentM <= opt.maxDistanceM;
}

void finalizeAirport(AirportData& candidate,
                     const AirportSearchOptions& opt,
                     const std::string& path,
                     int sourcePriority,
                     AirportData& best,
                     bool& foundAny,
                     bool& exactIcaoFound,
                     std::vector<std::string>& logLines) {
    if (candidate.headerId.empty()) return;

    if (candidate.icao.empty()) candidate.icao = candidate.headerId;
    candidate.sourceFile = path;
    candidate.sourcePriority = sourcePriority;
    recomputeCenter(candidate);
    candidate.distanceToCurrentM = geo::distanceMeters(
        opt.currentLat, opt.currentLon, candidate.centerLat, candidate.centerLon);

    if (!airportMatches(candidate, opt)) return;

    if (!candidate.valid()) {
        logLines.push_back("[A350AutoTaxi] Airport " + displayId(candidate) +
                           " found but has no usable 1201/1202 taxi network in " + path);
        return;
    }

    if (!opt.wantedIcao.empty()) {
        best = candidate;
        foundAny = true;
        exactIcaoFound = true;
        logLines.push_back("[A350AutoTaxi] Selected airport " + displayId(best) +
                           " from " + path + " nodes=" + std::to_string(best.nodes.size()) +
                           " edges=" + std::to_string(best.edges.size()));
        return;
    }

    if (!foundAny) {
        best = candidate;
        foundAny = true;
        return;
    }

    // GSX-style scenery resolving: distance identifies the current airport; scenery
    // priority decides between duplicate versions of the same/near-identical airport.
    const bool sameId = (!candidate.icao.empty() && !best.icao.empty() && iequals(candidate.icao, best.icao)) ||
                        (!candidate.headerId.empty() && !best.headerId.empty() && iequals(candidate.headerId, best.headerId));
    const bool similarDistance = std::abs(candidate.distanceToCurrentM - best.distanceToCurrentM) <=
                                 opt.sameAirportPriorityToleranceM;

    if ((sameId || similarDistance) && candidate.sourcePriority < best.sourcePriority) {
        best = candidate;
        return;
    }

    if (candidate.distanceToCurrentM + opt.sameAirportPriorityToleranceM < best.distanceToCurrentM) {
        best = candidate;
    }
}

} // namespace

bool AptDatParser::parseBestAirportFromFiles(const std::vector<std::string>& aptDatFiles,
                                             const AirportSearchOptions& options,
                                             AirportData& outAirport,
                                             std::vector<std::string>& logLines) {
    bool found = false;
    AirportData best;

    for (size_t i = 0; i < aptDatFiles.size(); ++i) {
        const auto& file = aptDatFiles[i];
        bool exact = false;
        AirportData inFile;
        if (parseFile(file, static_cast<int>(i), options, inFile, logLines, exact)) {
            if (!options.wantedIcao.empty() && exact) {
                outAirport = inFile;
                return true;
            }

            if (!found) {
                best = inFile;
                found = true;
            } else {
                const bool sameId = (!inFile.icao.empty() && !best.icao.empty() && iequals(inFile.icao, best.icao)) ||
                                    (!inFile.headerId.empty() && !best.headerId.empty() && iequals(inFile.headerId, best.headerId));
                const bool similarDistance = std::abs(inFile.distanceToCurrentM - best.distanceToCurrentM) <=
                                             options.sameAirportPriorityToleranceM;
                if ((sameId || similarDistance) && inFile.sourcePriority < best.sourcePriority) {
                    best = inFile;
                } else if (inFile.distanceToCurrentM + options.sameAirportPriorityToleranceM < best.distanceToCurrentM) {
                    best = inFile;
                }
            }
        }
    }

    if (found) {
        outAirport = best;
        logLines.push_back("[A350AutoTaxi] Selected nearest airport " + displayId(outAirport) +
                           " distance_m=" + std::to_string(static_cast<int>(outAirport.distanceToCurrentM)) +
                           " priority=" + std::to_string(outAirport.sourcePriority) +
                           " source=" + outAirport.sourceFile);
        return true;
    }

    return false;
}

bool AptDatParser::parseFile(const std::string& path,
                             int sourcePriority,
                             const AirportSearchOptions& options,
                             AirportData& outAirport,
                             std::vector<std::string>& logLines,
                             bool& exactIcaoFound) {
    exactIcaoFound = false;

    std::ifstream in(path);
    if (!in) return false;

    AirportData current;
    AirportData best;
    bool haveCurrent = false;
    bool foundAny = false;

    auto closeCurrent = [&]() {
        if (!haveCurrent) return;
        finalizeAirport(current, options, path, sourcePriority, best, foundAny, exactIcaoFound, logLines);
        current = AirportData{};
        haveCurrent = false;
    };

    std::string line;
    while (std::getline(in, line)) {
        if (isCommentOrBlank(line)) continue;

        const auto tok = splitWs(line);
        if (tok.empty()) continue;

        const int code = parseRowCode(tok);

        if (code == 99) {
            closeCurrent();
            break;
        }

        if (isAirportHeader(code)) {
            closeCurrent();
            if (exactIcaoFound) break;

            current = AirportData{};
            haveCurrent = true;
            if (tok.size() >= 5) current.headerId = tok[4];
            if (tok.size() >= 6) current.name = joinTokens(tok, 5);
            continue;
        }

        if (!haveCurrent) continue;

        switch (code) {
            case 100: {
                parseRunway100(tok, current);
                break;
            }

            case 1201: {
                // 1201 lat lon usage node_id node_name
                if (tok.size() < 5) break;
                TaxiNode n;
                n.lat = toDouble(tok[1]);
                n.lon = toDouble(tok[2]);
                n.usage = tok[3];
                n.id = toInt(tok[4]);
                if (tok.size() >= 6) n.name = joinTokens(tok, 5);
                if (n.id >= 0) current.nodes[n.id] = n;
                break;
            }

            case 1202: {
                // 1202 start_id end_id twoway|oneway taxiway|runway identifier
                if (tok.size() < 5) break;
                TaxiEdge e;
                e.from = toInt(tok[1]);
                e.to = toInt(tok[2]);
                e.bidirectional = !iequals(tok[3], "oneway");
                e.runway = iequals(tok[4], "runway");
                if (tok.size() >= 6) e.ident = joinTokens(tok, 5);
                if (e.from >= 0 && e.to >= 0) current.edges.push_back(e);
                break;
            }

            case 1204: {
                // Active zone belongs to immediately preceding 1202 edge.
                if (!current.edges.empty()) {
                    current.edges.back().activeZone = true;
                    for (size_t i = 1; i < tok.size(); ++i) {
                        current.edges.back().activeRunways.push_back(tok[i]);
                    }
                }
                break;
            }

            case 1206: {
                // Ground vehicles only. Same first fields as 1202; ignored for aircraft routing.
                if (tok.size() < 4) break;
                TaxiEdge e;
                e.groundVehicleOnly = true;
                e.from = toInt(tok[1]);
                e.to = toInt(tok[2]);
                e.bidirectional = tok.size() >= 4 ? !iequals(tok[3], "oneway") : true;
                if (tok.size() >= 5) e.ident = joinTokens(tok, 4);
                current.edges.push_back(e);
                break;
            }

            case 1300: {
                // 1300 lat lon heading type aircraft_types name...
                if (tok.size() < 7) break;
                RampStart r;
                r.lat = toDouble(tok[1]);
                r.lon = toDouble(tok[2]);
                r.headingTrue = toDouble(tok[3]);
                r.type = tok[4];
                r.aircraftTypes = tok[5];
                r.name = joinTokens(tok, 6);
                current.ramps.push_back(r);
                break;
            }

            case 1301: {
                // 1301 width operation airline...
                if (current.ramps.empty()) break;
                if (tok.size() >= 2) current.ramps.back().widthCode = tok[1];
                if (tok.size() >= 3) current.ramps.back().operationType = tok[2];
                for (size_t i = 3; i < tok.size(); ++i) current.ramps.back().airlines.push_back(tok[i]);
                break;
            }

            case 1302: {
                // X-Plane 11/12 metadata commonly uses icao_code or icao_id depending on exporter/spec wording.
                if (tok.size() >= 3) {
                    const auto key = toUpper(tok[1]);
                    if (key == "ICAO_CODE" || key == "ICAO_ID" || key == "ICAO") {
                        current.icao = tok[2];
                    }
                }
                break;
            }

            default:
                break;
        }

        // For exact ICAO search, do not exit immediately; we need the full airport block.
        // closeCurrent() will return the completed block once the next header/99 appears.
    }

    closeCurrent();

    if (foundAny) {
        outAirport = best;
        return true;
    }

    return false;
}

} // namespace autotaxi
