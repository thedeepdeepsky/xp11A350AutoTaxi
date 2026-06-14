#include "Config.h"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>

namespace autotaxi {

std::string trim(std::string s) {
    auto notSpace = [](unsigned char c) { return !std::isspace(c); };
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), notSpace));
    s.erase(std::find_if(s.rbegin(), s.rend(), notSpace).base(), s.end());
    return s;
}

std::string toUpper(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) {
        return static_cast<char>(std::toupper(c));
    });
    return s;
}

bool iequals(const std::string& a, const std::string& b) {
    return toUpper(a) == toUpper(b);
}

bool icontains(const std::string& haystack, const std::string& needle) {
    if (needle.empty()) return true;
    return toUpper(haystack).find(toUpper(needle)) != std::string::npos;
}

std::vector<std::string> splitCsv(const std::string& text) {
    std::vector<std::string> out;
    std::stringstream ss(text);
    std::string item;
    while (std::getline(ss, item, ',')) {
        item = trim(item);
        if (!item.empty()) out.push_back(item);
    }
    return out;
}

std::string joinTokens(const std::vector<std::string>& tokens, size_t first) {
    if (first >= tokens.size()) return {};
    std::string out;
    for (size_t i = first; i < tokens.size(); ++i) {
        if (!out.empty()) out += ' ';
        out += tokens[i];
    }
    return out;
}

bool IniConfig::load(const std::string& path, std::string& error) {
    values_.clear();

    std::ifstream in(path);
    if (!in) {
        error = "Cannot open config: " + path;
        return false;
    }

    std::string line;
    int lineNo = 0;
    while (std::getline(in, line)) {
        ++lineNo;
        line = trim(line);
        if (line.empty()) continue;
        if (line[0] == '#') continue;

        const auto eq = line.find('=');
        if (eq == std::string::npos) {
            error = "Bad ini line " + std::to_string(lineNo) + ": " + line;
            return false;
        }

        auto key = toUpper(trim(line.substr(0, eq)));
        auto val = trim(line.substr(eq + 1));

        // Strip inline comments only when preceded by whitespace: "value # comment".
        for (size_t i = 0; i + 1 < val.size(); ++i) {
            if (std::isspace(static_cast<unsigned char>(val[i])) && val[i + 1] == '#') {
                val = trim(val.substr(0, i));
                break;
            }
        }

        values_[key] = val;
    }

    return true;
}

std::string IniConfig::getString(const std::string& key, const std::string& def) const {
    auto it = values_.find(toUpper(key));
    if (it == values_.end()) return def;
    return it->second;
}

double IniConfig::getDouble(const std::string& key, double def) const {
    const auto s = getString(key, "");
    if (s.empty()) return def;
    try {
        return std::stod(s);
    } catch (...) {
        return def;
    }
}

int IniConfig::getInt(const std::string& key, int def) const {
    const auto s = getString(key, "");
    if (s.empty()) return def;
    try {
        return std::stoi(s);
    } catch (...) {
        return def;
    }
}

bool IniConfig::getBool(const std::string& key, bool def) const {
    const auto s = toUpper(getString(key, ""));
    if (s.empty()) return def;
    if (s == "1" || s == "TRUE" || s == "YES" || s == "ON") return true;
    if (s == "0" || s == "FALSE" || s == "NO" || s == "OFF") return false;
    return def;
}

std::vector<std::string> IniConfig::getCsv(const std::string& key, const std::vector<std::string>& def) const {
    const auto s = getString(key, "");
    if (s.empty()) return def;
    return splitCsv(s);
}

AutoTaxiConfig loadAutoTaxiConfig(const std::string& iniPath, std::string& error) {
    AutoTaxiConfig cfg;
    IniConfig ini;
    if (!ini.load(iniPath, error)) {
        // Return defaults if not found; caller will log the warning.
        return cfg;
    }

    cfg.aptDatPath = ini.getString("apt_dat_path", cfg.aptDatPath);
    cfg.icao = toUpper(ini.getString("icao", cfg.icao));

    cfg.destinationNode = ini.getInt("destination_node", cfg.destinationNode);
    cfg.destinationRamp = ini.getString("destination_ramp", cfg.destinationRamp);
    cfg.destinationLat = ini.getDouble("destination_lat", cfg.destinationLat);
    cfg.destinationLon = ini.getDouble("destination_lon", cfg.destinationLon);

    cfg.allowAnyAircraft = ini.getBool("allow_any_aircraft", cfg.allowAnyAircraft);
    cfg.allowedAircraftIcao = ini.getCsv("allowed_aircraft_icao", cfg.allowedAircraftIcao);

    cfg.airportSearchRadiusNm = ini.getDouble("airport_search_radius_nm", cfg.airportSearchRadiusNm);

    cfg.sameAirportPriorityToleranceM = ini.getDouble("same_airport_priority_tolerance_m", cfg.sameAirportPriorityToleranceM);

    cfg.taxiSpeedKts = ini.getDouble("taxi_speed_kts", cfg.taxiSpeedKts);
    cfg.slowSpeedKts = ini.getDouble("slow_speed_kts", cfg.slowSpeedKts);
    cfg.finalSpeedKts = ini.getDouble("final_speed_kts", cfg.finalSpeedKts);
    cfg.sharpTurnSpeedKts = ini.getDouble("sharp_turn_speed_kts", cfg.sharpTurnSpeedKts);
    cfg.waypointRadiusM = ini.getDouble("waypoint_radius_m", cfg.waypointRadiusM);
    cfg.finalStopRadiusM = ini.getDouble("final_stop_radius_m", cfg.finalStopRadiusM);
    cfg.terminalArrivalGuard = ini.getBool("terminal_arrival_guard", cfg.terminalArrivalGuard);
    cfg.terminalArrivalRadiusM = ini.getDouble("terminal_arrival_radius_m", cfg.terminalArrivalRadiusM);
    cfg.terminalOvershootRadiusM = ini.getDouble("terminal_overshoot_radius_m", cfg.terminalOvershootRadiusM);
    cfg.terminalOvershootSeconds = ini.getDouble("terminal_overshoot_seconds", cfg.terminalOvershootSeconds);
    cfg.terminalNoTightTurnDistanceM = ini.getDouble("terminal_no_tight_turn_distance_m", cfg.terminalNoTightTurnDistanceM);
    cfg.terminalSlowdownDistanceM = ini.getDouble("terminal_slowdown_distance_m", cfg.terminalSlowdownDistanceM);
    cfg.terminalSpeedKts = ini.getDouble("terminal_speed_kts", cfg.terminalSpeedKts);
    cfg.terminalMinFinalLegM = ini.getDouble("terminal_min_final_leg_m", cfg.terminalMinFinalLegM);
    cfg.terminalSharpTurnDeg = ini.getDouble("terminal_sharp_turn_deg", cfg.terminalSharpTurnDeg);
    cfg.terminalSharpTurnShortLegM = ini.getDouble("terminal_sharp_turn_short_leg_m", cfg.terminalSharpTurnShortLegM);
    cfg.terminalXteStopM = ini.getDouble("terminal_xte_stop_m", cfg.terminalXteStopM);
    cfg.lookaheadDistanceM = ini.getDouble("lookahead_distance_m", cfg.lookaheadDistanceM);
    cfg.lookaheadMinDistanceM = ini.getDouble("lookahead_min_distance_m", cfg.lookaheadMinDistanceM);
    cfg.lookaheadMaxDistanceM = ini.getDouble("lookahead_max_distance_m", cfg.lookaheadMaxDistanceM);
    cfg.lookaheadSpeedGainMPerKt = ini.getDouble("lookahead_speed_gain_m_per_kt", cfg.lookaheadSpeedGainMPerKt);
    cfg.turnSlowdownThresholdDeg = ini.getDouble("turn_slowdown_threshold_deg", cfg.turnSlowdownThresholdDeg);

    cfg.pidRouteTracking = ini.getBool("pid_route_tracking", cfg.pidRouteTracking);
    cfg.pidHeadingKp = ini.getDouble("pid_heading_kp", cfg.pidHeadingKp);
    cfg.pidHeadingKi = ini.getDouble("pid_heading_ki", cfg.pidHeadingKi);
    cfg.pidHeadingKd = ini.getDouble("pid_heading_kd", cfg.pidHeadingKd);
    cfg.pidIntegralLimitDegSec = ini.getDouble("pid_integral_limit_deg_sec", cfg.pidIntegralLimitDegSec);
    cfg.pidDerivativeFilterSec = ini.getDouble("pid_derivative_filter_sec", cfg.pidDerivativeFilterSec);
    cfg.pidCrossTrackGainDegPerM = ini.getDouble("pid_cross_track_gain_deg_per_m", cfg.pidCrossTrackGainDegPerM);
    cfg.pidCrossTrackMaxDeg = ini.getDouble("pid_cross_track_max_deg", cfg.pidCrossTrackMaxDeg);
    cfg.maxTrackCaptureAngleDeg = ini.getDouble("max_track_capture_angle_deg", cfg.maxTrackCaptureAngleDeg);
    cfg.fastRouteCapture = ini.getBool("fast_route_capture", cfg.fastRouteCapture);
    cfg.fastCaptureStartSeconds = ini.getDouble("fast_capture_start_seconds", cfg.fastCaptureStartSeconds);
    cfg.fastCaptureThresholdM = ini.getDouble("fast_capture_threshold_m", cfg.fastCaptureThresholdM);
    cfg.fastCaptureHardThresholdM = ini.getDouble("fast_capture_hard_threshold_m", cfg.fastCaptureHardThresholdM);
    cfg.fastCaptureGainDegPerM = ini.getDouble("fast_capture_gain_deg_per_m", cfg.fastCaptureGainDegPerM);
    cfg.fastCaptureMaxBiasDeg = ini.getDouble("fast_capture_max_bias_deg", cfg.fastCaptureMaxBiasDeg);
    cfg.fastCaptureKpBoost = ini.getDouble("fast_capture_kp_boost", cfg.fastCaptureKpBoost);
    cfg.fastCaptureKdBoost = ini.getDouble("fast_capture_kd_boost", cfg.fastCaptureKdBoost);
    cfg.fastCaptureSteerFullDeflectionDeg = ini.getDouble("fast_capture_steer_full_deflection_deg", cfg.fastCaptureSteerFullDeflectionDeg);
    cfg.fastCaptureSteerSmoothingPerSec = ini.getDouble("fast_capture_steer_smoothing_per_sec", cfg.fastCaptureSteerSmoothingPerSec);
    cfg.nearRouteDampingM = ini.getDouble("near_route_damping_m", cfg.nearRouteDampingM);
    cfg.predictiveXteLead = ini.getBool("predictive_xte_lead", cfg.predictiveXteLead);
    cfg.predictiveXteLeadTimeSec = ini.getDouble("predictive_xte_lead_time_sec", cfg.predictiveXteLeadTimeSec);
    cfg.predictiveXteMaxLeadM = ini.getDouble("predictive_xte_max_lead_m", cfg.predictiveXteMaxLeadM);
    cfg.predictiveXteRateFilterSec = ini.getDouble("predictive_xte_rate_filter_sec", cfg.predictiveXteRateFilterSec);
    cfg.predictiveXteMinActiveM = ini.getDouble("predictive_xte_min_active_m", cfg.predictiveXteMinActiveM);
    cfg.microAnticipateReturn = ini.getBool("micro_anticipate_return", cfg.microAnticipateReturn);
    cfg.microAnticipateBandM = ini.getDouble("micro_anticipate_band_m", cfg.microAnticipateBandM);
    cfg.microAnticipateLeadSec = ini.getDouble("micro_anticipate_lead_sec", cfg.microAnticipateLeadSec);
    cfg.microAnticipateGeomRateWeight = ini.getDouble("micro_anticipate_geom_rate_weight", cfg.microAnticipateGeomRateWeight);
    cfg.microAnticipateMinSpeedKts = ini.getDouble("micro_anticipate_min_speed_kts", cfg.microAnticipateMinSpeedKts);
    cfg.microAnticipateMinHeadingDeg = ini.getDouble("micro_anticipate_min_heading_deg", cfg.microAnticipateMinHeadingDeg);
    cfg.microAnticipateMaxLeadM = ini.getDouble("micro_anticipate_max_lead_m", cfg.microAnticipateMaxLeadM);
    cfg.microAnticipateBiasBoost = ini.getDouble("micro_anticipate_bias_boost", cfg.microAnticipateBiasBoost);
    cfg.microAnticipateDeadbandDeg = ini.getDouble("micro_anticipate_deadband_deg", cfg.microAnticipateDeadbandDeg);
    cfg.microAnticipateDirectSteer = ini.getBool("micro_anticipate_direct_steer", cfg.microAnticipateDirectSteer);
    cfg.microAnticipateDirectBandM = ini.getDouble("micro_anticipate_direct_band_m", cfg.microAnticipateDirectBandM);
    cfg.microAnticipateCounterSteerRatio = ini.getDouble("micro_anticipate_counter_steer_ratio", cfg.microAnticipateCounterSteerRatio);
    cfg.microAnticipateCounterMinRatio = ini.getDouble("micro_anticipate_counter_min_ratio", cfg.microAnticipateCounterMinRatio);
    cfg.microAnticipateTowardSteerThreshold = ini.getDouble("micro_anticipate_toward_steer_threshold", cfg.microAnticipateTowardSteerThreshold);
    cfg.microAnticipateDirectBlend = ini.getDouble("micro_anticipate_direct_blend", cfg.microAnticipateDirectBlend);
    cfg.microAnticipatePidReturnDeg = ini.getDouble("micro_anticipate_pid_return_deg", cfg.microAnticipatePidReturnDeg);
    cfg.turnAnticipation = ini.getBool("turn_anticipation", cfg.turnAnticipation);
    cfg.turnAnticipationDistanceM = ini.getDouble("turn_anticipation_distance_m", cfg.turnAnticipationDistanceM);
    cfg.turnAnticipationMinAngleDeg = ini.getDouble("turn_anticipation_min_angle_deg", cfg.turnAnticipationMinAngleDeg);
    cfg.turnAnticipationExtraLookaheadM = ini.getDouble("turn_anticipation_extra_lookahead_m", cfg.turnAnticipationExtraLookaheadM);
    cfg.turnAnticipationSlowdownKts = ini.getDouble("turn_anticipation_slowdown_kts", cfg.turnAnticipationSlowdownKts);
    cfg.tightTurnMode = ini.getBool("tight_turn_mode", cfg.tightTurnMode);
    cfg.tightTurnAngleDeg = ini.getDouble("tight_turn_angle_deg", cfg.tightTurnAngleDeg);
    cfg.tightTurnTriggerDistanceM = ini.getDouble("tight_turn_trigger_distance_m", cfg.tightTurnTriggerDistanceM);
    cfg.tightTurnLookaheadAfterApexM = ini.getDouble("tight_turn_lookahead_after_apex_m", cfg.tightTurnLookaheadAfterApexM);
    cfg.tightTurnMinLookaheadM = ini.getDouble("tight_turn_min_lookahead_m", cfg.tightTurnMinLookaheadM);
    cfg.tightTurnSpeedKts = ini.getDouble("tight_turn_speed_kts", cfg.tightTurnSpeedKts);
    cfg.tightTurnSteerFullDeflectionDeg = ini.getDouble("tight_turn_steer_full_deflection_deg", cfg.tightTurnSteerFullDeflectionDeg);
    cfg.tightTurnSteerSmoothingPerSec = ini.getDouble("tight_turn_steer_smoothing_per_sec", cfg.tightTurnSteerSmoothingPerSec);
    cfg.tightTurnKpBoost = ini.getDouble("tight_turn_kp_boost", cfg.tightTurnKpBoost);
    cfg.tightTurnKdBoost = ini.getDouble("tight_turn_kd_boost", cfg.tightTurnKdBoost);
    cfg.tightTurnDifferentialBrakeMax = ini.getDouble("tight_turn_differential_brake_max", cfg.tightTurnDifferentialBrakeMax);
    cfg.tightTurnDifferentialBrakeMinKts = ini.getDouble("tight_turn_differential_brake_min_kts", cfg.tightTurnDifferentialBrakeMinKts);
    cfg.steerFastResponse = ini.getBool("steer_fast_response", cfg.steerFastResponse);
    cfg.steerFastResponseErrorDeg = ini.getDouble("steer_fast_response_error_deg", cfg.steerFastResponseErrorDeg);
    cfg.steerFastResponseXteM = ini.getDouble("steer_fast_response_xte_m", cfg.steerFastResponseXteM);
    cfg.steerFastResponseRatePerSec = ini.getDouble("steer_fast_response_rate_per_sec", cfg.steerFastResponseRatePerSec);
    cfg.tightTurnSnapSteer = ini.getBool("tight_turn_snap_steer", cfg.tightTurnSnapSteer);
    cfg.tightTurnSnapBlend = ini.getDouble("tight_turn_snap_blend", cfg.tightTurnSnapBlend);
    cfg.tightTurnSnapMinTargetRatio = ini.getDouble("tight_turn_snap_min_target_ratio", cfg.tightTurnSnapMinTargetRatio);
    cfg.tightTurnSnapToRatio = ini.getDouble("tight_turn_snap_to_ratio", cfg.tightTurnSnapToRatio);
    cfg.tightTurnOnePassMode = ini.getBool("tight_turn_one_pass_mode", cfg.tightTurnOnePassMode);
    cfg.tightTurnOnePassAngleDeg = ini.getDouble("tight_turn_one_pass_angle_deg", cfg.tightTurnOnePassAngleDeg);
    cfg.tightTurnHardSnapToRatio = ini.getDouble("tight_turn_hard_snap_to_ratio", cfg.tightTurnHardSnapToRatio);
    cfg.tightTurnHardSnapMinTargetRatio = ini.getDouble("tight_turn_hard_snap_min_target_ratio", cfg.tightTurnHardSnapMinTargetRatio);
    cfg.tightTurnRolloutHeadingDeg = ini.getDouble("tight_turn_rollout_heading_deg", cfg.tightTurnRolloutHeadingDeg);
    cfg.tightTurnRolloutGain = ini.getDouble("tight_turn_rollout_gain", cfg.tightTurnRolloutGain);
    cfg.tightTurnRolloutMinPidDeg = ini.getDouble("tight_turn_rollout_min_pid_deg", cfg.tightTurnRolloutMinPidDeg);
    cfg.routeSegmentAdvanceAlong = ini.getDouble("route_segment_advance_along", cfg.routeSegmentAdvanceAlong);
    cfg.routeSegmentScanAhead = ini.getDouble("route_segment_scan_ahead", cfg.routeSegmentScanAhead);

    cfg.maxThrottle = ini.getDouble("max_throttle", cfg.maxThrottle);
    cfg.creepThrottle = ini.getDouble("creep_throttle", cfg.creepThrottle);
    cfg.breakawayThrottle = ini.getDouble("breakaway_throttle", cfg.breakawayThrottle);
    cfg.stuckBoostSeconds = ini.getDouble("stuck_boost_seconds", cfg.stuckBoostSeconds);
    cfg.dynamicThrottleControl = ini.getBool("dynamic_throttle_control", cfg.dynamicThrottleControl);
    cfg.throttleIdle = ini.getDouble("throttle_idle", cfg.throttleIdle);
    cfg.throttleTrim = ini.getDouble("throttle_trim", cfg.throttleTrim);
    cfg.throttleMinMoving = ini.getDouble("throttle_min_moving", cfg.throttleMinMoving);
    cfg.throttleSpeedKp = ini.getDouble("throttle_speed_kp", cfg.throttleSpeedKp);
    cfg.throttleCaptureReduction = ini.getDouble("throttle_capture_reduction", cfg.throttleCaptureReduction);
    cfg.throttleTurnReduction = ini.getDouble("throttle_turn_reduction", cfg.throttleTurnReduction);
    cfg.throttleTerminalReduction = ini.getDouble("throttle_terminal_reduction", cfg.throttleTerminalReduction);
    cfg.throttleIncreaseRatePerSec = ini.getDouble("throttle_increase_rate_per_sec", cfg.throttleIncreaseRatePerSec);
    cfg.throttleDecreaseRatePerSec = ini.getDouble("throttle_decrease_rate_per_sec", cfg.throttleDecreaseRatePerSec);
    cfg.throttleOverspeedBrakeDeadbandKts = ini.getDouble("throttle_overspeed_brake_deadband_kts", cfg.throttleOverspeedBrakeDeadbandKts);
    cfg.commandAssist = ini.getBool("command_assist", cfg.commandAssist);
    cfg.maxBrake = ini.getDouble("max_brake", cfg.maxBrake);
    cfg.brakeGain = ini.getDouble("brake_gain", cfg.brakeGain);
    cfg.steerFullDeflectionDeg = ini.getDouble("steer_full_deflection_deg", cfg.steerFullDeflectionDeg);
    cfg.steerDeadbandDeg = ini.getDouble("steer_deadband_deg", cfg.steerDeadbandDeg);
    cfg.steerSmoothingPerSec = ini.getDouble("steer_smoothing_per_sec", cfg.steerSmoothingPerSec);
    cfg.maxSteerRatio = ini.getDouble("max_steer_ratio", cfg.maxSteerRatio);
    cfg.lowSpeedMaxSteerRatio = ini.getDouble("low_speed_max_steer_ratio", cfg.lowSpeedMaxSteerRatio);
    cfg.highSpeedMaxSteerRatio = ini.getDouble("high_speed_max_steer_ratio", cfg.highSpeedMaxSteerRatio);
    cfg.highSpeedSteerKts = ini.getDouble("high_speed_steer_kts", cfg.highSpeedSteerKts);
    cfg.steerCurveExponent = ini.getDouble("steer_curve_exponent", cfg.steerCurveExponent);
    cfg.differentialBrakeThresholdDeg = ini.getDouble("differential_brake_threshold_deg", cfg.differentialBrakeThresholdDeg);
    cfg.differentialBrakeMax = ini.getDouble("differential_brake_max", cfg.differentialBrakeMax);

    cfg.fctlTurnCoupling = ini.getBool("fctl_turn_coupling", cfg.fctlTurnCoupling);
    cfg.fctlRudderGain = ini.getDouble("fctl_rudder_gain", cfg.fctlRudderGain);
    cfg.fctlRollGain = ini.getDouble("fctl_roll_gain", cfg.fctlRollGain);
    cfg.fctlMaxRatio = ini.getDouble("fctl_max_ratio", cfg.fctlMaxRatio);
    cfg.fctlSecondaryAssist = ini.getBool("fctl_secondary_assist", cfg.fctlSecondaryAssist);
    cfg.fctlSecondaryMinTightBlend = ini.getDouble("fctl_secondary_min_tight_blend", cfg.fctlSecondaryMinTightBlend);
    cfg.fctlSecondaryFlapRatio = ini.getDouble("fctl_secondary_flap_ratio", cfg.fctlSecondaryFlapRatio);
    cfg.fctlSecondaryRestoreOnRelease = ini.getBool("fctl_secondary_restore_on_release", cfg.fctlSecondaryRestoreOnRelease);

    cfg.avoidRunwayEdges = ini.getBool("avoid_runway_edges", cfg.avoidRunwayEdges);
    cfg.runwayPenaltyM = ini.getDouble("runway_penalty_m", cfg.runwayPenaltyM);
    cfg.activeZonePenaltyM = ini.getDouble("active_zone_penalty_m", cfg.activeZonePenaltyM);
    cfg.turnPenaltyM = ini.getDouble("turn_penalty_m", cfg.turnPenaltyM);
    cfg.autoReplanIfOffRoute = ini.getBool("auto_replan_if_off_route", cfg.autoReplanIfOffRoute);
    cfg.offRouteReplanDistanceM = ini.getDouble("off_route_replan_distance_m", cfg.offRouteReplanDistanceM);
    cfg.maxStartNodeDistanceM = ini.getDouble("max_start_node_distance_m", cfg.maxStartNodeDistanceM);
    cfg.maxStartSpeedKts = ini.getDouble("max_start_speed_kts", cfg.maxStartSpeedKts);
    cfg.gatePositionRadiusM = ini.getDouble("gate_position_radius_m", cfg.gatePositionRadiusM);
    cfg.taxiwayPositionRadiusM = ini.getDouble("taxiway_position_radius_m", cfg.taxiwayPositionRadiusM);
    cfg.routeSummaryMaxNames = ini.getInt("route_summary_max_names", cfg.routeSummaryMaxNames);
    cfg.holdParkingBrakeAtDestination = ini.getBool("hold_parking_brake_at_destination", cfg.holdParkingBrakeAtDestination);

    cfg.steerDataref = ini.getString("steer_dataref", cfg.steerDataref);
    cfg.fctlRudderDataref = ini.getString("fctl_rudder_dataref", cfg.fctlRudderDataref);
    cfg.fctlRollDataref = ini.getString("fctl_roll_dataref", cfg.fctlRollDataref);
    cfg.fctlRudderLegacyDataref = ini.getString("fctl_rudder_legacy_dataref", cfg.fctlRudderLegacyDataref);
    cfg.fctlRollLegacyDataref = ini.getString("fctl_roll_legacy_dataref", cfg.fctlRollLegacyDataref);
    cfg.fctlFlapRatioDataref = ini.getString("fctl_flap_ratio_dataref", cfg.fctlFlapRatioDataref);
    cfg.fctlFlapRequestDataref = ini.getString("fctl_flap_request_dataref", cfg.fctlFlapRequestDataref);
    cfg.fctlSlatRatioDataref = ini.getString("fctl_slat_ratio_dataref", cfg.fctlSlatRatioDataref);
    cfg.throttleAllDataref = ini.getString("throttle_all_dataref", cfg.throttleAllDataref);
    cfg.throttleArrayDataref = ini.getString("throttle_array_dataref", cfg.throttleArrayDataref);
    cfg.leftBrakeDataref = ini.getString("left_brake_dataref", cfg.leftBrakeDataref);
    cfg.rightBrakeDataref = ini.getString("right_brake_dataref", cfg.rightBrakeDataref);
    cfg.parkingBrakeDataref = ini.getString("parking_brake_dataref", cfg.parkingBrakeDataref);

    return cfg;
}

} // namespace autotaxi
