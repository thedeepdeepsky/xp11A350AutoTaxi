#pragma once
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

namespace autotaxi {

class IniConfig {
public:
    bool load(const std::string& path, std::string& error);
    std::string getString(const std::string& key, const std::string& def = "") const;
    double getDouble(const std::string& key, double def) const;
    int getInt(const std::string& key, int def) const;
    bool getBool(const std::string& key, bool def) const;
    std::vector<std::string> getCsv(const std::string& key, const std::vector<std::string>& def = {}) const;

private:
    std::unordered_map<std::string, std::string> values_;
};

struct AutoTaxiConfig {
    std::string aptDatPath;
    std::string icao;

    int destinationNode = -1;
    std::string destinationRamp;
    double destinationLat = std::numeric_limits<double>::quiet_NaN();
    double destinationLon = std::numeric_limits<double>::quiet_NaN();

    bool allowAnyAircraft = false;
    std::vector<std::string> allowedAircraftIcao = {"A359", "A35K", "A350"};

    // Airport/scenery resolving.
    double airportSearchRadiusNm = 8.0;
    double sameAirportPriorityToleranceM = 700.0;

    // Taxi motion model.
    double taxiSpeedKts = 25.0;
    double slowSpeedKts = 6.0;
    double finalSpeedKts = 3.0;
    double sharpTurnSpeedKts = 4.0;
    double waypointRadiusM = 18.0;
    double finalStopRadiusM = 10.0;

    // Terminal/arrival guard. Some apt.dat runway hold-short endpoints are represented
    // by a very short final leg after a sharp turn. For a heavy A350, forcing exact
    // node/heading capture there can make the aircraft circle around the destination.
    // These parameters merge/prune tiny terminal hooks and stop once inside an
    // operational arrival bubble instead of chasing a 1-2 node micro-turn.
    bool terminalArrivalGuard = true;
    double terminalArrivalRadiusM = 34.0;
    double terminalOvershootRadiusM = 55.0;
    double terminalOvershootSeconds = 0.8;
    double terminalNoTightTurnDistanceM = 85.0;
    double terminalSlowdownDistanceM = 130.0;
    double terminalSpeedKts = 2.0;
    double terminalMinFinalLegM = 22.0;
    double terminalSharpTurnDeg = 95.0;
    double terminalSharpTurnShortLegM = 55.0;
    double terminalXteStopM = 16.0;

    double lookaheadDistanceM = 80.0;
    double lookaheadMinDistanceM = 45.0;
    double lookaheadMaxDistanceM = 120.0;
    double lookaheadSpeedGainMPerKt = 3.0;
    double turnSlowdownThresholdDeg = 35.0;

    // AP-like path tracking. The aircraft follows the fixed planned taxi polyline;
    // it does not chase the closest node directly, which avoids +/- steering hunting near nodes.
    bool pidRouteTracking = true;
    double pidHeadingKp = 0.48;
    double pidHeadingKi = 0.000;
    double pidHeadingKd = 0.36;
    double pidIntegralLimitDegSec = 45.0;
    double pidDerivativeFilterSec = 0.62;
    double pidCrossTrackGainDegPerM = 0.10;
    double pidCrossTrackMaxDeg = 10.0;
    double maxTrackCaptureAngleDeg = 58.0;

    // Fast route-capture mode for startup/off-track cases.
    // When the aircraft is several metres away from the planned polyline,
    // the controller temporarily behaves like an AP localizer-capture mode:
    // stronger cross-track bias, faster steering rate, and more authority.
    // Once the aircraft is back near the line, it fades back to the smoother PID.
    bool fastRouteCapture = true;
    double fastCaptureStartSeconds = 28.0;
    double fastCaptureThresholdM = 5.5;
    double fastCaptureHardThresholdM = 16.0;
    double fastCaptureGainDegPerM = 0.52;
    double fastCaptureMaxBiasDeg = 28.0;
    double fastCaptureKpBoost = 1.05;
    double fastCaptureKdBoost = 1.65;
    double fastCaptureSteerFullDeflectionDeg = 45.0;
    double fastCaptureSteerSmoothingPerSec = 2.20;
    double nearRouteDampingM = 5.0;

    // Predictive cross-track lead. This is the taxi equivalent of an AP localizer
    // lead/damping term: if XTE is moving toward zero, command the next correction
    // before the displayed XTE actually changes sign. This prevents the late
    // "cross zero first, steer second" S-turn behavior.
    bool predictiveXteLead = true;
    double predictiveXteLeadTimeSec = 3.2;
    double predictiveXteMaxLeadM = 22.0;
    double predictiveXteRateFilterSec = 0.55;
    double predictiveXteMinActiveM = 0.8;

    // Micro-course anticipatory return steering. The measured XTE can change very slowly
    // during small corrections, so waiting for XTE rate or sign change makes the aircraft
    // cross the route first and then S-turn back. This predicts near-future XTE from the
    // aircraft heading relative to the path, so it starts unwinding/counter-steering before
    // XTE reaches zero even in tiny corrections.
    bool microAnticipateReturn = true;
    double microAnticipateBandM = 8.0;
    double microAnticipateLeadSec = 4.0;
    double microAnticipateGeomRateWeight = 0.85;
    double microAnticipateMinSpeedKts = 1.0;
    double microAnticipateMinHeadingDeg = 0.6;
    double microAnticipateMaxLeadM = 10.0;
    double microAnticipateBiasBoost = 1.25;
    double microAnticipateDeadbandDeg = 0.25;

    // Direct pre-zero steering return. The XTE bias alone can be too weak because the
    // look-ahead target may still sit on the current side of the centerline. When the
    // aircraft is close to the route and the tiller is still steering toward the line,
    // blend the requested steer toward the opposite side before XTE reaches zero.
    bool microAnticipateDirectSteer = true;
    double microAnticipateDirectBandM = 7.0;
    double microAnticipateCounterSteerRatio = 0.22;
    double microAnticipateCounterMinRatio = 0.06;
    double microAnticipateTowardSteerThreshold = 0.015;
    double microAnticipateDirectBlend = 0.80;
    double microAnticipatePidReturnDeg = 9.0;

    // Track-course controller. This directly follows the current taxiway leg heading
    // plus a Stanley/L1-style cross-track intercept angle. It reacts as soon as XTE
    // appears, not only after the look-ahead point has moved far enough. When the nose
    // is already cutting back toward the centerline it reduces/reverses the effective
    // XTE before zero, so the tiller begins to unwind before the aircraft crosses the line.
    bool trackCourseControl = true;
    double trackCourseBlend = 0.92;
    double trackCourseLookaheadM = 16.0;
    double trackCourseSpeedGainMPerKt = 0.55;
    double trackCourseGain = 1.18;
    double trackCourseMaxInterceptDeg = 26.0;
    double trackCoursePrezeroBandM = 11.0;
    double trackCoursePrezeroLeadSec = 5.6;
    double trackCoursePrezeroPower = 2.30;
    double trackCoursePrezeroMinHeadingDeg = 0.10;
    double trackCoursePrezeroCounterM = 4.8;
    double trackCourseTightTurnFade = 0.85;

    // XTE corridor / sway guard. The controller should not wait until a large
    // deviation exists, but it should also avoid over-commanding yaw and S-turning
    // across the centerline. This mode first captures into a +/- corridor, then
    // fades to fine tracking with stronger damping and smaller intercept angles.
    bool xteCorridorControl = true;
    double xteCorridorM = 5.0;
    double xteFineTuneM = 1.2;
    double xteCorridorOuterGain = 0.55;
    double xteCorridorInnerGain = 0.42;
    double xteCorridorMaxInterceptDeg = 13.0;
    double xteCorridorOuterMaxInterceptDeg = 22.0;
    double xteCorridorLeadSec = 4.0;
    double xteCorridorPrezeroReturnDeg = 7.0;
    double xteCorridorKpScale = 0.52;
    double xteCorridorKdBoost = 1.45;
    double xteCorridorCaptureFade = 0.18;
    double swayDampingBandM = 6.0;
    double swayDampingCommandScale = 0.55;
    double swayDampingSteerRatePerSec = 2.0;

    // Early hard-turn takeover. Track-course control is good on straight legs, but it can
    // keep commanding the current leg for too long before a 70-100 degree taxiway corner.
    // This mode fades track-course out earlier, blends toward the outbound leg, and allows
    // a snap-to-tiller command before the nose reaches the intersection.
    bool earlyTurnTakeover = true;
    double earlyTurnTakeoverAngleDeg = 50.0;
    double earlyTurnTakeoverDistanceM = 135.0;
    double earlyTurnFullDistanceM = 55.0;
    double earlyTurnMinBlend = 0.18;
    double earlyTurnHeadingBlend = 0.60;
    double earlyTurnTrackCourseFade = 0.55;
    double earlyTurnSnapToRatio = 0.45;
    double earlyTurnSnapMinTargetRatio = 0.12;

    // Hard snap for large upcoming turns. When a 70-100 degree corner is close,
    // do not ramp the steering command. Force full tiller immediately and let the
    // one-pass rollout logic release it only near the outbound heading.
    bool tightTurnForceFullSteer = true;
    double tightTurnForceFullSteerAngleDeg = 65.0;
    // Full tiller is intentionally delayed until close to the corner apex.
    // FAA/ICAO taxiway design assumes cockpit-over-centreline turning and fillet/radius
    // geometry, not a long full-tiller cut-in hundreds of metres before the intersection.
    double tightTurnForceFullSteerDistanceM = 55.0;
    double tightTurnForceFullSteerRatio = 1.00;
    double tightTurnForceFullReleaseHeadingDeg = 24.0;
    // Before the full-tiller window, allow anticipation but cap the requested tiller so
    // the aircraft does not cut the corner early.
    double tightTurnPreFullSteerCapRatio = 0.45;
    double tightTurnPreFullSnapMinRatio = 0.12;

    // Corner anticipation makes the taxi controller behave more like an AP lateral mode:
    // it starts blending toward the next leg before the nose reaches the node, instead of
    // waiting until the aircraft is already at the intersection and then correcting back.
    bool turnAnticipation = true;
    double turnAnticipationDistanceM = 135.0;
    double turnAnticipationMinAngleDeg = 15.0;
    double turnAnticipationExtraLookaheadM = 45.0;
    double turnAnticipationSlowdownKts = 4.5;

    // Tight-corner mode. A very long look-ahead makes a heavy A350 draw a large arc
    // through 90-degree taxiway turns. In tight-corner mode the target point is placed
    // a short distance after the corner apex instead of far down the next leg; speed,
    // steering authority, and differential braking are also scheduled for a smaller radius.
    bool tightTurnMode = true;
    double tightTurnAngleDeg = 25.0;
    double tightTurnTriggerDistanceM = 115.0;
    double tightTurnLookaheadAfterApexM = 18.0;
    double tightTurnMinLookaheadM = 14.0;
    double tightTurnSpeedKts = 3.0;
    double tightTurnSteerFullDeflectionDeg = 10.0;
    double tightTurnSteerSmoothingPerSec = 60.0;
    double tightTurnKpBoost = 2.50;
    double tightTurnKdBoost = 1.00;
    double tightTurnDifferentialBrakeMax = 0.28;
    double tightTurnDifferentialBrakeMinKts = 0.5;

    // Steering response boost. The old rate limiter could take about a second to
    // build large tiller input. In a tight turn we want the commanded value to
    // arrive quickly, otherwise the A350 draws a wide arc before the nose wheel catches up.
    bool steerFastResponse = true;
    double steerFastResponseErrorDeg = 10.0;
    double steerFastResponseXteM = 2.5;
    double steerFastResponseRatePerSec = 60.0;
    bool tightTurnSnapSteer = true;
    double tightTurnSnapBlend = 0.0;
    double tightTurnSnapMinTargetRatio = 0.12;
    double tightTurnSnapToRatio = 0.45;

    // One-pass 70-100 degree cornering. This is intentionally aggressive at the
    // beginning of a 90-degree taxiway turn, then rolls out before the nose passes
    // the outbound leg heading so the aircraft does not overshoot and S-turn back.
    bool tightTurnOnePassMode = true;
    double tightTurnOnePassAngleDeg = 70.0;
    double tightTurnHardSnapToRatio = 1.00;
    double tightTurnHardSnapMinTargetRatio = 0.98;
    double tightTurnRolloutHeadingDeg = 24.0;
    double tightTurnRolloutGain = 1.80;
    double tightTurnRolloutMinPidDeg = 3.0;

    double routeSegmentAdvanceAlong = 0.68;
    double routeSegmentScanAhead = 4.0;

    // Outputs.
    double maxThrottle = 0.48;
    double creepThrottle = 0.18;       // Low-speed creep thrust when the first waypoint is not aligned.
    double breakawayThrottle = 0.38;   // Temporary thrust boost if the aircraft is armed but not moving.
    double stuckBoostSeconds = 2.0;

    // Dynamic taxi speed governor. The old output was mostly a fixed throttle step.
    // This schedules thrust every frame from target speed, current speed, path capture
    // severity, terminal slowdown and tight-turn mode, then rate-limits the throttle.
    bool dynamicThrottleControl = true;
    double throttleIdle = 0.035;
    double throttleTrim = 0.110;
    double throttleMinMoving = 0.070;
    double throttleSpeedKp = 0.055;          // throttle ratio per kt of speed error.
    double throttleCaptureReduction = 0.35;  // reduce thrust during aggressive path capture.
    double throttleTurnReduction = 0.45;     // reduce thrust in tight turns.
    double throttleTerminalReduction = 0.60; // reduce thrust near final stop/hold-short.
    double throttleIncreaseRatePerSec = 0.40;
    double throttleDecreaseRatePerSec = 0.55;
    double throttleOverspeedBrakeDeadbandKts = 0.6;

    bool commandAssist = true;         // Also issue X-Plane brake/throttle commands when available.
    double maxBrake = 0.65;
    double brakeGain = 0.22;
    // Steering is deliberately conservative for a heavy A350.
    // The final command is scheduled by speed and capped before it is written.
    double steerFullDeflectionDeg = 70.0;
    double steerDeadbandDeg = 2.0;
    double steerSmoothingPerSec = 3.50;
    double maxSteerRatio = 1.00;
    double lowSpeedMaxSteerRatio = 1.00;
    double highSpeedMaxSteerRatio = 0.70;
    double highSpeedSteerKts = 10.0;
    double steerCurveExponent = 1.00;
    double differentialBrakeThresholdDeg = 10.0;
    double differentialBrakeMax = 0.14;

    // Optional F/CTL visual/input coupling while taxi turning.
    // This can make the ECAM F/CTL rudder/roll surfaces follow the turn command.
    bool fctlTurnCoupling = true;
    double fctlRudderGain = 0.45;
    double fctlRollGain = 0.28;
    double fctlMaxRatio = 0.35;

    // Optional secondary F/CTL assist for the SLATS/FLAPS line shown on ECAM F/CTL.
    // It does not physically replace nosewheel steering; it only asks X-Plane/default-compatible
    // aircraft to move flap/slat controls during tight taxi turns. Third-party A350s may ignore
    // these default datarefs unless you map them with DataRefTool.
    bool fctlSecondaryAssist = true;
    double fctlSecondaryMinTightBlend = 0.20;
    double fctlSecondaryFlapRatio = 0.16;
    bool fctlSecondaryRestoreOnRelease = true;

    // Runway destination line-up. If Destination is a runway, use the apt.dat runway
    // endpoints to capture runway centreline and runway heading before declaring arrival.
    bool runwayAlignmentMode = true;
    double runwayAlignCaptureDistanceM = 420.0;
    double runwayAlignFullDistanceM = 260.0;
    double runwayAlignCenterGainDegPerM = 0.85;
    double runwayAlignMaxInterceptDeg = 34.0;
    double runwayAlignSpeedKts = 8.0;
    double runwayAlignCenterToleranceM = 4.0;
    double runwayAlignHeadingToleranceDeg = 6.0;

    // Routing safety.
    bool avoidRunwayEdges = true;
    double runwayPenaltyM = 5000.0;
    double activeZonePenaltyM = 2000.0;
    double turnPenaltyM = 35.0;
    bool autoReplanIfOffRoute = true;
    double offRouteReplanDistanceM = 90.0;
    double maxStartNodeDistanceM = 180.0;
    double maxStartSpeedKts = 8.0;
    double gatePositionRadiusM = 140.0;
    double taxiwayPositionRadiusM = 65.0;
    bool holdParkingBrakeAtDestination = true;

    // Route preview.
    int routeSummaryMaxNames = 12;

    std::string steerDataref = "sim/cockpit2/controls/yoke_heading_ratio";
    std::string fctlRudderDataref = "sim/cockpit2/controls/yoke_heading_ratio";
    std::string fctlRollDataref = "sim/cockpit2/controls/yoke_roll_ratio";
    std::string fctlRudderLegacyDataref = "sim/joystick/yoke_heading_ratio";
    std::string fctlRollLegacyDataref = "sim/joystick/yoke_roll_ratio";
    std::string fctlFlapRatioDataref = "sim/cockpit2/controls/flap_ratio";
    std::string fctlFlapRequestDataref = "sim/flightmodel/controls/flaprqst";
    std::string fctlSlatRatioDataref = "sim/flightmodel/controls/slatrat";
    std::string throttleAllDataref = "sim/cockpit2/engine/actuators/throttle_ratio_all";
    std::string throttleArrayDataref = "sim/cockpit2/engine/actuators/throttle_ratio";
    std::string leftBrakeDataref = "sim/flightmodel/controls/l_brake_add";
    std::string rightBrakeDataref = "sim/flightmodel/controls/r_brake_add";
    std::string parkingBrakeDataref = "sim/cockpit2/controls/parking_brake_ratio";
};

AutoTaxiConfig loadAutoTaxiConfig(const std::string& iniPath, std::string& error);

std::string trim(std::string s);
std::string toUpper(std::string s);
bool iequals(const std::string& a, const std::string& b);
bool icontains(const std::string& haystack, const std::string& needle);
std::vector<std::string> splitCsv(const std::string& text);
std::string joinTokens(const std::vector<std::string>& tokens, size_t first);

} // namespace autotaxi
