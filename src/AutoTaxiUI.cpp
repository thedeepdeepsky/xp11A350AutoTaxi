#include "AutoTaxiUI.h"
#include "Config.h"

#include "XPLMGraphics.h"
#include "XPLMUtilities.h"

#if IBM
    #include <windows.h>
    #include <GL/gl.h>
#elif APL
    #include <OpenGL/gl.h>
#else
    #include <GL/gl.h>
#endif

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <utility>
#include <vector>

namespace autotaxi {
namespace {

constexpr int kWindowW = 720;
constexpr int kWindowH = 560;
constexpr int kMargin = 18;
constexpr int kRowH = 28;
constexpr int kMaxRows = 9;

float kWhite[] = {0.95f, 0.96f, 0.98f};
float kMuted[] = {0.68f, 0.74f, 0.78f};
float kTitle[] = {0.80f, 0.92f, 1.00f};
float kGood[] = {0.56f, 0.95f, 0.68f};
float kWarn[] = {1.00f, 0.78f, 0.38f};
float kBad[] = {1.00f, 0.45f, 0.42f};
float kAccent[] = {0.50f, 0.80f, 1.00f};
float kPanel[] = {0.03f, 0.05f, 0.07f};

std::string fmt(double value, int decimals = 1) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(decimals) << value;
    return oss.str();
}

std::string trunc(std::string s, size_t maxLen) {
    if (s.size() <= maxLen) return s;
    if (maxLen <= 3) return s.substr(0, maxLen);
    return s.substr(0, maxLen - 3) + "...";
}

std::string basename(const std::string& path) {
    try { return std::filesystem::path(path).filename().generic_string(); }
    catch (...) { return path; }
}

void setState() {
    XPLMSetGraphicsState(0, 0, 0, 0, 1, 0, 0);
}

void fillRect(int l, int t, int r, int b, float red, float green, float blue, float alpha) {
    setState();
    glColor4f(red, green, blue, alpha);
    glBegin(GL_QUADS);
    glVertex2i(l, b);
    glVertex2i(r, b);
    glVertex2i(r, t);
    glVertex2i(l, t);
    glEnd();
}

void outlineRect(int l, int t, int r, int b, float red, float green, float blue, float alpha) {
    setState();
    glColor4f(red, green, blue, alpha);
    glBegin(GL_LINE_LOOP);
    glVertex2i(l, b);
    glVertex2i(r, b);
    glVertex2i(r, t);
    glVertex2i(l, t);
    glEnd();
}

void progressBar(int l, int t, int r, int b, double pct) {
    fillRect(l, t, r, b, 0.08f, 0.10f, 0.12f, 0.94f);
    outlineRect(l, t, r, b, 0.36f, 0.54f, 0.65f, 0.52f);
    const int w = std::max(0, r - l - 2);
    const int fillW = static_cast<int>(w * std::max(0.0, std::min(100.0, pct)) / 100.0);
    if (fillW > 0) fillRect(l + 1, t - 1, l + 1 + fillW, b + 1, 0.22f, 0.56f, 0.72f, 0.80f);
}

void drawText(int x, int y, const std::string& text, float* color = kWhite, XPLMFontID font = xplmFont_Basic) {
    // X-Plane 11 SDK keeps a legacy char* parameter here even though the string is not modified.
    // Use a mutable, null-terminated copy so MinGW/G++ does not reject const char* -> char*.
    std::vector<char> buffer(text.begin(), text.end());
    buffer.push_back('\0');
    XPLMDrawString(color, x, y, buffer.data(), nullptr, font);
}

void drawButton(const AutoTaxiUI::Rect& rc, const std::string& text, bool enabled, bool primary = false) {
    if (enabled) {
        if (primary) fillRect(rc.l, rc.t, rc.r, rc.b, 0.12f, 0.35f, 0.48f, 0.96f);
        else fillRect(rc.l, rc.t, rc.r, rc.b, 0.12f, 0.17f, 0.21f, 0.94f);
        outlineRect(rc.l, rc.t, rc.r, rc.b, 0.50f, 0.80f, 1.00f, primary ? 0.92f : 0.45f);
        drawText(rc.l + 12, rc.b + 9, text, primary ? kWhite : kAccent);
    } else {
        fillRect(rc.l, rc.t, rc.r, rc.b, 0.08f, 0.09f, 0.10f, 0.72f);
        outlineRect(rc.l, rc.t, rc.r, rc.b, 0.35f, 0.38f, 0.42f, 0.35f);
        drawText(rc.l + 12, rc.b + 9, text, kMuted);
    }
}

std::vector<std::pair<int, int>> mousePointCandidates(int x, int y, int left, int top, int right, int bottom) {
    std::vector<std::pair<int, int>> pts;
    auto addUnique = [&](int px, int py) {
        for (const auto& p : pts) {
            if (p.first == px && p.second == py) return;
        }
        pts.emplace_back(px, py);
    };

    addUnique(x, y); // X-Plane 11 modern windows usually pass global boxel coordinates.

    const int w = std::max(0, right - left);
    const int h = std::max(0, top - bottom);
    if (x >= 0 && x <= w && y >= 0 && y <= h) {
        addUnique(left + x, bottom + y); // Some setups pass local bottom-left coordinates.
        addUnique(left + x, top - y);    // Defensive fallback for local top-left coordinates.
    }
    return pts;
}

bool hitRect(const AutoTaxiUI::Rect& rc,
             const std::vector<std::pair<int, int>>& pts,
             int* hitX = nullptr,
             int* hitY = nullptr) {
    constexpr int pad = 6; // Make button hit zones slightly forgiving on high-DPI screens.
    AutoTaxiUI::Rect padded{rc.l - pad, rc.t + pad, rc.r + pad, rc.b - pad};
    for (const auto& p : pts) {
        if (padded.contains(p.first, p.second)) {
            if (hitX) *hitX = p.first;
            if (hitY) *hitY = p.second;
            return true;
        }
    }
    return false;
}


} // namespace

AutoTaxiUI::AutoTaxiUI(AutoTaxiSystem& system) : system_(system) {}

AutoTaxiUI::~AutoTaxiUI() {
    destroy();
}

void AutoTaxiUI::create() {
    if (window_) return;

    int screenL = 0, screenT = 900, screenR = 1400, screenB = 0;
    XPLMGetScreenBoundsGlobal(&screenL, &screenT, &screenR, &screenB);

    XPLMCreateWindow_t params{};
    params.structSize = sizeof(params);
    params.left = screenL + 80;
    params.top = screenT - 80;
    params.right = params.left + kWindowW;
    params.bottom = params.top - kWindowH;
    params.visible = 0;
    params.drawWindowFunc = drawCallback;
    params.handleMouseClickFunc = mouseCallback;
    params.handleKeyFunc = keyCallback;
    params.handleCursorFunc = cursorCallback;
    params.handleMouseWheelFunc = wheelCallback;
    params.refcon = this;
    params.decorateAsFloatingWindow = xplm_WindowDecorationSelfDecoratedResizable;
    params.layer = xplm_WindowLayerFloatingWindows;
    params.handleRightClickFunc = mouseCallback;

    window_ = XPLMCreateWindowEx(&params);
    if (window_) XPLMSetWindowTitle(window_, "A350 AutoTaxi GSX Mode");
}

void AutoTaxiUI::destroy() {
    if (window_) {
        XPLMDestroyWindow(window_);
        window_ = nullptr;
    }
}

void AutoTaxiUI::toggleVisible() { setVisible(!isVisible()); }

void AutoTaxiUI::setVisible(bool visible) {
    if (!window_) create();
    if (window_) XPLMSetWindowIsVisible(window_, visible ? 1 : 0);
}

bool AutoTaxiUI::isVisible() const {
    return window_ && XPLMGetWindowIsVisible(window_) != 0;
}

void AutoTaxiUI::computeLayout(int left, int top, int right, int bottom) {
    const int w = std::max(600, right - left);
    const int innerL = left + kMargin;
    const int innerR = left + w - kMargin;

    closeRect_ = {innerR - 68, top - 32, innerR, top - 8};
    refreshRect_ = {innerR - 168, top - 32, innerR - 78, top - 8};
    filterRect_ = {innerL, top - 214, innerL + 132, top - 248};
    dropdownRect_ = {innerL + 142, top - 214, innerR - 86, top - 248};
    prevDestRect_ = {innerR - 78, top - 214, innerR - 42, top - 248};
    nextDestRect_ = {innerR - 36, top - 214, innerR, top - 248};
    // A bigger invisible hit zone for destination selection. Some X-Plane 11/HiDPI setups
    // report mouse coordinates a few pixels outside the drawn rectangle, which made the
    // combo look clickable but not open.
    dropdownHitRect_ = {filterRect_.r + 4, dropdownRect_.t + 10, nextDestRect_.r + 10, dropdownRect_.b - 10};
    planRect_ = {innerL, bottom + 30, innerL + 170, bottom + 76};
    startRect_ = {innerL + 184, bottom + 30, innerL + 384, bottom + 76};
    stopRect_ = {innerL + 398, bottom + 30, innerL + 606, bottom + 76};
    dropdownListRect_ = {dropdownRect_.l, dropdownRect_.b, dropdownRect_.r, dropdownRect_.b - kMaxRows * kRowH};
}

std::string AutoTaxiUI::filterName() const {
    switch (filter_) {
        case Filter::All: return "ALL";
        case Filter::Runway: return "RUNWAY";
        case Filter::GateRamp: return "GATE/RAMP";
        case Filter::Coordinate: return "GPS/NODE";
    }
    return "ALL";
}

void AutoTaxiUI::cycleFilter() {
    switch (filter_) {
        case Filter::All: filter_ = Filter::Runway; break;
        case Filter::Runway: filter_ = Filter::GateRamp; break;
        case Filter::GateRamp: filter_ = Filter::Coordinate; break;
        case Filter::Coordinate: filter_ = Filter::All; break;
    }
    dropdownScroll_ = 0;
}

std::vector<int> AutoTaxiUI::filteredDestinationIndices() const {
    std::vector<int> out;
    const auto& choices = system_.destinationChoices();
    for (int i = 0; i < static_cast<int>(choices.size()); ++i) {
        const auto& d = choices[i];
        bool ok = false;
        switch (filter_) {
            case Filter::All: ok = true; break;
            case Filter::Runway: ok = d.kind == DestinationKind::Runway; break;
            case Filter::GateRamp: ok = d.kind == DestinationKind::Ramp; break;
            case Filter::Coordinate: ok = d.kind == DestinationKind::Coordinate || d.kind == DestinationKind::Node; break;
        }
        if (ok) out.push_back(i);
    }
    return out;
}

bool AutoTaxiUI::selectFilteredRelative(int delta) {
    const auto filtered = filteredDestinationIndices();
    if (filtered.empty()) return false;

    int pos = 0;
    const int selected = system_.selectedDestinationIndex();
    for (int i = 0; i < static_cast<int>(filtered.size()); ++i) {
        if (filtered[i] == selected) { pos = i; break; }
    }
    pos = (pos + delta) % static_cast<int>(filtered.size());
    if (pos < 0) pos += static_cast<int>(filtered.size());
    return system_.selectDestinationIndex(filtered[pos]);
}

void AutoTaxiUI::drawPanel(int left, int top, int right, int bottom) {
    computeLayout(left, top, right, bottom);

    const auto st = system_.snapshot();
    const bool airportReady = st.airportLoaded;
    const bool canPlan = airportReady && st.selectedDestinationIndex >= 0 && !st.active;
    const bool canStart = canPlan && st.controlsReady;

    fillRect(left, top, right, bottom, kPanel[0], kPanel[1], kPanel[2], 0.97f);
    outlineRect(left, top, right, bottom, 0.30f, 0.56f, 0.72f, 0.65f);

    drawText(left + kMargin, top - 24, "A350 AutoTaxi - GSX Style Ground Routing", kTitle);
    drawButton(refreshRect_, "Refresh", true, false);
    drawButton(closeRect_, "Close", true, false);

    float* statusColor = st.active ? kGood : (st.arrived ? kWarn : kMuted);
    std::string status = st.active ? "ACTIVE" : (st.arrived ? "ARRIVED" : "STANDBY");
    if (!st.controlsReady) status += " / DATAREF WARN";
    drawText(left + kMargin, top - 54, "Status: " + status, st.controlsReady ? statusColor : kWarn);

    const std::string airport = airportReady
        ? st.airportId + "  " + trunc(st.airportName, 54)
        : "Not loaded. Click Refresh at an airport.";
    drawText(left + kMargin, top - 82, "Airport: " + airport, airportReady ? kWhite : kWarn);

    drawText(left + kMargin, top - 106,
             "apt.dat: candidates " + std::to_string(st.aptDatCandidateCount) +
             " / priority " + std::to_string(st.sourcePriority) +
             " / " + trunc(basename(st.sourceFile), 54), airportReady ? kMuted : kWarn);

    const std::string pos = airportReady && !st.positionLabel.empty()
        ? st.positionLabel
        : "Unknown / outside loaded taxi network";
    drawText(left + kMargin, top - 132, "Current position: " + trunc(pos, 86), airportReady ? kWhite : kMuted);

    std::string edge = st.nearestEdgeIdent.empty() ? "-" : st.nearestEdgeIdent;
    drawText(left + kMargin, top - 156,
             "Nearest edge: " + trunc(edge, 42) + " / " + fmt(st.nearestEdgeDistanceM, 1) + " m" +
             "    GS: " + fmt(st.aircraftGroundSpeedKts, 1) + " kt" +
             "    HDG: " + fmt(st.aircraftHeadingDeg, 0), kMuted);

    if (st.active || st.lastThrottleCmd > 0.001 || st.lastLeftBrakeCmd > 0.001 || st.lastRightBrakeCmd > 0.001) {
        drawText(left + kMargin, top - 178,
                 "Control: steer " + fmt(st.lastSteerCmd, 2) +
                 " / throttle " + fmt(st.lastThrottleCmd, 2) +
                 " tgt " + fmt(st.lastTargetSpeedKts, 1) + "kt" +
                 (st.throttleMode.empty() ? std::string() : " " + st.throttleMode) +
                 " / brake L-R " + fmt(st.lastLeftBrakeCmd, 2) + "-" + fmt(st.lastRightBrakeCmd, 2) +
                 " / xte " + fmt(st.lastCrossTrackErrorM, 1) + "m" +
                 " / pid " + fmt(st.lastPidOutputDeg, 1) + "deg" +
                 " / fctl " + fmt(st.lastFctlRudderCmd, 2) + "/" + fmt(st.lastFctlRollCmd, 2),
                 st.active ? kGood : kMuted);
    }

    drawText(left + kMargin, top - 204, "Destination type", kMuted);
    drawButton(filterRect_, filterName(), airportReady, false);

    fillRect(dropdownRect_.l, dropdownRect_.t, dropdownRect_.r, dropdownRect_.b, 0.08f, 0.11f, 0.14f, 0.97f);
    outlineRect(dropdownRect_.l, dropdownRect_.t, dropdownRect_.r, dropdownRect_.b, 0.50f, 0.80f, 1.00f, 0.70f);
    const std::string selected = st.selectedDestinationLabel.empty() ? "<select runway / gate / ramp>" : st.selectedDestinationLabel;
    drawText(dropdownRect_.l + 12, dropdownRect_.b + 12, trunc(selected, 58), airportReady ? kWhite : kMuted);
    drawText(dropdownRect_.r - 24, dropdownRect_.b + 12, dropdownOpen_ ? "^" : "v", kAccent);
    drawButton(prevDestRect_, "<", airportReady && st.destinationCount > 0, false);
    drawButton(nextDestRect_, ">", airportReady && st.destinationCount > 0, false);

    drawText(left + kMargin, dropdownRect_.b - 30,
             "Network: nodes " + std::to_string(st.taxiNodeCount) +
             " / edges " + std::to_string(st.taxiEdgeCount) +
             " / gates-ramps " + std::to_string(st.rampCount) +
             " / runway ends " + std::to_string(st.runwayEndCount), kMuted);

    if (st.routeReady) {
        drawText(left + kMargin, dropdownRect_.b - 58,
                 "Route: " + fmt(st.routeDistanceM, 0) + " m / ETA " + fmt(st.routeEtaSec / 60.0, 1) +
                 " min / " + std::to_string(st.routeNodeCount) + " nodes", kGood);
        drawText(left + kMargin, dropdownRect_.b - 82, "Path: " + trunc(st.routeSummary, 92), kWhite);
        drawText(left + kMargin, dropdownRect_.b - 106, "Safety: " + trunc(st.routeSafety, 92),
                 st.routeSafety.find("WARNING") != std::string::npos ? kBad : kMuted);
        if (!st.nextInstruction.empty()) {
            drawText(left + kMargin, dropdownRect_.b - 130, "Next: " + trunc(st.nextInstruction, 92), kMuted);
        }
        progressBar(left + kMargin, dropdownRect_.b - 150, right - kMargin, dropdownRect_.b - 164, st.routeProgressPct);
        drawText(right - kMargin - 82, dropdownRect_.b - 160, fmt(st.routeProgressPct, 0) + "%", kMuted);
    } else {
        drawText(left + kMargin, dropdownRect_.b - 58, "Route: not planned. Choose destination and click Plan Route.", kWarn);
    }

    if (!st.selectedDestinationDetail.empty()) {
        drawText(left + kMargin, bottom + 82, "Selected detail: " + trunc(st.selectedDestinationDetail, 88), kMuted);
    }

    if (!st.lastMessage.empty()) {
        drawText(left + kMargin, bottom + 104, trunc("Log: " + st.lastMessage, 96), kMuted);
    }

    drawButton(planRect_, "Plan Route", canPlan, false);
    drawButton(startRect_, st.active ? "Running" : "Start AutoTaxi", canStart && !st.active, true);
    drawButton(stopRect_, "Stop / Release", st.active || st.routeReady || st.arrived, false);

    if (dropdownOpen_) drawDropdownList();
}

void AutoTaxiUI::drawDropdownList() {
    clampScroll();
    const auto filtered = filteredDestinationIndices();
    const auto& choices = system_.destinationChoices();
    if (filtered.empty()) {
        dropdownListRect_ = {dropdownRect_.l, dropdownRect_.b, dropdownRect_.r, dropdownRect_.b - kRowH};
        fillRect(dropdownListRect_.l, dropdownListRect_.t, dropdownListRect_.r, dropdownListRect_.b,
                 0.04f, 0.06f, 0.08f, 0.99f);
        outlineRect(dropdownListRect_.l, dropdownListRect_.t, dropdownListRect_.r, dropdownListRect_.b,
                    0.50f, 0.80f, 1.00f, 0.75f);
        drawText(dropdownListRect_.l + 10, dropdownListRect_.b + 9, "No destinations in this filter", kWarn);
        return;
    }

    const int rows = std::min(kMaxRows, static_cast<int>(filtered.size()) - dropdownScroll_);
    const int listBottom = dropdownRect_.b - rows * kRowH;
    dropdownListRect_ = {dropdownRect_.l, dropdownRect_.b, dropdownRect_.r, listBottom};

    fillRect(dropdownListRect_.l, dropdownListRect_.t, dropdownListRect_.r, dropdownListRect_.b,
             0.04f, 0.06f, 0.08f, 0.99f);
    outlineRect(dropdownListRect_.l, dropdownListRect_.t, dropdownListRect_.r, dropdownListRect_.b,
                0.50f, 0.80f, 1.00f, 0.75f);

    const int selected = system_.selectedDestinationIndex();
    for (int row = 0; row < rows; ++row) {
        const int globalIdx = filtered[dropdownScroll_ + row];
        const auto& d = choices[globalIdx];
        const int rowTop = dropdownRect_.b - row * kRowH;
        const int rowBottom = rowTop - kRowH;
        if (globalIdx == selected) fillRect(dropdownListRect_.l + 1, rowTop, dropdownListRect_.r - 1, rowBottom, 0.11f, 0.30f, 0.42f, 0.85f);
        else if (row % 2 == 1) fillRect(dropdownListRect_.l + 1, rowTop, dropdownListRect_.r - 1, rowBottom, 0.07f, 0.09f, 0.11f, 0.70f);

        float* groupColor = d.heavyCompatible ? kAccent : kWarn;
        drawText(dropdownListRect_.l + 10, rowBottom + 9, d.group.empty() ? "DEST" : d.group, groupColor);
        drawText(dropdownListRect_.l + 92, rowBottom + 9, trunc(d.label, 50), d.heavyCompatible ? kWhite : kWarn);
        if (!d.detail.empty()) drawText(dropdownListRect_.r - 210, rowBottom + 9, trunc(d.detail, 30), kMuted);
    }

    if (static_cast<int>(filtered.size()) > rows) {
        drawText(dropdownListRect_.r - 84, dropdownListRect_.b + 8,
                 std::to_string(dropdownScroll_ + 1) + "/" + std::to_string(filtered.size()), kMuted);
    }
}

void AutoTaxiUI::draw(XPLMWindowID windowId) {
    int l = 0, t = 0, r = 0, b = 0;
    XPLMGetWindowGeometry(windowId, &l, &t, &r, &b);
    drawPanel(l, t, r, b);
}

int AutoTaxiUI::handleMouseClick(XPLMWindowID windowId, int x, int y, XPLMMouseStatus status) {
    if (status != xplm_MouseDown) return 1;

    int l = 0, t = 0, r = 0, b = 0;
    XPLMGetWindowGeometry(windowId, &l, &t, &r, &b);
    computeLayout(l, t, r, b);

    const auto pts = mousePointCandidates(x, y, l, t, r, b);
    auto hit = [&](const Rect& rc, int* hx = nullptr, int* hy = nullptr) {
        return hitRect(rc, pts, hx, hy);
    };

    if (hit(closeRect_)) {
        setVisible(false);
        dropdownOpen_ = false;
        return 1;
    }

    if (hit(refreshRect_)) {
        dropdownOpen_ = false;
        dropdownScroll_ = 0;
        system_.reload();
        return 1;
    }

    if (hit(filterRect_)) {
        if (!system_.hasAirport()) system_.reload();
        cycleFilter();
        dropdownOpen_ = true;
        clampScroll();
        return 1;
    }

    if (hit(prevDestRect_)) {
        if (!system_.hasAirport()) system_.reload();
        selectFilteredRelative(-1);
        dropdownOpen_ = false;
        return 1;
    }

    if (hit(nextDestRect_)) {
        if (!system_.hasAirport()) system_.reload();
        selectFilteredRelative(1);
        dropdownOpen_ = false;
        return 1;
    }

    if (hit(dropdownRect_) || hit(dropdownHitRect_)) {
        if (!system_.hasAirport()) system_.reload();
        dropdownOpen_ = !dropdownOpen_;
        clampScroll();
        return 1;
    }

    if (dropdownOpen_) {
        clampScroll();
        const auto filtered = filteredDestinationIndices();
        const int rows = filtered.empty() ? 1 : std::min(kMaxRows, static_cast<int>(filtered.size()) - dropdownScroll_);
        dropdownListRect_ = {dropdownRect_.l, dropdownRect_.b, dropdownRect_.r, dropdownRect_.b - rows * kRowH};
        int hitX = x;
        int hitY = y;
        if (hit(dropdownListRect_, &hitX, &hitY)) {
            if (!filtered.empty()) {
                const int row = (dropdownListRect_.t - hitY) / kRowH;
                const int localIdx = dropdownScroll_ + row;
                if (localIdx >= 0 && localIdx < static_cast<int>(filtered.size())) {
                    system_.selectDestinationIndex(filtered[localIdx]);
                    dropdownOpen_ = false;
                }
            }
            return 1;
        }
        dropdownOpen_ = false;
        return 1;
    }

    if (hit(planRect_)) {
        dropdownOpen_ = false;
        if (!system_.hasAirport()) system_.reload();
        system_.planSelectedDestination();
        return 1;
    }

    if (hit(startRect_)) {
        dropdownOpen_ = false;
        if (!system_.hasAirport()) system_.reload();
        system_.startSelectedDestination();
        return 1;
    }

    if (hit(stopRect_)) {
        dropdownOpen_ = false;
        system_.stopByUser();
        return 1;
    }

    return 1;
}

void AutoTaxiUI::handleKey(XPLMWindowID, char key, XPLMKeyFlags, char, int losingFocus) {
    if (losingFocus) return;
    if (key == 27) dropdownOpen_ = false;
}

XPLMCursorStatus AutoTaxiUI::handleCursor(XPLMWindowID, int, int) {
    return xplm_CursorArrow;
}

int AutoTaxiUI::handleMouseWheel(XPLMWindowID windowId, int x, int y, int wheel, int clicks) {
    if (wheel != 0) return 1;

    int l = 0, t = 0, r = 0, b = 0;
    XPLMGetWindowGeometry(windowId, &l, &t, &r, &b);
    computeLayout(l, t, r, b);
    const auto pts = mousePointCandidates(x, y, l, t, r, b);

    if (!dropdownOpen_ && hitRect(dropdownHitRect_, pts)) {
        dropdownOpen_ = true;
    }
    if (!dropdownOpen_) return 1;

    dropdownScroll_ += clicks;
    clampScroll();
    return 1;
}

void AutoTaxiUI::clampScroll() {
    const int n = static_cast<int>(filteredDestinationIndices().size());
    const int maxScroll = std::max(0, n - kMaxRows);
    dropdownScroll_ = std::max(0, std::min(dropdownScroll_, maxScroll));
}

void AutoTaxiUI::drawCallback(XPLMWindowID windowId, void* refcon) {
    static_cast<AutoTaxiUI*>(refcon)->draw(windowId);
}

int AutoTaxiUI::mouseCallback(XPLMWindowID windowId, int x, int y, XPLMMouseStatus status, void* refcon) {
    return static_cast<AutoTaxiUI*>(refcon)->handleMouseClick(windowId, x, y, status);
}

void AutoTaxiUI::keyCallback(XPLMWindowID windowId, char key, XPLMKeyFlags flags, char virtualKey, void* refcon, int losingFocus) {
    static_cast<AutoTaxiUI*>(refcon)->handleKey(windowId, key, flags, virtualKey, losingFocus);
}

XPLMCursorStatus AutoTaxiUI::cursorCallback(XPLMWindowID windowId, int x, int y, void* refcon) {
    return static_cast<AutoTaxiUI*>(refcon)->handleCursor(windowId, x, y);
}

int AutoTaxiUI::wheelCallback(XPLMWindowID windowId, int x, int y, int wheel, int clicks, void* refcon) {
    return static_cast<AutoTaxiUI*>(refcon)->handleMouseWheel(windowId, x, y, wheel, clicks);
}

} // namespace autotaxi
