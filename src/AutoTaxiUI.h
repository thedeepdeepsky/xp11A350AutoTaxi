#pragma once
#include "AutoTaxi.h"

#include "XPLMDisplay.h"

#include <string>
#include <vector>

namespace autotaxi {

class AutoTaxiUI {
public:
    explicit AutoTaxiUI(AutoTaxiSystem& system);
    ~AutoTaxiUI();

    void create();
    void destroy();
    void toggleVisible();
    void setVisible(bool visible);
    bool isVisible() const;

    struct Rect {
        int l = 0, t = 0, r = 0, b = 0;
        bool contains(int x, int y) const { return x >= l && x <= r && y <= t && y >= b; }
    };

private:
    enum class Filter { All, Runway, GateRamp, Coordinate };

    AutoTaxiSystem& system_;
    XPLMWindowID window_ = nullptr;
    bool dropdownOpen_ = false;
    int dropdownScroll_ = 0;
    Filter filter_ = Filter::All;

    Rect closeRect_;
    Rect refreshRect_;
    Rect filterRect_;
    Rect dropdownRect_;
    Rect dropdownHitRect_;
    Rect dropdownListRect_;
    Rect prevDestRect_;
    Rect nextDestRect_;
    Rect planRect_;
    Rect startRect_;
    Rect stopRect_;

    void computeLayout(int left, int top, int right, int bottom);
    void drawPanel(int left, int top, int right, int bottom);
    void drawDropdownList();
    void draw(XPLMWindowID windowId);
    int handleMouseClick(XPLMWindowID windowId, int x, int y, XPLMMouseStatus status);
    void handleKey(XPLMWindowID windowId, char key, XPLMKeyFlags flags, char virtualKey, int losingFocus);
    XPLMCursorStatus handleCursor(XPLMWindowID windowId, int x, int y);
    int handleMouseWheel(XPLMWindowID windowId, int x, int y, int wheel, int clicks);

    std::vector<int> filteredDestinationIndices() const;
    bool selectFilteredRelative(int delta);
    void cycleFilter();
    std::string filterName() const;
    void clampScroll();

    static void drawCallback(XPLMWindowID windowId, void* refcon);
    static int mouseCallback(XPLMWindowID windowId, int x, int y, XPLMMouseStatus status, void* refcon);
    static void keyCallback(XPLMWindowID windowId, char key, XPLMKeyFlags flags, char virtualKey, void* refcon, int losingFocus);
    static XPLMCursorStatus cursorCallback(XPLMWindowID windowId, int x, int y, void* refcon);
    static int wheelCallback(XPLMWindowID windowId, int x, int y, int wheel, int clicks, void* refcon);
};

} // namespace autotaxi
