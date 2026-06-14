#include "AutoTaxi.h"
#include "AutoTaxiUI.h"

#include "XPLMDefs.h"
#include "XPLMMenus.h"
#include "XPLMPlugin.h"
#include "XPLMProcessing.h"
#include "XPLMUtilities.h"

#include <cstring>
#include <filesystem>
#include <memory>
#include <string>

namespace fs = std::filesystem;

namespace {

std::unique_ptr<autotaxi::AutoTaxiSystem> gAutoTaxi;
std::unique_ptr<autotaxi::AutoTaxiUI> gUI;
XPLMFlightLoopID gFlightLoop = nullptr;
XPLMMenuID gMenu = nullptr;
XPLMCommandRef gToggleCmd = nullptr;
XPLMCommandRef gReloadCmd = nullptr;
XPLMCommandRef gPlanCmd = nullptr;
XPLMCommandRef gPanelCmd = nullptr;

void safeCopy(char* dst, const char* src) {
    if (!dst) return;
    std::strncpy(dst, src, 255);
    dst[255] = '\0';
}

void logLine(const std::string& s) {
    XPLMDebugString((s + "\n").c_str());
}

std::string pluginRootFromBinaryPath(const std::string& binaryPath) {
    fs::path p(binaryPath);

    // Typical fat plugin:
    //   Resources/plugins/A350AutoTaxi/64/win.xpl
    // The plugin root is parent of "64".
    auto parent = p.parent_path();
    if (parent.filename() == "64" || parent.filename() == "32") {
        return parent.parent_path().generic_string();
    }

    return parent.generic_string();
}

float flightLoopCallback(float elapsedSinceLastCall,
                         float elapsedTimeSinceLastFlightLoop,
                         int counter,
                         void* refcon) {
    (void)elapsedTimeSinceLastFlightLoop;
    (void)counter;
    (void)refcon;

    if (!gAutoTaxi) return 0.5f;
    return gAutoTaxi->flightLoop(elapsedSinceLastCall);
}

int commandHandler(XPLMCommandRef commandRef,
                   XPLMCommandPhase phase,
                   void* refcon) {
    (void)refcon;
    if (phase != xplm_CommandBegin) return 1;

    if (!gAutoTaxi) return 1;

    if (commandRef == gToggleCmd) {
        gAutoTaxi->toggle();
    } else if (commandRef == gReloadCmd) {
        gAutoTaxi->reload();
    } else if (commandRef == gPlanCmd) {
        gAutoTaxi->planSelectedDestination();
    } else if (commandRef == gPanelCmd && gUI) {
        gUI->toggleVisible();
    }

    return 1;
}

void menuHandler(void* menuRef, void* itemRef) {
    (void)menuRef;
    const auto action = reinterpret_cast<intptr_t>(itemRef);
    if (!gAutoTaxi) return;

    if (action == 1 && gUI) {
        gUI->toggleVisible();
    } else if (action == 2) {
        gAutoTaxi->toggle();
    } else if (action == 3) {
        gAutoTaxi->reload();
    } else if (action == 4) {
        gAutoTaxi->planSelectedDestination();
    }
}

void createMenuAndCommands() {
    gToggleCmd = XPLMCreateCommand("a350_autotaxi/toggle", "Toggle A350 AutoTaxi");
    gReloadCmd = XPLMCreateCommand("a350_autotaxi/reload", "Reload A350 AutoTaxi config and apt.dat");
    gPlanCmd = XPLMCreateCommand("a350_autotaxi/plan", "Plan A350 AutoTaxi route preview");
    gPanelCmd = XPLMCreateCommand("a350_autotaxi/panel", "Show or hide A350 AutoTaxi panel");

    XPLMRegisterCommandHandler(gToggleCmd, commandHandler, 1, nullptr);
    XPLMRegisterCommandHandler(gReloadCmd, commandHandler, 1, nullptr);
    XPLMRegisterCommandHandler(gPlanCmd, commandHandler, 1, nullptr);
    XPLMRegisterCommandHandler(gPanelCmd, commandHandler, 1, nullptr);

    const int item = XPLMAppendMenuItem(XPLMFindPluginsMenu(), "A350 AutoTaxi", nullptr, 1);
    gMenu = XPLMCreateMenu("A350 AutoTaxi", XPLMFindPluginsMenu(), item, menuHandler, nullptr);

    XPLMAppendMenuItem(gMenu, "Open AutoTaxi Panel", reinterpret_cast<void*>(1), 1);
    XPLMAppendMenuItem(gMenu, "Toggle AutoTaxi", reinterpret_cast<void*>(2), 1);
    XPLMAppendMenuItem(gMenu, "Reload Config/Apt.dat", reinterpret_cast<void*>(3), 1);
    XPLMAppendMenuItem(gMenu, "Plan Route Preview", reinterpret_cast<void*>(4), 1);
}

void destroyMenuAndCommands() {
    if (gMenu) {
        XPLMDestroyMenu(gMenu);
        gMenu = nullptr;
    }

    if (gToggleCmd) {
        XPLMUnregisterCommandHandler(gToggleCmd, commandHandler, 1, nullptr);
        gToggleCmd = nullptr;
    }

    if (gReloadCmd) {
        XPLMUnregisterCommandHandler(gReloadCmd, commandHandler, 1, nullptr);
        gReloadCmd = nullptr;
    }

    if (gPlanCmd) {
        XPLMUnregisterCommandHandler(gPlanCmd, commandHandler, 1, nullptr);
        gPlanCmd = nullptr;
    }

    if (gPanelCmd) {
        XPLMUnregisterCommandHandler(gPanelCmd, commandHandler, 1, nullptr);
        gPanelCmd = nullptr;
    }
}

} // namespace

// X-Plane loads plug-ins with GetProcAddress("XPluginStart", ...).
// When this file is compiled as C++, the exported entry point names must be
// unmangled C symbols. MinGW/MSVC builds are both covered here.
#ifndef A350_AUTOTAXI_PLUGIN_EXPORT
#  ifdef _WIN32
#    define A350_AUTOTAXI_PLUGIN_EXPORT extern "C" __declspec(dllexport)
#  else
#    define A350_AUTOTAXI_PLUGIN_EXPORT extern "C" __attribute__((visibility("default")))
#  endif
#endif

A350_AUTOTAXI_PLUGIN_EXPORT int XPluginStart(char* outName, char* outSig, char* outDesc) {
    safeCopy(outName, "A350 AutoTaxi");
    safeCopy(outSig, "com.example.a350autotaxi");
    safeCopy(outDesc, "A350 GSX-style auto taxi plugin using scenery-priority apt.dat taxi networks.");

    char systemPath[1024] = {};
    XPLMGetSystemPath(systemPath);

    char name[256] = {};
    char filePath[1024] = {};
    char sig[256] = {};
    char desc[256] = {};
    XPLMGetPluginInfo(XPLMGetMyID(), name, filePath, sig, desc);

    const std::string xplaneRoot = fs::path(systemPath).generic_string();
    const std::string pluginRoot = pluginRootFromBinaryPath(filePath);

    gAutoTaxi = std::make_unique<autotaxi::AutoTaxiSystem>(xplaneRoot, pluginRoot, logLine);
    gAutoTaxi->initialize();

    gUI = std::make_unique<autotaxi::AutoTaxiUI>(*gAutoTaxi);
    gUI->create();

    createMenuAndCommands();

    XPLMCreateFlightLoop_t params{};
    params.structSize = sizeof(params);
    params.phase = xplm_FlightLoop_Phase_BeforeFlightModel;
    params.callbackFunc = flightLoopCallback;
    params.refcon = nullptr;
    gFlightLoop = XPLMCreateFlightLoop(&params);
    XPLMScheduleFlightLoop(gFlightLoop, 0.25f, 1);

    logLine("[A350AutoTaxi] Plugin started. Root=" + pluginRoot);
    return 1;
}

A350_AUTOTAXI_PLUGIN_EXPORT void XPluginStop(void) {
    if (gAutoTaxi) {
        gAutoTaxi->stopByUser();
    }

    if (gFlightLoop) {
        XPLMDestroyFlightLoop(gFlightLoop);
        gFlightLoop = nullptr;
    }

    destroyMenuAndCommands();

    if (gUI) {
        gUI->destroy();
        gUI.reset();
    }

    gAutoTaxi.reset();
    logLine("[A350AutoTaxi] Plugin stopped.");
}

A350_AUTOTAXI_PLUGIN_EXPORT int XPluginEnable(void) {
    if (gAutoTaxi) {
        gAutoTaxi->initialize();
    }

    if (gFlightLoop) {
        XPLMScheduleFlightLoop(gFlightLoop, 0.25f, 1);
    }

    return 1;
}

A350_AUTOTAXI_PLUGIN_EXPORT void XPluginDisable(void) {
    if (gAutoTaxi) {
        gAutoTaxi->stopByUser();
    }

    if (gFlightLoop) {
        XPLMScheduleFlightLoop(gFlightLoop, 0.0f, 1);
    }
}

A350_AUTOTAXI_PLUGIN_EXPORT void XPluginReceiveMessage(XPLMPluginID inFromWho,
                                      int inMessage,
                                      void* inParam) {
    (void)inFromWho;
    (void)inParam;

    // Reload after aircraft changes so the A350 guard reads the new aircraft.
    // 102 = XPLM_MSG_PLANE_LOADED in current SDK headers; use the named macro when present.
#ifdef XPLM_MSG_PLANE_LOADED
    if (inMessage == XPLM_MSG_PLANE_LOADED && gAutoTaxi) {
        gAutoTaxi->reload();
    }
#else
    (void)inMessage;
#endif
}
