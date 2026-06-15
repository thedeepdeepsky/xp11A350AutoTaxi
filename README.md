# A350AutoTaxi - GSX-style Ground Routing for X-Plane 11.50

This is a **C++ plugin source project** for **Windows + CLion + the X-Plane 11.50 Plugin SDK**. The new version upgrades the original simple AutoTaxi prototype into a **GSX-style ground routing system**: it automatically detects the current airport, reads `apt.dat` files based on scenery priority, identifies internal airport positions, provides dropdown selection for runways/Gates/Ramps, previews routes, and then starts automated taxiing.

> Note: This is not FSDreamTeam GSX, and it does not control ATC, jet bridges, ground service vehicles, or online traffic avoidance. The term “GSX-style” here refers to the interaction flow, airport ground route identification, destination selection UI, route preview, and safety status panel.

---

## 1. Core Features of the New Version

### apt.dat Scanning and Airport Detection

On startup, the plugin automatically scans the X-Plane directory:

```text
1. A350AutoTaxi.ini apt_dat_path (if specified)
2. All enabled SCENERY_PACK entries in Custom Scenery/scenery_packs.ini, in order, searching for Earth nav data/apt.dat
3. Custom Scenery/Global Airports/Earth nav data/apt.dat
4. Resources/default scenery/default apt.dat/Earth nav data/apt.dat
```

If `icao=` is empty, the plugin searches for the nearest airport based on aircraft position. If multiple `apt.dat` files contain the same or similar airports, the new version uses **scenery priority** to select the highest-priority airport instead of relying only on distance to the airport center.

---

### Parsed apt.dat Elements

```text
100   runway
1201  taxi route network node
1202  taxi route network edge
1204  runway active zone / hold-short metadata
1206  ground-vehicle-only edge, excluded from aircraft taxi routing
1300  ramp start / gate / stand
1301  ramp width, operations, airline metadata
1302  airport metadata / ICAO
```

---

### UI Panel

Menu path:

```text
Plugins -> A350 AutoTaxi -> Open AutoTaxi Panel
```

The panel includes:

```text
Status              STANDBY / ACTIVE / ARRIVED / DATAREF WARN
Airport             detected airport
apt.dat             number of candidate files, priority, source file
Current position    gate/ramp, taxiway edge, or taxi node
Nearest edge        nearest taxi route/runway edge and distance
Destination type    ALL / RUNWAY / GATE-RAMP / GPS-NODE
Destination         dropdown: runway, gate, ramp, coordinates
Plan Route          generate and preview route first
Start AutoTaxi      start automatic taxi
Stop / Release      stop and release control
```

---

### Route Planning

The system uses **A* graph search**, with cost penalties including:

```text
runway_penalty_m         default 5000 m, avoids unintended runway use
active_zone_penalty_m    default 2000 m, avoids runway protection / hold-short areas
groundVehicleOnly        1206 ground-vehicle-only edges are excluded from aircraft routing
turn_penalty_m           small penalty for sharp turns and junction changes for more realistic routing
```

For runway destinations, the system prioritizes using `1204` active zones to find hold-short nodes near runway ends. If not found, it falls back to the nearest taxi node near the runway threshold.

---

### A350 Taxi Control Improvements

```text
lookahead_distance_m         forward target point to reduce oscillation
turn_slowdown_threshold_deg  automatic slowdown for large turns
sharp_turn_speed_kts         target speed for tight turns
auto_replan_if_off_route     automatically recompute route if deviating
off_route_replan_distance_m  deviation threshold
max_start_node_distance_m    reject start if too far from taxi network
max_start_speed_kts         reject start if aircraft is moving too fast
```

---

## 2. Windows + CLion Build Instructions

Prepare the official X-Plane Plugin SDK, for example:

```text
D:/XPlaneSDK/
  CHeaders/XPLM/
  CHeaders/Wrappers/
  Libraries/Win/XPLM_64.lib
```

Open the project root `A350AutoTaxi` in CLion, and set CMake options:

```text
-DXPLANE_SDK_DIR="D:/XPlaneSDK"
```

After building, the output will be:

```text
cmake-build-*/A350AutoTaxi/
  A350AutoTaxi.ini
  64/
    win.xpl
```

Copy the entire folder into:

```text
X-Plane 11/Resources/plugins/
```

Final structure:

```text
X-Plane 11/Resources/plugins/A350AutoTaxi/
  A350AutoTaxi.ini
  64/
    win.xpl
```

---

## 3. Plugin Commands

```text
a350_autotaxi/panel     open/close UI panel
a350_autotaxi/plan      plan and preview route to selected destination
a350_autotaxi/toggle    start/stop AutoTaxi
a350_autotaxi/reload    reload ini and apt.dat files
```

---

## 4. A350 Aircraft Compatibility

Default datarefs:

```text
steer_dataref=sim/cockpit2/controls/yoke_heading_ratio
throttle_all_dataref=sim/cockpit2/engine/actuators/throttle_ratio_all
left_brake_dataref=sim/flightmodel/controls/l_brake_add
right_brake_dataref=sim/flightmodel/controls/r_brake_add
parking_brake_dataref=sim/cockpit2/controls/parking_brake_ratio
```

FlightFactor A350 or other heavily customized A350 aircraft may override default datarefs. If the panel shows `DATAREF WARN`, use DataRefTool to locate writable steering, brake, and throttle datarefs for that aircraft, then update `A350AutoTaxi.ini`.

For testing non-A350 aircraft:

```text
allow_any_aircraft=true
```

---

## 5. Recommended Usage

1. Enter an airport and park at a Gate/Ramp or taxiway.
2. Open the panel and click `Refresh`.
3. Use `Destination type` to filter destination categories.
4. Select a destination such as `Runway xx` or `Gate/Ramp`.
5. Click `Plan Route` to view route, distance, ETA, and safety info.
6. Once datarefs are confirmed working, click `Start AutoTaxi`.

For first-time testing, use low speed and external camera view. Always be ready to manually brake or press `Stop / Release`.

---

## 6. Current Limitations

* No awareness of real-time ATC clearances, AI/online traffic conflicts, or ground vehicle positions.
* Third-party airports without complete `1201/1202/1204` data may only allow runway destinations to map to the nearest taxi node.
* Route visualization is only shown in the plugin UI; no 3D ground rendering of paths.
* Windows build requires local compilation using the official SDK to generate `win.xpl`.
