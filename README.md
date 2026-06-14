# A350AutoTaxi - GSX-style Ground Routing for X-Plane 11.50

这是一个面向 **Windows + CLion + X-Plane 11.50 Plugin SDK** 的 C++ 插件源工程。新版从“简单 AutoTaxi 原型”升级为 **GSX-style ground routing**：自动识别当前机场、读取 scenery 优先级下的 apt.dat、识别机场内部位置、下拉选择跑道/Gate/Ramp、预览路线、再启动自动滑行。

> 说明：它不是 FSDreamTeam GSX，也不接管 ATC、登机桥、地勤车辆或在线交通避让；这里的“GSX-style”指交互流程、机场地面路线识别、目的地下拉、路线预览和安全状态面板。

## 1. 新版核心功能

### apt.dat 扫描与机场识别

插件启动后通过 X-Plane 根目录自动扫描：

```text
1. A350AutoTaxi.ini 中 apt_dat_path 指定的文件，如果设置
2. Custom Scenery/scenery_packs.ini 中启用的 SCENERY_PACK，按文件顺序逐个查找 Earth nav data/apt.dat
3. Custom Scenery/Global Airports/Earth nav data/apt.dat
4. Resources/default scenery/default apt dat/Earth nav data/apt.dat
```

如果 `icao=` 为空，插件用飞机当前位置搜索最近的可用机场；如果多个 apt.dat 都有相同或近似机场，新版会用 **scenery priority** 选优先级更高的机场，而不是只比较机场中心距离。

### 解析的 apt.dat 行

```text
100   runway
1201  taxi route network node
1202  taxi route network edge
1204  runway active zone / hold-short metadata
1206  ground-vehicle-only edge，自动排除在飞机滑行路线外
1300  ramp start / gate / stand
1301  ramp width、operation、airline metadata
1302  airport metadata / ICAO
```

### UI 面板

菜单：

```text
Plugins -> A350 AutoTaxi -> Open AutoTaxi Panel
```

面板包含：

```text
Status              STANDBY / ACTIVE / ARRIVED / DATAREF WARN
Airport             当前识别机场
apt.dat             候选文件数量、优先级、来源文件
Current position    Gate/Ramp、Taxiway edge 或 Taxi node
Nearest edge        当前最近滑行线/跑道边及距离
Destination type    ALL / RUNWAY / GATE-RAMP / GPS-NODE
Destination         下拉选择跑道、Gate、Ramp、坐标
Plan Route          先规划并预览路线
Start AutoTaxi      启动自动滑行
Stop / Release      停止并释放控制
```

### 路线规划

新版使用 A* 图搜索，并在成本里加入：

```text
runway_penalty_m         默认 5000 m，避免误走跑道
active_zone_penalty_m    默认 2000 m，谨慎进入跑道保护区/hold-short 附近
groundVehicleOnly        1206 地勤车辆路线不用于飞机路径
turn_penalty_m           换滑行道/急转弯加入小惩罚，路线更像地面引导
```

跑道目的地会优先通过 `1204` active zone 找对应跑道端附近的 hold-short 节点；找不到才退化为跑道阈值附近最近 taxi node。

### A350 滑行控制改进

```text
lookahead_distance_m         前视目标点，减少左右摆动
turn_slowdown_threshold_deg  大转弯自动减速
sharp_turn_speed_kts         急转弯目标速度
auto_replan_if_off_route     偏离路线后自动重算
off_route_replan_distance_m  偏离阈值
max_start_node_distance_m    离 taxi network 太远拒绝启动
max_start_speed_kts          飞机已经移动太快则拒绝启动
```

## 2. Windows + CLion 编译

准备官方 X-Plane Plugin SDK，例如：

```text
D:/XPlaneSDK/
  CHeaders/XPLM/
  CHeaders/Wrappers/
  Libraries/Win/XPLM_64.lib
```

CLion 打开工程根目录 `A350AutoTaxi`，CMake Options 填：

```text
-DXPLANE_SDK_DIR="D:/XPlaneSDK"
```

编译后输出：

```text
cmake-build-*/A350AutoTaxi/
  A350AutoTaxi.ini
  64/
    win.xpl
```

复制整个文件夹到：

```text
X-Plane 11/Resources/plugins/
```

最终结构：

```text
X-Plane 11/Resources/plugins/A350AutoTaxi/
  A350AutoTaxi.ini
  64/
    win.xpl
```

## 3. 插件命令

```text
a350_autotaxi/panel     打开/关闭 UI 面板
a350_autotaxi/plan      规划并预览当前下拉目的地路线
a350_autotaxi/toggle    启动/停止 AutoTaxi
a350_autotaxi/reload    重新读取 ini 和 apt.dat
```

## 4. A350 机模兼容

默认写入：

```text
steer_dataref=sim/cockpit2/controls/yoke_heading_ratio
throttle_all_dataref=sim/cockpit2/engine/actuators/throttle_ratio_all
left_brake_dataref=sim/flightmodel/controls/l_brake_add
right_brake_dataref=sim/flightmodel/controls/r_brake_add
parking_brake_dataref=sim/cockpit2/controls/parking_brake_ratio
```

FlightFactor A350 或其他深度自定义 A350 可能拦截默认 dataref。面板若显示 `DATAREF WARN`，请用 DataRefTool 找到该机模可写的转向、刹车、油门 dataref，然后改 `A350AutoTaxi.ini`。

测试非 A350 机型时：

```text
allow_any_aircraft=true
```

## 5. 使用建议

1. 先进机场并停在 Gate/Ramp 或 taxiway 上。
2. 打开面板，点 `Refresh`。
3. 用 `Destination type` 过滤目的地类型。
4. 下拉选 `Runway xx` 或 `Gate/Ramp`。
5. 点 `Plan Route` 看路线、距离、ETA、安全提示。
6. 确认 dataref 正常后点 `Start AutoTaxi`。

首次测试必须低速、外部视角观察，随时准备手动刹车或点 `Stop / Release`。

## 6. 当前限制

- 不读取实时 ATC 放行、AI/在线交通冲突、地勤车辆位置。
- 第三方机场如果没有完整 `1201/1202/1204`，跑道目的地可能只能映射到最近 taxi node。
- 路线显示在插件面板中，不在 3D 地面绘制发光路线。
- Windows 版需要你本机用官方 SDK 编译生成 `win.xpl`。
