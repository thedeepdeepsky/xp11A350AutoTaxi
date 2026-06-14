# Scan result and GSX-style upgrade notes

## 原源码实际扫描什么 apt.dat

我检查了 `src/AutoTaxi.cpp`，原源码的 `collectAptDatCandidates()` 已经不是固定读一个 apt.dat，而是：

1. 先读 `A350AutoTaxi.ini` 的 `apt_dat_path=`；如果配置的是相对路径，则拼到 X-Plane 根目录。
2. 再读 `X-Plane 11/Custom Scenery/scenery_packs.ini`。
3. 对每个启用的 `SCENERY_PACK`，尝试加入：
   `该 scenery 包/Earth nav data/apt.dat`。
4. 最后加入：
   `X-Plane 11/Resources/default scenery/default apt dat/Earth nav data/apt.dat`。

原源码的问题：

- 没有显式 fallback `Custom Scenery/Global Airports/Earth nav data/apt.dat`，如果 scenery_packs.ini 不完整或未生成，可能漏掉 Global Airports。
- 多个 apt.dat 都包含同一机场时，原选择逻辑更偏“距离最近”，没有严谨按 scenery 优先级解决重复机场。
- UI 只有一个目的地下拉，没有目的地类型筛选、路线预览、路线 ETA、安全提示。
- 路线规划是 Dijkstra，缺少路线名摘要、急转弯/换道惩罚、偏航重算、安全启动条件。
- 当前位置只显示最近 Gate/Ramp 或 node，不能显示最近 taxiway edge。

## 已升级内容

- 增加 scenery priority 选择逻辑：相同/近似机场优先选更高优先级的 apt.dat。
- 增加 Global Airports 显式 fallback。
- 增加 `a350_autotaxi/plan` 命令。
- UI 改为 GSX-style 面板：目的地类型筛选、下拉目的地、路线距离、ETA、路径摘要、安全提示、进度条、最近 taxiway edge。
- Dijkstra 改为 A* 图搜索，并加入 runway/active-zone/turn penalty。
- 1206 ground vehicle only 路线继续排除，防止飞机走地勤车道。
- 跑道目的地优先映射到 1204 active zone / hold-short 附近节点。
- Gate/Ramp 读取 1301 width code，宽度小于 E 的停机位会提示 small for A350。
- 自动滑行控制加入 lookahead、急转弯减速、启动安全检查、偏离路线自动重算。
- Refresh 时若正在控制飞机，会先释放控制，避免残留油门/转向/刹车输出。

## 仍无法做到的“真 GSX”部分

- 不控制地勤车辆、登机桥、乘客、货物动画。
- 不读取在线交通冲突或 X-Plane ATC 放行。
- 不在 3D 地面绘制可见发光路线。

这版目标是 GSX-style 的机场地面路线交互与自动滑行控制，而不是完整复刻 GSX 地勤生态。
