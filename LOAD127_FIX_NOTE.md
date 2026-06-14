# Error Code 127 修复说明

X-Plane 日志中的：

```text
.../Resources/plugins/A350AutoTaxi/64/win.xpl : Error Code = 127 : 找不到指定的程序。
```

通常不是 `apt.dat` 问题，而是 Windows `LoadLibrary/GetProcAddress` 加载插件入口或依赖 DLL 失败。

本版做了两个修复：

1. `src/Plugin.cpp` 中的五个 X-Plane 插件入口改成显式 C 导出，避免 C++ 名字改编导致 X-Plane 找不到 `XPluginStart` 等入口。

2. `CMakeLists.txt` 对 MinGW 增加静态 GCC 运行库链接，减少 `libstdc++-6.dll` / `libgcc_s_seh-1.dll` / `libwinpthread-1.dll` 版本不匹配引发的 126/127。

## 重新编译步骤

在 CLion 中执行：

```text
Tools -> CMake -> Reset Cache and Reload Project
Build -> Rebuild Project
```

CMake options 保持：

```text
-DXPLANE_SDK_DIR=D:/XPlaneSDK
```

## 验证导出入口

Visual Studio Developer Command Prompt：

```bat
dumpbin /exports win.xpl | findstr XPlugin
```

MinGW：

```bat
objdump -p win.xpl | findstr XPlugin
```

应该能看到：

```text
XPluginStart
XPluginStop
XPluginEnable
XPluginDisable
XPluginReceiveMessage
```

如果这五个名字不是原样出现，X-Plane 就会加载失败。

## 最稳建议

如果 MinGW 仍然报链接或加载问题，CLion 的 Toolchain 改成 `Visual Studio 2022`，再重新 Reset Cache and Reload。X-Plane Windows SDK 的 `.lib` 用 MSVC 最稳。
