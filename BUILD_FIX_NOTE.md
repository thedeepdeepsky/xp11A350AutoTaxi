# Build fix: XPLMDrawString const char* error

Fixed `src/AutoTaxiUI.cpp`:

- Old code passed `text.c_str()` directly to `XPLMDrawString`.
- X-Plane 11 SDK declares `XPLMDrawString(..., char* inChar, ...)`, while `std::string::c_str()` is `const char*`.
- MinGW/G++ correctly rejects the implicit conversion.
- New code creates a mutable, null-terminated `std::vector<char>` buffer before calling `XPLMDrawString`.

After replacing this source package, in CLion run:

1. Tools -> CMake -> Reset Cache and Reload Project
2. Build -> Rebuild Project

Keep CMake option similar to:

```text
-DXPLANE_SDK_DIR=D:/XPlaneSDK
```

If MinGW later fails during linking against `XPLM_64.lib`, switch CLion Toolchain to Visual Studio 2022. The source code itself is C++17-compatible.
