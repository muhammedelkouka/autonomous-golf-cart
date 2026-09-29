# System Requirements

Everything a Windows laptop needs to build and run the autonomous golf cart system.
Versions listed are the ones the system was developed and validated with.

---

## 1. Operating system

| Item | Requirement |
|---|---|
| OS | Windows 10 or 11, 64-bit |
| CPU | Multi-core recommended (particle-filter relocalisation is parallelised with OpenMP) |
| GPU | Optional. An NVIDIA GPU with CUDA (tested: GTX 1650) is needed only for the YOLO pedestrian detector |
| Network | Wi-Fi adapter, on the same network as the ESP32 |
| Ports | 1 × Ethernet (LiDAR), 1 × USB (IMU), 1–2 × USB (cameras, optional) |

---

## 2. Build toolchain (C++ application)

| Tool | Tested version | Notes |
|---|---|---|
| MinGW-w64 GCC (`g++`) | 13.2.0 (x86_64-ucrt-posix-seh) | Must support C++17 and OpenMP (`-fopenmp`). Installed with Strawberry Perl at `C:\Strawberry\c\bin` |
| CMake | 3.29.2 (minimum 3.15) | |
| GNU Make (`mingw32-make`) | 4.4.1 | Ships with MinGW |

Make sure `g++`, `cmake` and `mingw32-make` are on the system `PATH`.

**Already included in the repository (nothing to install):**
- Raylib 5.0 (`raylib-5.0_win64_mingw-w64/`): graphics and GUI
- EKF sensor-fusion code (`codegen/`): generated from MATLAB, MATLAB is **not** needed to build
- Windows system libraries used: `ws2_32`, `winmm`, `gdi32`, `iphlpapi`, `shell32` (part of Windows)

### Build

```
cmake -B build -G "MinGW Makefiles"
cmake --build build
```

The build copies `GolfCartControl.exe` and the MinGW runtime DLLs into `runtime_data/`.

### Run

Always start the program **from inside `runtime_data/`**. It loads maps, routes and
configuration files relative to that folder.

---

## 3. Python (pedestrian detection and analysis tools)

Python is needed only for the pedestrian-detection feature and the offline analysis scripts.
The main driving system runs without it.

| Item | Tested version |
|---|---|
| Python | **3.8.6**, 64-bit |

> **Important:** the GUI launches the pedestrian detector from this exact path, set in `main.cpp`:
> `C:\Users\bios\AppData\Local\Programs\Python\Python38\python.exe`
> On a different laptop or user account, either install Python 3.8 to the same path or update the path in `main.cpp`.

### Python packages

| Package | Tested version | Used for |
|---|---|---|
| `opencv-python` | **4.10.0.84** | camera capture, detection (do **not** use a 5.x preview build; it lacks required modules) |
| `numpy` | 1.24.4 | all tools |
| `ultralytics` | 8.4.91 | YOLOv8 pedestrian detector |
| `torch` | 1.13.1+cu116 | YOLO backend (GPU) |
| `torchvision` | 0.14.1+cu116 | YOLO backend |
| `pyserial` | 3.5 | IMU test scripts |
| `pandas` | 2.0.3 | log analysis |
| `matplotlib` | 3.7.5 | plots and figures |
| `pillow` | 10.4.0 | figure scripts |

Install:

```
python -m pip install opencv-python==4.10.0.84 numpy==1.24.4 pandas==2.0.3 matplotlib==3.7.5 pillow==10.4.0 pyserial==3.5 ultralytics
python -m pip install torch==1.13.1+cu116 torchvision==0.14.1+cu116 --extra-index-url https://download.pytorch.org/whl/cu116
```

For GPU detection, also install an **NVIDIA driver with CUDA 11.6 support**. Check with:

```
python -c "import torch; print(torch.cuda.is_available())"
```

Model files are included in the repository: `tools/yolov8n.pt` and `tools/models/` (MobileNet-SSD).

---

## 4. Hardware connections and drivers

| Device | Connection | Setting |
|---|---|---|
| Velodyne VLP-16 LiDAR | Ethernet | Receives UDP on port **2368**. Set the laptop's Ethernet adapter to a static IP on the LiDAR's subnet |
| Hiwonder IM10A IMU | USB-serial | **COM11**, 115200 baud, 100 Hz. Install the IMU's USB-serial (CH340) driver if Windows does not detect it |
| ESP32 controller | Wi-Fi (UDP) | ESP32 at **10.63.129.200**, commands on port **4210**, telemetry on port **4211** |
| USB cameras (optional) | USB | Camera index 0 and 1 (stereo pedestrian detection) |

The COM port, LiDAR port and ESP32 address are set in `common.hpp`. If the IMU appears on a
different COM port, or the ESP32 receives a different IP address, update `common.hpp` and rebuild.

### IMU pre-configuration (one-time)

The IMU must be configured with the **Hiwonder PC application** before use: enable
Acceleration + Angular velocity + Angle outputs at 100 Hz and save to flash. The system only
reads from the IMU; it does not configure it.

### Windows Firewall

Allow **inbound UDP on port 4211** (ESP32 telemetry) and **port 2368** (LiDAR) for
`GolfCartControl.exe`. Windows usually asks the first time the program runs; choose **Allow**.

---

## 5. ESP32 firmware (only needed to re-flash the controller)

| Item | Requirement |
|---|---|
| Arduino IDE | 2.x |
| Board package | **esp32 by Espressif Systems** (Boards Manager) |
| Board | ESP32 Dev Module (ESP32 DevKit V1) |
| USB driver | CP210x or CH340, depending on the DevKit's USB chip |
| Libraries | `WiFi`, `WiFiUdp`, `Wire`: all included with the ESP32 board package, nothing extra |

Firmware: `golf_cart_v28_smooth/golf_cart_v28_smooth.ino`. Set the Wi-Fi network name and
password in the sketch before flashing.

---

## 6. Optional software

| Tool | Purpose |
|---|---|
| Hiwonder PC application | IMU configuration and calibration |
| VeloView | viewing raw LiDAR data |
| MATLAB | only to regenerate the EKF code in `codegen/` (not needed to build or run) |
| VS Code | development environment |

---

## 7. Quick checklist

- [ ] MinGW-w64 GCC, CMake and Make installed and on `PATH`
- [ ] Project builds with `cmake --build build`
- [ ] Python 3.8 + packages installed (for pedestrian detection)
- [ ] LiDAR Ethernet adapter set to a static IP
- [ ] IMU detected on COM11 and pre-configured
- [ ] Laptop on the same Wi-Fi as the ESP32
- [ ] Firewall allows UDP 4211 and 2368
- [ ] Program started from `runtime_data/`

---

## 8. Running on Linux (recommendation)

The system is currently **Windows-only**. It does not build or run on Linux without code
changes. Porting is feasible and worthwhile: Linux gives better real-time scheduling for the
50 Hz control loop and is the standard platform for robotics (ROS 2, embedded computers such as
NVIDIA Jetson). The core algorithms (localisation, EKF, planning, Pure Pursuit) are standard
C++17 and need no changes. Only the operating-system layer does.

**Recommendation:** port on a separate git branch and validate each module on the bench
(LiDAR → IMU → ESP32 link → full drive) before any cart test, keeping the Windows version as
the working baseline.

### 8.1 Recommended Linux setup

| Item | Recommendation |
|---|---|
| Distribution | **Ubuntu 22.04 LTS** (64-bit) |
| Compiler | GCC 11+ (`build-essential`), C++17 + OpenMP |
| Build | CMake 3.15+, Make or Ninja |
| Graphics | Raylib 5.0 built for Linux (the bundled copy is Windows-only) |
| Python | Python 3.8–3.10 with the same packages as Section 3 |

Install:

```
sudo apt update
sudo apt install build-essential cmake git pkg-config \
     libasound2-dev libx11-dev libxrandr-dev libxi-dev libgl1-mesa-dev \
     libglu1-mesa-dev libxcursor-dev libxinerama-dev libwayland-dev libxkbcommon-dev \
     python3 python3-pip v4l-utils

# Raylib 5.0 from source
git clone --branch 5.0 --depth 1 https://github.com/raysan5/raylib.git
cd raylib && mkdir build && cd build
cmake -DBUILD_SHARED_LIBS=ON .. && make -j$(nproc) && sudo make install

# Serial-port access for the IMU (log out and back in afterwards)
sudo usermod -aG dialout $USER
```

Python packages: same as Section 3, using the PyTorch build that matches the installed CUDA version.

### 8.2 What needs to change in the code

| Area | File | Windows today | Linux replacement |
|---|---|---|---|
| Build config | `CMakeLists.txt` | Links `ws2_32`, `winmm`, `gdi32`, `iphlpapi`, `shell32`; bundled Windows Raylib; `-mwindows`; copies MinGW DLLs | Use `find_package(raylib)`; link `raylib m pthread dl GL`; drop `-mwindows` and the DLL-copy step |
| Networking (UDP) | `communication.cpp`, `localization.cpp`, `main.cpp`, `common.hpp` | Winsock (`winsock2.h`, `WSAStartup`, `closesocket`, `ioctlsocket`, `SOCKET`) | POSIX sockets (`sys/socket.h`, `close`, `fcntl(O_NONBLOCK)`, `int` socket). Remove the `#pragma comment(lib, …)` lines |
| IMU serial port | `localization.cpp`, `common.hpp` | `CreateFileA("\\\\.\\COM11")`, `ReadFile`, `CloseHandle` | `open("/dev/ttyUSB0")` + `termios` (115200 8N1, raw mode), `read`, `close` |
| High-resolution timer | `localization.cpp` | `QueryPerformanceCounter` / `QueryPerformanceFrequency` | `std::chrono::steady_clock` |
| Control-loop priority | `control.cpp` | `SetThreadPriority(… TIME_CRITICAL)` | `pthread_setschedparam` with `SCHED_FIFO` (needs root or `CAP_SYS_NICE`) |
| Pedestrian-detector launch | `main.cpp` | `CreateProcessA` / `TerminateProcess`, hard-coded Windows Python path, `..\\tools\\` path | `fork` + `execvp("python3", …)` / `kill(pid, SIGTERM)`, `../tools/` path |
| Open ESP32 web page | `ui.cpp` | `ShellExecuteA(…"open"…)` | `system("xdg-open http://…")` |
| GUI font | `main.cpp` | `C:/Windows/Fonts/segoeui.ttf` | A font shipped in `runtime_data/` (e.g. DejaVu Sans or Noto Sans) |
| Camera capture | `tools/pedestrian_detect.py` | `cv2.CAP_DSHOW` (DirectShow) | `cv2.CAP_V4L2` |
| Alert sound | `tools/pedestrian_detect.py`, stereo calibration scripts | `winsound.Beep` | Terminal bell, or run with `--no-beep` |
| Network setup | system | Windows Firewall | Allow UDP 2368 / 4211 in `ufw`, if enabled |

**Unchanged:** all localisation, EKF (`codegen/`), planning and control algorithms, all maps,
routes and configuration files, the ESP32 firmware, and the UDP message protocol.

**Suggested approach:** put the OS-specific parts (sockets, serial port, process launch, thread
priority) behind a small platform layer (`#ifdef _WIN32 … #else … #endif`). The same source then
builds on both Windows and Linux, and the validated Windows build stays intact.

### 8.3 Estimated effort

| Task | Effort |
|---|---|
| Build system + Raylib | ~0.5 day |
| Sockets, serial, timer, thread priority, process launch | ~1–2 days |
| Font, camera, small fixes | ~0.5 day |
| Bench validation of every module + one supervised cart test | ~2–3 days |
