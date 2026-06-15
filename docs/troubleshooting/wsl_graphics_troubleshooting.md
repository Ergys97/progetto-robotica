# Troubleshooting WSL2 Graphics: Crash & Lag Resolution

This document explains how the MuJoCo simulator startup crashes and rendering performance lag (low FPS/high latency) were diagnosed and resolved inside the WSL2 (Ubuntu 24.04) environment on Windows.

---

## 1. Issue 1: Simulator Crash (`GLFWError: X11: The DISPLAY environment variable is missing`)

### Symptoms
When running the launch command:
```bash
ros2 launch progetto_robotica teleop_sim_launch.py
```
The node `mujoco_sim` immediately crashed with:
```text
[mujoco_sim-1] GLFWError: (65550) b'X11: The DISPLAY environment variable is missing'
[mujoco_sim-1] ERROR: could not initialize GLFW
[ERROR] [mujoco_sim-1]: process has died ...
```
And running `echo $DISPLAY` inside WSL returned an empty line.

### Diagnosis
WSL2 supports graphical interfaces natively using WSLg, but it was globally disabled on the Windows host. We inspected the global WSL configuration file on Windows at `C:\Users\ergys\.wslconfig` and found:
```ini
[wsl2]
guiApplications=false
```
The `guiApplications=false` flag tells WSL not to initialize the Wayland/X11 server (WSLg) and not to set up the `$DISPLAY` environment variable.

### Resolution
1. Opened the file `C:\Users\ergys\.wslconfig` and removed `guiApplications=false` (or set it to `true`).
2. Shut down the WSL virtual machine to apply the changes by running this command inside the WSL terminal (or PowerShell on Windows):
   ```bash
   wsl.exe --shutdown
   ```
3. Restarted WSL. The `$DISPLAY` environment variable was automatically set to `:0` and the X11 server became active.

---

## 2. Issue 2: Extreme Performance Lag & FPS Drop (Software Rendering LLVMpipe)

### Symptoms
After solving the crash, the simulator launched successfully but ran with very low FPS, severe rendering lag, and high latency, making teleoperation nearly impossible.

### Diagnosis
We ran the OpenGL diagnostics utility inside WSL:
```bash
sudo apt update && sudo apt install -y mesa-utils
glxinfo -B
```
The output revealed that the OpenGL renderer was falling back to the CPU (software rendering):
```text
OpenGL renderer string: llvmpipe (LLVM 20.1.2, 256 bits)
Accelerated: no
```
Mesa was not automatically linking with the physical GPU driver to leverage hardware acceleration.

### Resolution
For AMD Radeon GPUs (such as the RX 6800 in this system), Mesa provides GPU acceleration via the Direct3D 12 (D3D12) backend inside WSL. We can force Mesa to use this backend by setting the `GALLIUM_DRIVER` environment variable.

1. **Forced the D3D12 Gallium driver**:
   We tested forcing the driver in the environment:
   ```bash
   export GALLIUM_DRIVER=d3d12
   ```
2. **Verified hardware acceleration**:
   Re-running `glxinfo -B` confirmed that Mesa successfully bound to the GPU:
   ```text
   Device: D3D12 (AMD Radeon RX 6800) (0xffffffff)
   Accelerated: yes
   ```
3. **Made the fix permanent**:
   We appended the export to the user's `~/.bashrc` file:
   ```bash
   echo 'export GALLIUM_DRIVER=d3d12' >> ~/.bashrc
   source ~/.bashrc
   ```

Now, the MuJoCo simulation renders smoothly using the AMD Radeon RX 6800 GPU with zero lag.
