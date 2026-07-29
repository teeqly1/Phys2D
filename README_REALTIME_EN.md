# phys2d — patch: realtime scene

The archive contains only changed and new files. Unzip it over
`D:\projects\phys2d` with replacement.

| File | What's wrong with it |
|------|----------|
| `examples/realtime_scene.cpp` | NEW — interactive scene (Win32 + GDI, no dependencies) |
| `src/embedded_tables.S` | REPLACE — fixed build error under MinGW (COFF does not allow label differences in `.quad`) |
| `CMakeLists.txt` | REPLACE — added `phys2d_realtime` target (links with `gdi32 user32` on Windows) |

## Building and running (MSYS2 MinGW64)

```bash
cd /d/projects/phys2d
rm -rf build
cmake -B build -G "MinGW Makefiles" -DCMAKE_MAKE_PROGRAM=mingw32-make \
-DCMAKE_BUILD_TYPE=Release -DPHYS2D_NATIVE=ON -DPHYS2D_SIMD_AVX=ON \
-DPHYS2D_BIG_BINARY=ON -DPHYS2D_TABLE_MB=34
cmake --build build -j$(nproc)
./build/phys2d_realtime.exe
```

Fast, without CMake and built-in tables:

```bash
cd /d/projects/phys2d
g++ -std=c++17 -O3 -march=native -Iinclude \
examples/realtime_scene.cpp src/*.cpp -pthread -lgdi32 -luser32 \
-o realtime.exe
./realtime.exe
```

On non-Windows systems, the same file is built in headless mode: N seconds of
simulation run, profiler report, and `realtime_scene.json` (`./realtime 10`).

## Scene

A 40x30 arena with walls, two inclined ramps, circular and hexagonal obstacles,
three rotating kinematic gears, an 8-link chain with hinges,
a spring pendulum, and 240 dynamic bodies (circles, rectangles,
polygons, capsules). Fixed step size of 1/60 with time accumulator,
rendering via GDI with double buffering.

## Controls

| Key | Action |
|---------|----------|
| `SPACE` | Pause / Resume |
| `S` | Pause one step |
| `R` / `C` | Rebuild scene / Delete all dynamic bodies |
| `1` `2` `3` `4` | Create circle / rectangle / polygon / capsule under cursor |
| LMB | Grab and drag body (spring at grab point) |
| RMB | Explode at cursor point |
| Wheel / Arrow keys | Zoom / Pan |
| `G` | Gravity on/off |
| `I` | Toggle integrator (symplectic Euler / Verlet / RK4) |
| `A` `N` `T` | AABB / Contacts with normals / Trajectories |
| `M` `F` `E` `P` | multithreading / friction / recovery / positional correction |
| `[` `]` | fewer / more speed solver iterations |
| `,` `.` | fewer / more substeps |
| `J` `L` | save / load `scene.json` |
| `ESC` | exit |

The top left corner is the HUD: FPS, frame and step time, timings of all stages
(broad / narrow / velocity / position / constraints / sleeping), number of bodies, pairs,
contacts and points, GJK/EPA/SAT/TOI calls and axis cache hits, total
system energy, arena size, and module status.
