# Connected & Autonomous Vehicle Controller and Multi-aspect Evaluation

[![Python](https://img.shields.io/badge/python-3.x-blue.svg)](https://www.python.org/downloads/)
[![SUMO](https://img.shields.io/badge/SUMO-microsimulation-orange.svg)](https://www.eclipse.org/sumo/)
[![License](https://img.shields.io/badge/license-See%20LICENSE-green.svg)](LICENSE)

A collection of calibrated SUMO scenarios and connected/autonomous vehicle (CAV) controllers for **microsimulation of mixed traffic**.  
The repository uses the **TraCI API** and optional **compiled controllers** (codegen + C/C++) to directly command CAV longitudinal behavior in simulation.

On top of the controllers and scenarios, the project provides:

- A **multi-aspect analysis pipeline** (`analysis.py`) that evaluates PCC/CAV penetration in terms of safety, mobility, fuel consumption and driving behavior.
- An interactive, publication-oriented **dashboard** (`dashboard.py`) that visualizes these metrics and exports plot-ready datasets and time–space diagrams.


## Overview

`cav_pcc_evaluation` provides a reusable environment for exploring **CAV penetration**, **longitudinal controllers**, and **mixed traffic dynamics** in SUMO.

The workflow is:

1. Run SUMO with PCC controllers at different penetration levels.
2. Use `analysis.py` to build a **multi-aspect evaluation** of each experiment:
   - safety, mobility, environmental impact, behavioral indicators, macroscopic wave patterns, and microsimulation quality control.
3. Use `dashboard.py` to **visualize and export** these metrics in a Dash/Plotly web app:
   - interactive tabs (Safety, Mobility, Driving Behavioral, Fuel Consumption),
   - CSV/XLSX/JSON exports for offline analysis.

The goal is to make it easy to (i) deploy and swap CAV controllers in SUMO, (ii) run controlled experiments across scenarios and penetration levels, and (iii) inspect the resulting traffic and controller performance in a systematic, reproducible way.

---

## Key Features

- **CAV Longitudinal Control via TraCI**
  - Python-based TraCI control loop with direct access to vehicle states.
  - Example implementation of a predictive cruise controller (PCC).

- **Compiled Controller Support (Codegen)**
  - Optional C/C++-compiled controllers with shared libraries (`.so` / `.dll`).
  - `cmake`-based build scripts and wrappers (`cwrapper.py`, `cppwrapper.py`).

- **Multi-Aspect Post-Simulation Analysis**
  - `analysis.py` ingests SUMO outputs (FCD, stats, lane/edge aggregates, detectors/TLS) and builds:
    - **Safety** metrics (TTC, PET, DRAC, headways).
    - **Mobility** metrics (throughput, delay, speeds, acceleration, queues).
    - **Fuel Consumption** metrics (fuel consumption, fuel efficiency).
    - **Driving Behavior** metrics (following gaps, lane changes, gap acceptance).
  - Optional **urban-signal** analytics: PAoG, GOR, TTS, spillback detection.

- **Interactive, Publication-Ready Dashboard**
  - `dashboard.py` wraps the analysis results in a Dash/Plotly web app:
    - Tabs for Safety, Mobility, Driving Behavior, Fuel Consumption.
    - Time–space viewer and exporters (CSV) for detailed lane-level visualization.
    - XLSX/CSV/JSON exports of plot-ready datasets and spillback events.

- **Cross-Platform Setup**
  - Shell-based installer for Linux (and Git Bash on Windows).
  - Batch installer for Windows Developer Command Prompt.
  - Manual install path for custom Python/SUMO environments.

- **Simulation & Analysis Pipeline**
  - `main.py` for SUMO mixed traffic runs.
  - `analysis.py` for aggregating, caching, and exporting metrics.
  - Scenario-based organization of inputs and outputs for reproducibility.
---

##  Installation

### Prerequisites

- **Python** 3.x
- **SUMO** (installed via this repo or system-wide)
- **Git** (to clone this repository)
- **cmake** (required if you want to compile codegen controllers)
- On Windows: **Visual Studio C/C++ build tools** + **Developer Command Prompt**

---

### Option 1: Linux / macOS (Recommended)

From the repository root (`cav_pcc_evaluation/`):

```bash
# Clone the repository
git clone <repository-url>
cd cav_pcc_evaluation

# Run installer (Python + SUMO + optional controllers)
. install.sh
```

What this does:

- Checks for a valid Python + `pip`.
- Installs SUMO and required Python packages (globally or in your chosen env).
- Optionally compiles and installs CAV controllers, placing shared libraries in `cav_pcc_evaluation/` and copying wrappers to `src/`.

>  You can edit `install.sh` to enable virtual environments, e.g. `USING_VENV=1`.

To run only the controller installers:

```bash
cd codegen
. install.sh
```

---

### Option 2: Windows (Developer Command Prompt / Git Bash)

**Approach A — Git Bash (Unix-like)**

1. Install:
   - Git (with Git Bash)
   - Visual Studio C/C++ tools
   - `cmake`
2. Open **Git Bash**
3. Follow the same steps as Linux:

```bash
git clone <repository-url>
cd cav_pcc_evaluation
. install.sh
```

**Approach B — Windows Developer Command Prompt**

From the **Developer Command Prompt for VS**:

```bat
git clone <repository-url>
cd cav_pcc_evaluation

rem Run Windows installer
install.bat
```

This will:

- Install SUMO dependencies.
- Attempt to compile the CAV controllers using `cmake`.
- Copy the shared libraries to the `cav_pcc_evaluation/` root.
- Copy `cwrapper.py` to `src/`.

To run only the controller installers (without SUMO install):

```bat
cd codegen
install.bat
```

---

### Option 3: Manual Python & Controller Installation

If you prefer manual setup:

```bash
# Python packages
pip install eclipse-sumo
pip install traci
pip install libsumo
```

To build the PCC controller (requires `cmake`):

```bash
cd codegen

# Choose build type
BUILD_TYPE=Release  # Options: Debug, RelWithDebInfo, Release

cd pcc_codegen
mkdir build

cmake -B build -DCMAKE_BUILD_TYPE=$BUILD_TYPE
cmake --build build --config $BUILD_TYPE
cmake --install build --config $BUILD_TYPE

rm -r build

python test.py
```

If the tests pass, copy shared libraries and wrappers into the main repo:

```bash
cp lib/pcc_so.dll ../../  # or .so on Linux
cp _cppwrapper.py ../../src/cppwrapper.py

cd ..
```

---

## Quick Start

### 1. Run Mixed Traffic Simulation

From the `cav_pcc_evaluation/` root:

```bash
python main.py
```

Check available options:

```bash
python main.py -h
```

Typical options include:

- Selecting the **scenario** to run.
- Setting **CAV penetration rate**.
- Choosing output folders / run IDs.

>  `main.py` will run SUMO using a TraCI control loop, calling your CAV controller each step.

---


### 2. Run Simulation Analysis

After simulations are complete:

```bash
python analysis.py
python analysis.py -h   # View options
```

You can configure:

- Which output file/folder to read.
- Whether to run in freeway or urban mode.
- Where to write processed metrics and exports.

---

### 3. Launch the Dashboard

After `analysis.py` has created metrics and (optionally) caches:

```bash
# Example: On-ramp freeway scenario
python dashboard.py --scenario_folder sumo_scenarios --scenario onramp --urban no --p 0.1
```

Then open your browser at:

- http://localhost:8050

More advanced examples (lane filtering):

```bash
# Onramp example
python dashboard.py --scenario onramp --urban no     --exclude-lane ramp_0 --exclude-lane E2_0 --p 0.1

# I-24 example
python dashboard.py --scenario i24 --urban no     --exclude-lane E2_0 --exclude-lane E4_0     --exclude-lane E6_0 --exclude-lane E1_0 --p 0.1
```

Useful options (see `dashboard.py -h`):

- `--file` to open a specific FCD file.
- `--exclude-lane`, `--exclude-lane-prefix`, `--lane-pos-window` to control which samples are included.
- `--urban` to control Urban Signals integration (`auto` / `yes` / `no`).

---

## 📁 Project Structure

A high-level view of the repository (simplified):

```text
cav_pcc_evaluation/
├── src/                   # Python source (agents, controllers, utilities)
│   ├── agents.py          # CAV controller bindings (PCC, etc.)
│   └── ...
│
├── codegen/               # Compiled controller build scripts & sources
│   ├── install.sh
│   └── pcc_codegen/
│       ├── CMakeLists.txt
│       └── ...
│
├── sumo_scenarios/        # SUMO networks, routes, and configs per scenario
│   ├── onramp/
│   ├── i24/
│   └── ...
│
├── output/                # Simulation outputs (per-scenario subfolders)
│   └── .../excel_file/    # Analysis exports used by dashboard
│
├── dashboard.py           # Interactive Dash/Plotly visualization
├── main.py                # Main SUMO mixed-traffic simulation entrypoint
├── main_xil.py            # XIL (server-side) entrypoint
├── analysis.py            # Post-processing & export script
├── install.sh             # Linux / Git Bash installer
├── install.bat            # Windows installer
├── README.md              # Project documentation (this file)
└── LICENSE                # License file
```

> Note: Exact structure may vary slightly by branch or use case.

---

## Scenarios & Controllers

### Scenarios

- **Freeway on-ramp** (`onramp`)
- **I-24 corridor** (`i24`)
- Additional scenarios can be added under `sumo_scenarios/<scenario_name>/` with their own:
  - Network (`.net.xml`)
  - Routes (`.rou.xml`)
  - Configuration and detector definitions.

### Controllers

- **PCC (Probabilistic Cruise Controller)**:
  - Longitudinal CAV controller compiled via `codegen/pcc_codegen`.
  - Invoked from Python using shared libraries + wrappers.

- **TraCI Loop Example** (simplified):

```python
while traci.simulation.getMinExpectedNumber() > 0:
    traci.simulationStep()

    sim_time = traci.simulation.getTime()
    vehicle_ids = traci.vehicle.getIDList()

    cav_ids = (vid for vid in vehicle_ids if traci.vehicle.getTypeID(vid) == "cav")
    for ego_id in cav_ids:
        ego_speed = traci.vehicle.getSpeed(ego_id)
        ego_accel = traci.vehicle.getAcceleration(ego_id)
        ego_distance = traci.vehicle.getDistance(ego_id)

        leader_info = traci.vehicle.getLeader(ego_id)
        if leader_info is not None and leader_info[0] != "":
            lead_id = leader_info[0]
            lead_rel_distance = leader_info[1]
            lead_speed = traci.vehicle.getSpeed(lead_id)
            lead_accel = traci.vehicle.getAcceleration(lead_id)
        else:
            lead_rel_distance = 2000.0
            lead_speed = 0.0
            lead_accel = 0.0

        desired_accel = getCommand(
            sim_time, ego_accel, ego_speed, ego_distance,
            s_max, v_max, lead_accel, lead_speed, lead_rel_distance
        )

        traci.vehicle.setAcceleration(ego_id, desired_accel, acc_duration=2.0)
```

This pattern can be reused for custom CAV controllers.

---

##  Analysis: Multi-Aspect Evaluation

`analysis.py` is the main **post-simulation evaluation engine**.  
It takes raw SUMO outputs (FCD, statistics, edge/lane aggregates, and optionally detector/TLS data) and builds a **multi-aspect evaluation** of PCC and other controllers under different CAV penetration levels.

### Inputs

Typical input files from SUMO include:

- **FCD (floating car data)**, e.g. `fcd.xml`  
  Per-timestep, per-vehicle states with leader information (gap, speed, acceleration).

- **Simulation statistics**, e.g. `stats.xml`  
  Real-time factor, number of loaded/inserted/running/teleported vehicles, emergency braking, etc.

- **Edge / lane aggregates**, e.g. `by_edge.xml`, `by_lane.xml`  
  For macroscopic and quality-control measures.

- **(Optional: Urban mode)** Detector and signal data  
  TLS time series, static `tlLogic`, detector outputs, and lane mapping (`add.xml`).

### Metric families

The script organizes metrics into several dimensions:

1. **Safety**
   - TTC (Time-to-Collision) distribution.
   - PET (Post-Encroachment Time) distribution.
   - Time headway distribution (HDV and CAV).
   - DRAC (Deceleration Rate to Avoid a Crash).

2. **Mobility**
   - Throughput vs. penetration rate.
   - Total delay and average speed.
   - Lane-/edge-specific travel times and level of service.
   - Queue length and stop time at bottlenecks / intersections.

3. **Fuel Consumption**
   - Emissions (per vehicle / scenario).
   - Fuel consumption rate based on a physics-inspired traction-power model:
     traction power + auxiliaries, idle floor, BSFC-based fuel-rate conversion (g/s), with stable behavior at low speeds.

4. **Driving Behavior**
   - Overtaking / lane-change rate and duration.
   - HDV/CAV following gap and headway distributions.
   - Gap acceptance at merges / lane changes.
   - (Planned) reaction time and other microscopic behavior indicators.

5. **Macroscopic characteristics**
   - Shockwave propagation and stop-and-go wave intensity along the corridor.
   - Aggregate speed-flow-density patterns.

### Urban signal analytics (`urban_mode`)

When `urban_mode` is enabled, `analysis.py` adds a full **signalized-corridor** pipeline:

- Detector → lane → TLS mapping from detector outputs, `tlLogic`, and lane mapping files.
- Urban metrics:
  - **PAoG** (Percent of Arrivals on Green).
  - **GOR** (Green Occupancy Ratio), both overall and by movement type.
  - Intersection approach delay and queue spillback detection.
  - **Time-To-Service (TTS)** for queued vehicles.

This allows PCC impacts to be evaluated consistently in both freeway and urban signalized environments.

### Caching and robustness

To support fast iteration and stable metrics, `analysis.py`:

- Uses `load_or_build_metrics(file_dir, urban_mode)` as a **cache loader**:
  - Computes a signature of all relevant inputs (e.g., mtime + size).
  - Reuses cached results (e.g., `metrics_p0.1t.pkl`) when inputs have not changed.
  - Automatically rebuilds metrics when inputs differ.

- Applies **gating, limits, and sanitization**:
  - Spatial gates (`POS_MIN_ANALYSIS`, `TT_USE_POS_GATE`, `ENERGY_USE_POS_GATE`).
  - Physical bounds on speed/acceleration (`V_MAX`, `ACC_MIN`, `ACC_MAX`).
  - Visualization bounds for histograms (`SPEED_MIN_HIST`, `SPEED_MAX_HIST`, `ACC_MAX_HIST`).
  - Cut-offs for TTC/PET/headway/space-gap to remove numerical artifacts.

In short, `analysis.py` turns raw SUMO logs into a consistent, publication-ready **multi-aspect evaluation** of PCC under different CAV penetration levels.

---

## Dashboard

`dashboard.py` is an interactive **front-end** on top of `analysis.py`.  
It reads the processed metrics and exports from the analysis pipeline and exposes them through a Dash/Plotly web app.

### What the dashboard provides

- Multiple analysis tabs:
  - **Safety** – TTC/PET/headways/DRAC distributions and related indicators.
  - **Mobility** – throughput, delay, speeds, accelerations, travel times.
  - **Behavioral** – space gap, lane-change frequency, gap acceptance.
  - **Fuel Consumption** – fuel efficiency, per-trip fuel consumption, avergage fuel consumption.
  - **Time–Space** (disable) – time–space diagrams with CAV penetration overlays and export tools.
  - **Urban Signals** (optional) – PAoG, GOR, approach delay, TTS, and spillback charts.

- Interactive filters:
  - Scenario selection and CAV penetration.
  - Direction, lanes, lane prefixes, and position windows.
  - Warm-up trimming and queue-clearing thresholds (aligned with `analysis.py`).


### Relationship to `analysis.py`

The dashboard is intentionally **thin** on heavy computation:

- It calls `load_or_build_metrics(...)` from `analysis.py` to either:
  - Reuse cached metrics when SUMO outputs are unchanged, or
  - Rebuild metrics when inputs changed.
- This keeps the dashboard responsive even for large networks and long simulations.
- All thresholds, gates, and definitions are shared with the analysis code, so the plots match the numerical results used in papers.

### Time–Space viewer and exporters

The **Time–Space** tab is built as a combined viewer and exporter:

- Hooks into an internal visualizer to render time–space diagrams.
- Provides a robust CSV exporter (e.g., `export_time_space_csv(...)`) with options for:
  - Warm-up removal and position gating.
  - Queue-clearing rules (`CLEAR_V`, `CLEAR_M`).
  - Jump detection and temporal/vehicle stride.
  - Optional fields like speed, plus gzip compression for large files.

These exports are intended to support downstream analysis or figure generation outside the dashboard (e.g., in notebooks).

### Reproducible figure exports

To support paper-quality figures and post-processing, the dashboard also exposes export helpers, such as:

- `export_dashboard_datasets(...)`:
  - Writes compact, plot-ready tables to **XLSX** (multi-sheet) or CSV.
  - Includes time-series data and histogram bins, so plots can be recreated offline.

- `export_spillback(...)`:
  - Saves detected spillback events to **CSV/JSON**.
  - Prints a short console summary for quick checks.

### Usage recap

Typical usage pattern:

1. Run SUMO with PCC enabled to produce raw outputs (`fcd.xml`, `stats.xml`, etc.).
2. Run `analysis.py` (optionally in `urban_mode`) to build metrics and caches.
3. Launch the dashboard and point it to the scenario folder:

```bash
# Freeway onramp example (no urban signals)
python dashboard.py --scenario_folder sumo_scenarios     --scenario onramp --urban no --exclude-lane ramp_0 --exclude-lane E2_0 --p 0.1

# I-24 example
python dashboard.py --scenario i24 --urban no     --exclude-lane E2_0 --exclude-lane E4_0     --exclude-lane E6_0 --exclude-lane E1_0 --p 0.1
```

The dashboard then loads (or rebuilds) the metrics via `analysis.py` and exposes them in an interactive, publication-style web UI.

---

##  XIL-Based Simulation

`main_xil.py` turns SUMO into a real-time server that can be driven by external vehicle simulators or hardware-in-the-loop components.

Key capabilities:

- Configure:
  - Number of clients.
  - Network endpoint (IP/port).
  - Scenario and CAV penetration.
- Receive and apply external control messages for selected vehicles.
- Maintain synchronized time with external clients.

`ext/vehicle_sim` provides a simple Python client you can use as:

```bash
python -m ext.vehicle_sim
```

Use the help flag to see options:

```bash
python -m ext.vehicle_sim -h
```

---

## Debugging / FAQ

### 1. Shared library `<cav_controller>` not found

> _“The `<cav_controller>` shared library file cannot be found, but I can see it in my directory.”_

- Check that the installation path in the **PCC** class inside `src/agents.py` points to the correct shared library location.
- If the library was built on another machine or OS, you may need to **rebuild** it:
  - Follow the compiled controller install steps in [Installation](#installation).
  - Ensure `cmake` and your compiler toolchain are correctly installed.

### 2. Dashboard shows empty plots or fails to load

- Confirm that `analysis.py` has been run and that:
  - `<scenario>/excel_file` (or equivalent exports) exist.
  - The expected CSV/Excel files are present.
- Double-check that `--scenario` and `--scenario_folder` match your directory names.

### 3. SUMO / TraCI import errors

- Make sure `eclipse-sumo`, `traci`, and `libsumo` are installed in the active Python environment.
- On Windows, verify that SUMO is on your `PATH` or that `SUMO_HOME` is set correctly.

---

##  Contributing

Contributions are welcome! Suggested improvements include:

- New microsimulation scenarios (e.g., signalized corridors).
- Additional CAV controllers (eco-approach, merging control, etc.).
- Quality-control scripts and new dashboard views.
- Enhanced XIL interfaces (e.g., ROS bindings).

Typical workflow:

1. Fork the repository.
2. Create a feature branch  
   `git checkout -b feature/amazing-feature`
3. Commit your changes  
   `git commit -m "Add amazing feature"`
4. Push to your fork  
   `git push origin feature/amazing-feature`
5. Open a Pull Request.

Please:

- Follow PEP 8 for Python style where practical.
- Add docstrings to new functions/classes.
- Include tests or small examples when adding major features.
- Update the README / docs for user-facing changes.

---

##  License

See the [LICENSE](LICENSE) file for details.

---

##  Contact

For questions, bug reports, or feature requests:

- Open an issue on the project’s GitHub repository.
- Or contact the maintainer / research team listed in the repo metadata (fill in as appropriate for your deployment).

---

##  Acknowledgments

This repository builds on work in:

- SUMO (Simulation of Urban MObility) and the TraCI API.
- Prior CAV controller and code generation examples by the project contributors.
- Coursework and research on mixed traffic microsimulation, PCC, and XIL testing.

---

**Last Updated**: December 2025  
**Status**: Research / Development Ready




















