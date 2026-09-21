# reson

Motor vibration fault diagnosis for ROS 2.

```
CSV (real rig data) -> csv_replay_node -> /reson/vibration_raw
                                              |
                                              v
                                          fft_node  -> /reson/fft_spectrum
                                              |
                                              v
                                       diagnosis_node -> /diagnostics
                                                       -> /reson/health_index
```

- `csv_replay_node` (C++): reads a `(time_ms, value)` vibration CSV recorded
  on the physical motor rig and republishes it sample-by-sample at (roughly)
  its original timing.
- `fft_node` (C++): buffers samples into windows, mean-centers + Hann-windows
  + zero-pads to a power of two, runs a self-contained radix-2 FFT (no
  external FFT dependency), and publishes the magnitude spectrum.
- `diagnosis_node` (C++): turns each spectrum into a spectral fingerprint
  (std dev, dominant frequency/magnitude, peak count, band-energy split),
  compares it to a healthy-motor baseline fingerprint (normalized Euclidean
  distance) and a rule-based threshold table, and publishes the diagnosis on
  the standard `/diagnostics` topic plus a simple 0-100 `/reson/health_index`.
- `scripts/analysis.py`, `scripts/compute_baseline.py` (Python, retained):
  offline dataset exploration/plots and baseline-fingerprint generation from
  your own `normal_*.csv` recordings.

## 1. Install dependencies (WSL / Ubuntu, ROS 2 already installed)

```bash
sudo apt update
sudo apt install -y python3-pip
pip3 install --user numpy pandas matplotlib
```

No extra C++ FFT/DSP library is required - the FFT is implemented in
`include/reson/fft.hpp`.

## 2. Put the package in your workspace

```bash
cd ~/ros2_ws/src
# (repo already at ros2_ws/src/reson per your layout - otherwise:)
# cp -r /path/to/reson ~/ros2_ws/src/

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select reson
source install/setup.bash
```

## 3. Add your CSVs

Copy your recorded CSVs (e.g. `normal_2.csv`, `fault2_1.csv`, `fault1_4.csv`)
into `ros2_ws/src/reson/data/`. Any two-column CSV with a header works; the
node auto-detects a time column (`time_ms`/`timestamp`/`time`) and a value
column (`vibration`/`value`/`accel`/...). If it guesses wrong, pass
`time_column:=` / `value_column:=` explicitly (see step 5).

## 4. (Optional but recommended) Generate your own baseline

```bash
cd ~/ros2_ws/src/reson
python3 scripts/compute_baseline.py data/normal_*.csv
```

Paste the printed `baseline:` block into `config/reson_params.yaml` under
`diagnosis_node.ros__parameters.baseline`, then `colcon build --packages-select reson` again
(or just edit the *installed* copy under `install/reson/share/reson/config/`
for quick iteration).

## 5. Run the full pipeline

```bash
source ~/ros2_ws/install/setup.bash

ros2 launch reson reson_pipeline.launch.py \
  csv_path:=$(ros2 pkg prefix reson)/share/reson/data/fault2_1.csv
```

Replay a normal run on loop instead:

```bash
ros2 launch reson reson_pipeline.launch.py \
  csv_path:=.../data/normal_2.csv loop:=true playback_rate:=2.0
```

## 6. Watch the diagnosis

```bash
# Human-readable diagnostics stream
ros2 topic echo /diagnostics

# Just the health index (0 = failed, 100 = perfect)
ros2 topic echo /reson/health_index

# GUI monitor
ros2 run rqt_runtime_monitor rqt_runtime_monitor
```

## 7. Run nodes individually (debugging)

```bash
ros2 run reson csv_replay_node --ros-args -p csv_path:=/abs/path/normal_2.csv
ros2 run reson fft_node --ros-args --params-file src/reson/config/reson_params.yaml
ros2 run reson diagnosis_node --ros-args --params-file src/reson/config/reson_params.yaml
```

## Tuning

Everything lives in `config/reson_params.yaml`:
- `window_size` / `hop_size`: FFT window and overlap.
- `baseline.*`: the healthy-motor fingerprint (regenerate with
  `compute_baseline.py`).
- `scale.*`: per-feature normalization for the baseline distance.
- `thresholds.*`: rule-based severity cutoffs (defaults come from the
  reference grinder-motor rig - std dev, dominant frequency, dominant
  magnitude, and peak-count bands for Normal/Warning/Alert/Critical).

`diagnosis_node` combines both: rule thresholds set the severity level
(`OK`/`WARN`/`ERROR` on `/diagnostics`), and the baseline distance further
trims the 0-100 health index so gradual drift is visible even before any
single threshold trips.
