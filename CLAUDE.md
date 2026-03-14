# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Research project implementing a quaternion-based sensor fusion pipeline for 3D inertial position and orientation tracking using consumer-grade IMU (accelerometer + gyroscope + magnetometer) data. The output is both a working Python implementation and an academic paper.

## Commands

**Build PDF documentation:**
```bash
./scripts/build_pdf.sh
```
Compiles `docs/main.tex` → `docs/output/main.pdf` (runs pdflatex twice to resolve references).
Alternatively, use the project slash command `/latex-pdf-builder` which handles engine detection, error reporting, and bibtex/biber passes automatically.

**Generate figures:**
```bash
python docs/scripts/plot_gyro_errors.py
```
Outputs `docs/images/gyro_drift_errors.{pdf,png}` and `gyro_sampling_comparison.{pdf,png}`.

**Run notebooks:** Open in Jupyter (`jupyter notebook` or VS Code). Key notebooks:
- `notebooks/madgwick_pipeline.ipynb` — main implementation
- `notebooks/sensor_exploration.ipynb` — exploratory analysis
- `notebooks/drone_attitude.ipynb` — additional attitude estimation work

**Key Python dependencies:** `pandas`, `numpy`, `matplotlib`, `plotly`, `ahrs`

> **Note:** There is no `requirements.txt` or `pyproject.toml`. Dependencies must be installed manually (e.g. `pip install pandas numpy matplotlib plotly ahrs`).

## Architecture

### Signal Processing Pipeline

Raw CSV sensor data → attitude estimation → frame rotation → gravity removal → double integration → 3D position/trajectory.

1. **Data loading:** CSV files in `data/` with columns `timestamp, accel_x/y/z, gyro_x/y/z, mag_x/y/z, pressure, altitude`. Timestamps are in milliseconds; gyroscope in deg/s.

2. **Attitude estimation (two implementations):**
   - *Complementary filter:* SLERP-blends high-pass gyro integration and low-pass accelerometer tilt. Parameter α ≈ 0.96–0.98.
   - *Madgwick filter:* Gradient descent on orientation error in quaternion space (`ahrs.filters.Madgwick`). Handles accelerometer + magnetometer natively.

3. **Frame rotation:** `a_world = q ⊗ a_body ⊗ q*` (quaternion conjugation) rotates body-frame acceleration to world frame.

4. **Gravity removal:** Subtract `[0, 0, 9.81]` from world-frame acceleration.

5. **Position integration:** Trapezoidal/Euler integration twice (→ velocity → position).

### Error Sources
- Gyroscope bias drift: linear error growth
- Gyroscope random walk: √t error growth
- Gravity leakage: residual gravity from orientation error causes quadratic position drift

### Documentation Structure

LaTeX paper in `docs/chapters/`: introduction → background → methodology → implementation (complementary filter) → madgwick → results → conclusions. The `results.tex` chapter is fully written (world-frame projection, position integration analysis). Only `conclusions.tex` is a TODO placeholder.

**VS Code / LaTeX Workshop:** `.vscode/settings.json` configures LaTeX Workshop to auto-build on save using pdflatex, outputting to `docs/output/`. Editing `.tex` files in VS Code will trigger automatic builds.

**`main.py`:** PyCharm boilerplate placeholder — not a functional entry point. Ignore it.

## Data

Datasets in `data/` represent physical motion experiments: square trajectories (`cuadrados.csv`), linear displacement, vertical up/down with and without pauses. All collected ~March 2026 at ~100 Hz.
