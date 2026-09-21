#!/usr/bin/env python3
"""
compute_baseline.py

Computes the average spectral fingerprint (std dev, dominant frequency,
dominant magnitude, peak count, band-energy split) across one or more
"normal" (healthy-motor) vibration CSVs, and prints a YAML block ready to
paste into config/reson_params.yaml -> diagnosis_node.ros__parameters.baseline

This mirrors exactly the feature extraction done on-line in
include/reson/fingerprint.hpp, so the baseline it produces is directly
comparable to what diagnosis_node computes at runtime.

Usage:
  python3 compute_baseline.py normal_2.csv normal_4.csv normal_5.csv normal_6.csv
  python3 compute_baseline.py --time-col time_ms --value-col vibration *.csv
"""
import argparse
import glob
import sys
from typing import Optional

import numpy as np
import pandas as pd


def load_csv(path: str, time_col: Optional[str], value_col: Optional[str]):
    df = pd.read_csv(path)
    cols_lower = {c.lower(): c for c in df.columns}

    def find(col_hint, candidates):
        if col_hint and col_hint in df.columns:
            return col_hint
        for cand in candidates:
            if cand in cols_lower:
                return cols_lower[cand]
        return None

    tcol = find(time_col, ["time_ms", "timestamp", "time", "t", "ms"])
    vcol = find(value_col, ["vibration", "value", "accel", "az", "ax", "ay", "z", "amplitude"])
    if vcol is None:
        numeric_cols = [c for c in df.columns if c != tcol and pd.api.types.is_numeric_dtype(df[c])]
        if not numeric_cols:
            raise ValueError(f"Could not find a value column in {path}")
        vcol = numeric_cols[0]

    t = df[tcol].to_numpy(dtype=float) if tcol else np.arange(len(df)) * (1000.0 / 142.7)
    v = df[vcol].to_numpy(dtype=float)
    return t, v


def fingerprint(t_ms: np.ndarray, v: np.ndarray, min_freq_hz: float = 5.0, peak_ratio: float = 0.10):
    duration_s = (t_ms[-1] - t_ms[0]) / 1000.0
    sample_rate_hz = (len(v) - 1) / duration_s if duration_s > 0 else 142.7

    x = v - np.mean(v)
    std_dev = float(np.std(v))

    window = np.hanning(len(x))
    x = x * window

    n_fft = 1
    while n_fft < len(x):
        n_fft *= 2

    spectrum = np.fft.rfft(x, n=n_fft)
    mag = np.abs(spectrum)
    freqs = np.fft.rfftfreq(n_fft, d=1.0 / sample_rate_hz)

    in_band = freqs >= min_freq_hz
    mag_band = mag[in_band]
    freqs_band = freqs[in_band]

    dominant_idx = int(np.argmax(mag_band))
    dominant_freq = float(freqs_band[dominant_idx])
    dominant_mag = float(mag_band[dominant_idx])

    thresh = dominant_mag * peak_ratio
    num_peaks = int(np.sum(mag_band > thresh))

    energy = mag ** 2
    total = float(np.sum(energy)) or 1.0

    def band_pct(lo, hi):
        sel = (freqs >= lo) & (freqs < hi) & (freqs >= min_freq_hz)
        return 100.0 * float(np.sum(energy[sel])) / total

    return {
        "std_dev": std_dev,
        "dominant_freq_hz": dominant_freq,
        "dominant_mag": dominant_mag,
        "num_peaks": num_peaks,
        "energy_5_10": band_pct(5, 10),
        "energy_10_50": band_pct(10, 50),
        "energy_50_100": band_pct(50, 100),
        "energy_above_100": band_pct(100, 1e9),
        "sample_rate_hz": sample_rate_hz,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("files", nargs="+", help="Normal/healthy-motor CSV files (globs OK)")
    parser.add_argument("--time-col", default=None)
    parser.add_argument("--value-col", default=None)
    args = parser.parse_args()

    paths = []
    for pattern in args.files:
        matches = glob.glob(pattern)
        paths.extend(matches if matches else [pattern])

    if not paths:
        print("No input files found.", file=sys.stderr)
        sys.exit(1)

    fps = []
    for p in paths:
        t, v = load_csv(p, args.time_col, args.value_col)
        fp = fingerprint(t, v)
        fps.append(fp)
        print(
            f"{p}: std={fp['std_dev']:.1f}  f0={fp['dominant_freq_hz']:.2f}Hz  "
            f"mag0={fp['dominant_mag']:.1f}  peaks={fp['num_peaks']}  "
            f"rate={fp['sample_rate_hz']:.2f}Hz",
            file=sys.stderr,
        )

    avg = {k: float(np.mean([fp[k] for fp in fps])) for k in fps[0] if k != "sample_rate_hz"}

    print("\n# Paste into config/reson_params.yaml -> diagnosis_node.ros__parameters.baseline")
    print("baseline:")
    print(f"  std_dev: {avg['std_dev']:.2f}")
    print(f"  dominant_freq_hz: {avg['dominant_freq_hz']:.2f}")
    print(f"  dominant_mag: {avg['dominant_mag']:.1f}")
    print(f"  num_peaks: {int(round(avg['num_peaks']))}")
    print(f"  energy_5_10: {avg['energy_5_10']:.1f}")
    print(f"  energy_10_50: {avg['energy_10_50']:.1f}")
    print(f"  energy_50_100: {avg['energy_50_100']:.1f}")
    print(f"  energy_above_100: {avg['energy_above_100']:.1f}")


if __name__ == "__main__":
    main()
