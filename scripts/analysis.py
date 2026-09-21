#!/usr/bin/env python3
"""
analysis.py

Standalone offline FFT analysis tool for motor vibration CSVs, kept from
the original data-analysis workflow (see the old Predictive_Maintenance_of_Motor
repo). Use this for one-off exploration and plotting of new recordings
before promoting a config to the ROS 2 pipeline in reson/.

Single file report:
  python3 analysis.py --file normal_2.csv

Compare multiple files (e.g. normal vs. two faults) + plot:
  python3 analysis.py --file normal_2.csv fault2_1.csv fault1_4.csv --plot
"""
import argparse

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

MIN_FREQ_HZ = 5.0
PEAK_RATIO = 0.10


def load_csv(path: str):
    df = pd.read_csv(path)
    cols_lower = {c.lower(): c for c in df.columns}

    def find(candidates):
        for cand in candidates:
            if cand in cols_lower:
                return cols_lower[cand]
        return None

    tcol = find(["time_ms", "timestamp", "time", "t", "ms"])
    vcol = find(["vibration", "value", "accel", "az", "ax", "ay", "z", "amplitude"])
    if vcol is None:
        numeric_cols = [c for c in df.columns if c != tcol and pd.api.types.is_numeric_dtype(df[c])]
        vcol = numeric_cols[0]

    t = df[tcol].to_numpy(dtype=float) if tcol else np.arange(len(df)) * (1000.0 / 142.7)
    v = df[vcol].to_numpy(dtype=float)
    return t, v


def analyze(path: str):
    t, v = load_csv(path)
    duration_s = (t[-1] - t[0]) / 1000.0
    sample_rate_hz = (len(v) - 1) / duration_s if duration_s > 0 else 142.7

    x = v - np.mean(v)
    std_dev = float(np.std(v))

    window = np.hanning(len(x))
    xw = x * window

    n_fft = 1
    while n_fft < len(xw):
        n_fft *= 2

    spectrum = np.fft.rfft(xw, n=n_fft)
    mag = np.abs(spectrum)
    freqs = np.fft.rfftfreq(n_fft, d=1.0 / sample_rate_hz)

    in_band = freqs >= MIN_FREQ_HZ
    mag_band = mag[in_band]
    freqs_band = freqs[in_band]

    dominant_idx = int(np.argmax(mag_band))
    dominant_freq = float(freqs_band[dominant_idx])
    dominant_mag = float(mag_band[dominant_idx])

    thresh = dominant_mag * PEAK_RATIO
    num_peaks = int(np.sum(mag_band > thresh))

    energy = mag ** 2
    total = float(np.sum(energy)) or 1.0

    def band_pct(lo, hi):
        sel = (freqs >= lo) & (freqs < hi) & (freqs >= MIN_FREQ_HZ)
        return 100.0 * float(np.sum(energy[sel])) / total

    top_idx = np.argsort(mag_band)[::-1][:15]
    top = [(freqs_band[i], mag_band[i]) for i in top_idx]

    report = {
        "path": path,
        "sample_rate_hz": sample_rate_hz,
        "duration_s": duration_s,
        "n_samples": len(v),
        "std_dev": std_dev,
        "min": float(np.min(v)),
        "max": float(np.max(v)),
        "peak_to_peak": float(np.max(v) - np.min(v)),
        "dominant_freq_hz": dominant_freq,
        "dominant_mag": dominant_mag,
        "num_peaks": num_peaks,
        "energy_5_10": band_pct(5, 10),
        "energy_10_50": band_pct(10, 50),
        "energy_50_100": band_pct(50, 100),
        "energy_above_100": band_pct(100, 1e9),
        "top15": top,
        "freqs": freqs,
        "mag": mag,
    }
    return report


def print_report(r: dict):
    print("=" * 70)
    print("FFT ANALYSIS REPORT - MOTOR VIBRATION DATA")
    print("=" * 70)
    print(f"File: {r['path']}")
    print(f"Duration: {r['duration_s']:.3f} s   Sampling rate: {r['sample_rate_hz']:.2f} Hz"
          f"   Samples: {r['n_samples']}")
    print()
    print(f"Std Dev: {r['std_dev']:.2f}   Min: {r['min']:.2f}   Max: {r['max']:.2f}"
          f"   Peak-to-peak: {r['peak_to_peak']:.2f}")
    print()
    print(f"Dominant frequency (>= {MIN_FREQ_HZ} Hz): {r['dominant_freq_hz']:.2f} Hz")
    print(f"Dominant magnitude: {r['dominant_mag']:.1f}")
    print(f"Number of significant peaks (>{int(PEAK_RATIO*100)}% max): {r['num_peaks']}")
    print()
    print("Energy distribution:")
    print(f"  5-10 Hz:    {r['energy_5_10']:.1f}%")
    print(f"  10-50 Hz:   {r['energy_10_50']:.1f}%")
    print(f"  50-100 Hz:  {r['energy_50_100']:.1f}%")
    print(f"  > 100 Hz:   {r['energy_above_100']:.1f}%")
    print()
    print("Top 15 frequency components:")
    print("-" * 70)
    for i, (f, m) in enumerate(r["top15"], start=1):
        pct = 100.0 * m / r["dominant_mag"] if r["dominant_mag"] else 0.0
        print(f"{i:2d}. {f:7.2f} Hz | Magnitude: {m:12.1f} | {pct:5.1f}%")
    print()


def print_comparison(reports: list):
    print("=" * 90)
    print("FFT ANALYSIS COMPARISON")
    print("=" * 90)
    header = f"{'Parameter':<22}" + "".join(f"{r['path']:>18}" for r in reports)
    print(header)
    rows = [
        ("Dominant Freq (Hz)", "dominant_freq_hz", "{:.2f}"),
        ("Dominant Magnitude", "dominant_mag", "{:.1f}"),
        ("Std Dev", "std_dev", "{:.2f}"),
        ("Peak-to-Peak", "peak_to_peak", "{:.2f}"),
        ("Duration (s)", "duration_s", "{:.2f}"),
        ("Num Peaks", "num_peaks", "{:d}"),
    ]
    for label, key, fmt in rows:
        line = f"{label:<22}"
        for r in reports:
            line += f"{fmt.format(r[key]):>18}"
        print(line)
    print()


def plot_reports(reports: list, max_freq_hz: float = 150.0):
    plt.figure(figsize=(10, 5))
    for r in reports:
        mask = r["freqs"] <= max_freq_hz
        plt.plot(r["freqs"][mask], r["mag"][mask], label=r["path"])
    plt.xlabel("Frequency (Hz)")
    plt.ylabel("Magnitude")
    plt.title("FFT Magnitude Spectrum Comparison")
    plt.legend()
    plt.tight_layout()
    plt.savefig("fft_comparison.png", dpi=150)
    print("Saved plot to fft_comparison.png")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--file", nargs="+", required=True, help="One or more CSV files")
    parser.add_argument("--plot", action="store_true", help="Save a comparison spectrum plot")
    args = parser.parse_args()

    reports = [analyze(f) for f in args.file]

    for r in reports:
        print_report(r)

    if len(reports) > 1:
        print_comparison(reports)

    if args.plot:
        plot_reports(reports)


if __name__ == "__main__":
    main()
