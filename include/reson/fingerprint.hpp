#ifndef RESON_FINGERPRINT_HPP_
#define RESON_FINGERPRINT_HPP_

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace reson
{

/// Compact spectral fingerprint of one analysis window, mirroring the
/// statistics used in the original Python/ESP32 prototype (std dev,
/// dominant frequency/magnitude, peak count, band energy split).
struct Fingerprint
{
  double std_dev = 0.0;
  double dominant_freq_hz = 0.0;
  double dominant_mag = 0.0;
  int num_peaks = 0;
  double energy_5_10 = 0.0;
  double energy_10_50 = 0.0;
  double energy_50_100 = 0.0;
  double energy_above_100 = 0.0;

  /// Fixed-order feature vector used for similarity/distance calculations.
  /// Magnitude is log-compressed since it spans orders of magnitude.
  std::vector<double> to_vector() const
  {
    return {
      std_dev,
      dominant_freq_hz,
      std::log10(std::max(dominant_mag, 1.0)),
      static_cast<double>(num_peaks),
      energy_5_10,
      energy_10_50,
      energy_50_100,
      energy_above_100
    };
  }
};

/// Extract a Fingerprint from a magnitude spectrum. `min_freq_hz` excludes
/// low-frequency drift/bias (the old prototype ignored < 5 Hz).
/// `peak_ratio` is the fraction of the in-band max magnitude a bin must
/// exceed to count as a "significant peak" (0.10 == 10%, as in the
/// original analysis.py).
inline Fingerprint extract_fingerprint(
  const std::vector<double> & mag_spectrum,
  double freq_resolution_hz,
  double std_dev,
  double min_freq_hz = 5.0,
  double peak_ratio = 0.10)
{
  Fingerprint fp;
  fp.std_dev = std_dev;

  const std::size_t min_bin = static_cast<std::size_t>(
    std::ceil(min_freq_hz / freq_resolution_hz));

  double total_energy = 0.0;
  double band_5_10 = 0.0, band_10_50 = 0.0, band_50_100 = 0.0, band_above = 0.0;
  double max_in_band = 0.0;
  std::size_t max_bin = min_bin < mag_spectrum.size() ? min_bin : 0;

  for (std::size_t i = 0; i < mag_spectrum.size(); ++i) {
    const double freq = static_cast<double>(i) * freq_resolution_hz;
    const double e = mag_spectrum[i] * mag_spectrum[i];
    total_energy += e;

    if (i >= min_bin) {
      if (freq < 10.0) {
        band_5_10 += e;
      } else if (freq < 50.0) {
        band_10_50 += e;
      } else if (freq < 100.0) {
        band_50_100 += e;
      } else {
        band_above += e;
      }
      if (mag_spectrum[i] > max_in_band) {
        max_in_band = mag_spectrum[i];
        max_bin = i;
      }
    }
  }

  fp.dominant_freq_hz = static_cast<double>(max_bin) * freq_resolution_hz;
  fp.dominant_mag = max_in_band;

  int peaks = 0;
  const double thresh = max_in_band * peak_ratio;
  for (std::size_t i = min_bin; i < mag_spectrum.size(); ++i) {
    if (mag_spectrum[i] > thresh) {
      ++peaks;
    }
  }
  fp.num_peaks = peaks;

  const double denom = total_energy > 0.0 ? total_energy : 1.0;
  fp.energy_5_10 = 100.0 * band_5_10 / denom;
  fp.energy_10_50 = 100.0 * band_10_50 / denom;
  fp.energy_50_100 = 100.0 * band_50_100 / denom;
  fp.energy_above_100 = 100.0 * band_above / denom;

  return fp;
}

/// Normalized Euclidean distance between two fingerprint feature vectors.
/// `scale` holds a per-feature normalization divisor (roughly the expected
/// spread of that feature) so no single feature dominates the distance.
inline double fingerprint_distance(
  const Fingerprint & a,
  const Fingerprint & b,
  const std::vector<double> & scale)
{
  const auto va = a.to_vector();
  const auto vb = b.to_vector();
  double sum_sq = 0.0;
  for (std::size_t i = 0; i < va.size() && i < vb.size(); ++i) {
    const double s = (i < scale.size() && scale[i] > 1e-9) ? scale[i] : 1.0;
    const double d = (va[i] - vb[i]) / s;
    sum_sq += d * d;
  }
  return std::sqrt(sum_sq / static_cast<double>(va.size()));
}

/// Cosine similarity between two fingerprint feature vectors (1.0 = identical
/// direction, 0.0 = orthogonal). Provided as an alternative similarity
/// measure alongside the Euclidean distance above.
inline double fingerprint_cosine_similarity(const Fingerprint & a, const Fingerprint & b)
{
  const auto va = a.to_vector();
  const auto vb = b.to_vector();
  double dot = 0.0, na = 0.0, nb = 0.0;
  for (std::size_t i = 0; i < va.size() && i < vb.size(); ++i) {
    dot += va[i] * vb[i];
    na += va[i] * va[i];
    nb += vb[i] * vb[i];
  }
  if (na < 1e-12 || nb < 1e-12) {
    return 0.0;
  }
  return dot / (std::sqrt(na) * std::sqrt(nb));
}

/// Severity levels, ordered worst-last so `std::max` picks the worst.
enum class Severity : int
{
  kNormal = 0,
  kWarning = 1,
  kAlert = 2,
  kCritical = 3
};

inline std::string severity_to_string(Severity s)
{
  switch (s) {
    case Severity::kNormal: return "NORMAL";
    case Severity::kWarning: return "WARNING";
    case Severity::kAlert: return "ALERT";
    case Severity::kCritical: return "CRITICAL";
  }
  return "UNKNOWN";
}

/// Threshold table for rule-based classification, defaults taken from the
/// empirical table in the original grinder-motor experiment. Override via
/// ROS 2 parameters / config/reson_params.yaml for your own motor.
struct Thresholds
{
  // Std dev: higher is worse.
  double std_dev_warning = 2000.0;
  double std_dev_alert = 3000.0;
  double std_dev_critical = 5000.0;

  // Dominant frequency: LOWER is worse (energy shifting down = fault).
  double freq_warning = 20.0;
  double freq_alert = 16.0;
  double freq_critical = 13.0;

  // Dominant magnitude: higher is worse.
  double mag_warning = 200000.0;
  double mag_alert = 400000.0;
  double mag_critical = 1000000.0;

  // Number of significant peaks: LOWER is worse (energy concentrating into
  // fewer, larger peaks = fault).
  double peaks_warning = 700.0;
  double peaks_alert = 500.0;
  double peaks_critical = 300.0;
};

inline Severity classify_std_dev(double v, const Thresholds & t)
{
  if (v >= t.std_dev_critical) {return Severity::kCritical;}
  if (v >= t.std_dev_alert) {return Severity::kAlert;}
  if (v >= t.std_dev_warning) {return Severity::kWarning;}
  return Severity::kNormal;
}

inline Severity classify_freq(double v, const Thresholds & t)
{
  if (v < t.freq_critical) {return Severity::kCritical;}
  if (v < t.freq_alert) {return Severity::kAlert;}
  if (v < t.freq_warning) {return Severity::kWarning;}
  return Severity::kNormal;
}

inline Severity classify_mag(double v, const Thresholds & t)
{
  if (v >= t.mag_critical) {return Severity::kCritical;}
  if (v >= t.mag_alert) {return Severity::kAlert;}
  if (v >= t.mag_warning) {return Severity::kWarning;}
  return Severity::kNormal;
}

inline Severity classify_peaks(double v, const Thresholds & t)
{
  if (v < t.peaks_critical) {return Severity::kCritical;}
  if (v < t.peaks_alert) {return Severity::kAlert;}
  if (v < t.peaks_warning) {return Severity::kWarning;}
  return Severity::kNormal;
}

/// Overall diagnosis result.
struct Diagnosis
{
  Severity overall = Severity::kNormal;
  Severity std_dev_sev = Severity::kNormal;
  Severity freq_sev = Severity::kNormal;
  Severity mag_sev = Severity::kNormal;
  Severity peaks_sev = Severity::kNormal;
  double baseline_distance = 0.0;
  double health_index = 100.0;   // 0 (failed) .. 100 (perfect)
  std::string fault_hint;
};

/// Combine per-parameter rule thresholds with the baseline distance into a
/// single diagnosis + 0-100 health index.
/// `distance_scale` controls how quickly the baseline distance erodes the
/// health index (bigger => more tolerant of drift from baseline).
inline Diagnosis diagnose(
  const Fingerprint & fp,
  const Fingerprint & baseline,
  const std::vector<double> & baseline_feature_scale,
  const Thresholds & thresholds,
  double distance_scale = 3.0)
{
  Diagnosis d;
  d.std_dev_sev = classify_std_dev(fp.std_dev, thresholds);
  d.freq_sev = classify_freq(fp.dominant_freq_hz, thresholds);
  d.mag_sev = classify_mag(fp.dominant_mag, thresholds);
  d.peaks_sev = classify_peaks(static_cast<double>(fp.num_peaks), thresholds);

  d.overall = std::max({d.std_dev_sev, d.freq_sev, d.mag_sev, d.peaks_sev});

  d.baseline_distance = fingerprint_distance(fp, baseline, baseline_feature_scale);

  // Health index: rule-based severity dominates (25 pts lost per level),
  // then the residual is trimmed by how far the spectrum has drifted from
  // the healthy baseline fingerprint.
  const double severity_penalty = 25.0 * static_cast<int>(d.overall);
  const double drift_penalty = std::min(40.0, d.baseline_distance / distance_scale * 40.0);
  d.health_index = std::max(0.0, 100.0 - severity_penalty - drift_penalty);

  if (d.overall == Severity::kNormal) {
    d.fault_hint = "none";
  } else if (d.freq_sev >= Severity::kAlert && d.mag_sev >= Severity::kAlert) {
    d.fault_hint = "likely bearing defect / severe imbalance (energy shifted to low freq, high magnitude)";
  } else if (d.peaks_sev >= Severity::kAlert) {
    d.fault_hint = "possible structural looseness / bearing wear (spectral energy concentrating)";
  } else if (d.std_dev_sev >= Severity::kWarning) {
    d.fault_hint = "possible misalignment / imbalance (elevated overall vibration energy)";
  } else {
    d.fault_hint = "deviation from baseline spectrum, cause unclear - inspect motor";
  }

  return d;
}

}  // namespace reson

#endif  // RESON_FINGERPRINT_HPP_
