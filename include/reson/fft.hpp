#ifndef RESON_FFT_HPP_
#define RESON_FFT_HPP_

#include <cmath>
#include <complex>
#include <cstdint>
#include <vector>

namespace reson
{

using Complex = std::complex<double>;

/// Next power of two >= n (minimum 1).
inline std::size_t next_pow2(std::size_t n)
{
  std::size_t p = 1;
  while (p < n) {
    p <<= 1;
  }
  return p;
}

/// In-place iterative radix-2 Cooley-Tukey FFT. `data.size()` MUST be a
/// power of two. Forward transform (no 1/N normalization applied here).
inline void fft_radix2(std::vector<Complex> & data)
{
  const std::size_t n = data.size();
  if (n <= 1) {
    return;
  }

  // Bit-reversal permutation.
  for (std::size_t i = 1, j = 0; i < n; ++i) {
    std::size_t bit = n >> 1;
    for (; j & bit; bit >>= 1) {
      j ^= bit;
    }
    j ^= bit;
    if (i < j) {
      std::swap(data[i], data[j]);
    }
  }

  // Iterative Cooley-Tukey.
  for (std::size_t len = 2; len <= n; len <<= 1) {
    const double ang = -2.0 * M_PI / static_cast<double>(len);
    const Complex wlen(std::cos(ang), std::sin(ang));
    for (std::size_t i = 0; i < n; i += len) {
      Complex w(1.0, 0.0);
      for (std::size_t k = 0; k < len / 2; ++k) {
        const Complex u = data[i + k];
        const Complex v = data[i + k + len / 2] * w;
        data[i + k] = u + v;
        data[i + k + len / 2] = u - v;
        w *= wlen;
      }
    }
  }
}

/// Hann window, reduces spectral leakage (mirrors the "windowed" step used
/// in the original ESP32/Python prototype).
inline void apply_hann_window(std::vector<double> & samples)
{
  const std::size_t n = samples.size();
  if (n < 2) {
    return;
  }
  for (std::size_t i = 0; i < n; ++i) {
    const double w = 0.5 * (1.0 - std::cos(2.0 * M_PI * static_cast<double>(i) /
      static_cast<double>(n - 1)));
    samples[i] *= w;
  }
}

/// Mean-center, Hann-window, zero-pad to the next power of two, and return
/// the single-sided magnitude spectrum (length = padded_n / 2 + 1).
/// `freq_resolution_hz` is set to sample_rate_hz / padded_n.
inline std::vector<double> magnitude_spectrum(
  std::vector<double> samples,
  double sample_rate_hz,
  double & freq_resolution_hz)
{
  // Mean-center (removes DC / sensor bias).
  double mean = 0.0;
  for (double s : samples) {
    mean += s;
  }
  mean /= static_cast<double>(samples.empty() ? 1 : samples.size());
  for (double & s : samples) {
    s -= mean;
  }

  apply_hann_window(samples);

  const std::size_t padded_n = next_pow2(samples.size());
  std::vector<Complex> buf(padded_n, Complex(0.0, 0.0));
  for (std::size_t i = 0; i < samples.size(); ++i) {
    buf[i] = Complex(samples[i], 0.0);
  }

  fft_radix2(buf);

  freq_resolution_hz = sample_rate_hz / static_cast<double>(padded_n);

  const std::size_t half = padded_n / 2 + 1;
  std::vector<double> mag(half);
  for (std::size_t i = 0; i < half; ++i) {
    mag[i] = std::abs(buf[i]);
  }
  return mag;
}

}  // namespace reson

#endif  // RESON_FFT_HPP_
