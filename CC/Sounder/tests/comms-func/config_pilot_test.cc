/**
 * @file config_pilot_test.cc
 * @brief The run configs through the REAL Config (AP-79 final review), NO
 *        hardware: the pilot slot is exactly one slot long, its symbols are
 *        fft_size points, and its spectrum sits inside the +-24 MHz channel.
 *
 * The bug this pins: for fft_size != 64 the Zadoff-Chu pilot was built at the
 * next power of two ABOVE ofdm_data_num (the sequence generator's own
 * padding), not at fft_size. At R1 (fft 256, 96 data subcarriers) that made a
 * 2560-sample pilot slot against the 4096-sample slot everything else uses:
 * the view mode threw at startup, the UE's burst copy read 6 KB past the
 * pilot and transmitted it, and the tones sat at twice the spacing, outside
 * the filters. Config now refuses a pilot slot of the wrong length, and this
 * test constructs every run config to prove none has one.
 *
 * Run from CC/Sounder (the configs' relative paths): ctest sets the working
 * directory.
 */
#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdio>
#include <string>
#include <vector>

#include "include/comms-lib.h"
#include "include/config.h"

namespace {
int g_fail = 0;
void check(bool ok, const std::string& what) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what.c_str());
  if (!ok) ++g_fail;
}

// Energy of one pilot symbol body outside +-edge_hz, dB, from the built slot.
double outOfBandDb(Config& c, double edge_hz) {
  const size_t n = c.fft_size();
  const size_t body = static_cast<size_t>(c.prefix()) + c.cp_size();  // symbol 0's body
  const auto& p = c.pilot_ci16();
  double in = 0.0, out = 0.0;
  for (size_t k = 0; k < n; ++k) {
    std::complex<double> a(0, 0);
    for (size_t t = 0; t < n; ++t) {
      const double ph = -2.0 * M_PI * static_cast<double>((k * t) % n) / static_cast<double>(n);
      a += std::complex<double>(p[body + t].real(), p[body + t].imag()) * std::complex<double>(std::cos(ph), std::sin(ph));
    }
    const double f = (k < n / 2 ? static_cast<double>(k) : static_cast<double>(k) - n) * c.rate() / n;
    (std::fabs(f) <= edge_hz ? in : out) += std::norm(a);
  }
  return 10.0 * std::log10(out / in + 1e-30);
}
}  // namespace

int main() {
  for (const char* f : {"files/houdini-r0.json", "files/houdini-dualband-r1.json", "files/houdini-dualband-r2.json",
                        "files/houdini-dualband.json", "files/houdini-dualband-40.json"}) {
    try {
      Config c(f, "/tmp", false, false, false);
      const bool mode_v = c.mode_v();
      std::printf("%s: slot %zu, pilot slot %zu, pilot_sym_f %zu, fft %zu, data %zu\n", f, c.samps_per_slot(),
                  c.pilot_ci16().size(), c.pilot_sym_f().at(0).size(), c.fft_size(), c.symbol_data_subcarrier_num());
      check(c.pilot_ci16().size() == c.samps_per_slot(), std::string(f) + ": the pilot slot is exactly one slot");
      check(c.pilot_sym_f().at(0).size() == c.fft_size(), std::string(f) + ": the pilot's frequency grid is fft_size points");
      {
        // The data symbols' pilot tones: every one must carry a real value
        // (unit magnitude ZC on the fft grid, or the 802.11 pilots at fft 64),
        // or the BS's timing fit and phase fix run on noise (final review 2).
        double min_mag = 1e9;
        for (const auto& v : c.pilot_sc()) min_mag = std::min(min_mag, static_cast<double>(std::abs(v)));
        std::printf("  %zu data-symbol pilot tones, smallest |value| %.3f\n", c.pilot_sc().size(), min_mag);
        check(!c.pilot_sc().empty() && min_mag > 0.5, std::string(f) + ": every data-symbol pilot tone carries a value (|v| > 0.5)");
        // And they are what the UE actually transmits: the pilot tones of the
        // first data symbol of the built U slot.
        const size_t n = c.fft_size();
        const size_t body = static_cast<size_t>(c.prefix()) + c.cp_size();
        const auto& ud = c.ue_data_ci16();
        std::vector<std::complex<double>> F(n);
        for (size_t k = 0; k < n; ++k) {
          std::complex<double> a(0, 0);
          for (size_t t = 0; t < n; ++t) {
            const double ph = -2.0 * M_PI * static_cast<double>((k * t) % n) / static_cast<double>(n);
            a += std::complex<double>(ud[body + t].real(), ud[body + t].imag()) * std::complex<double>(std::cos(ph), std::sin(ph));
          }
          F[(k + n / 2) % n] = a;  // DC-centred, like the grid
        }
        double worst = 1e9, strong = 0;
        for (size_t j = 0; j < c.data_ind().size(); ++j) strong = std::max(strong, std::abs(F[c.data_ind()[j]]));
        for (size_t i = 0; i < c.pilot_sc_ind().size(); ++i) worst = std::min(worst, std::abs(F[c.pilot_sc_ind()[i]]));
        std::printf("  transmitted pilot tones: weakest %.1f dB under the strongest data tone\n", 20 * std::log10(strong / worst));
        check(20 * std::log10(strong / worst) < 20.0, std::string(f) + ": every pilot tone is actually transmitted in the U slot");
      }
      if (mode_v) {
        const double oob = outOfBandDb(c, 24e6);
        std::printf("  pilot symbol energy outside +-24 MHz: %.1f dB\n", oob);
        check(oob < -40.0, std::string(f) + ": the pilot sits inside the +-24 MHz channel (outside < -40 dB)");
        const auto zc = CommsLib::getSequence(CommsLib::LTE_ZADOFF_CHU, c.symbol_data_subcarrier_num());
        check(zc.at(0).size() != c.fft_size(),
              std::string(f) + ": the generator's power-of-two symbol (" + std::to_string(zc.at(0).size()) +
                  ") is NOT fft_size, the old bug's shape");
      }
    } catch (const std::exception& e) {
      check(false, std::string(f) + ": Config threw: " + e.what());
    }
  }
  std::printf("%s: %d failure(s)\n", g_fail == 0 ? "ALL PASS" : "FAILED", g_fail);
  return g_fail == 0 ? 0 : 1;
}
