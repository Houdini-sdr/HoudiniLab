/**
 * @file beacon_tx_gold.cc
 * @brief TX the Sounder's REAL beacon (STS + the actual CommsLib GOLD_IFFT
 *        sequence that receiver.cc::syncSearch correlates against) continuously
 *        from a Houdini board, so a Houdini UE running the unmodified
 *        receiver.cc client-sync path can acquire it.
 *
 * Unlike beacon_hil/client_sync_cuda (which used a stand-in QPSK match), this
 * builds the beacon from CommsLib::getSequence(GOLD_IFFT)/getSequence(STS_SEQ)
 * exactly as Config::genPilots() does, so config_->gold_cf32() on the UE side
 * matches. The beacon is band-limited x8-upsampled (DAC replay runs at ~983.04
 * MSPS, the UE samples at the config rate) and, by default, CONJUGATED: the
 * matched-NCO real->complex mixer delivers the beacon conjugated and syncSearch
 * feeds the raw RX straight to find_beacon, so pre-conjugating the TX cancels it.
 *
 * Build: added as a CMake target (links comms-lib + utils + muFFT + SoapySDR).
 * Run (venv SoapySDR runtime):
 *   ./beacon_tx_gold --tx-ip 168.6.244.21 --tx-ch 1 --nco-mhz 500
 */
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

#include "comms-lib.h"

namespace {
using cf32 = std::complex<float>;
volatile std::sig_atomic_t g_stop = 0;
void OnSig(int) { g_stop = 1; }

// band-limited x f upsample via DFT zero-pad (startup only -> O(N^2) is fine)
std::vector<cf32> Upsample(const std::vector<cf32>& x, int f) {
  const int L = static_cast<int>(x.size()), Lu = L * f, h = L / 2;
  std::vector<cf32> X(L), Xu(Lu, cf32(0, 0)), xu(Lu);
  for (int k = 0; k < L; ++k) {
    cf32 s(0, 0);
    for (int n = 0; n < L; ++n)
      s += x[n] * std::exp(cf32(0, -2.0 * M_PI * k * n / L));
    X[k] = s;
  }
  for (int k = 0; k < h; ++k) { Xu[k] = X[k]; Xu[Lu - h + k] = X[h + k]; }
  for (int n = 0; n < Lu; ++n) {
    cf32 s(0, 0);
    for (int k = 0; k < Lu; ++k)
      if (Xu[k] != cf32(0, 0))
        s += Xu[k] * std::exp(cf32(0, 2.0 * M_PI * k * n / Lu));
    xu[n] = s;
  }
  return xu;
}

std::string opt(int c, char** v, const std::string& k, const std::string& d) {
  for (int i = 1; i < c - 1; ++i)
    if (k == v[i]) return v[i + 1];
  return d;
}
}  // namespace

int main(int argc, char** argv) {
  const std::string tx_ip = opt(argc, argv, "--tx-ip", "168.6.244.21");
  const int tx_ch = std::stoi(opt(argc, argv, "--tx-ch", "1"));
  const double nco = std::stod(opt(argc, argv, "--nco-mhz", "500")) * 1e6;
  const std::string port = opt(argc, argv, "--port", "55132");
  const int upf = std::stoi(opt(argc, argv, "--upsample", "8"));
  const float amp = std::stof(opt(argc, argv, "--amp", "0.6"));
  // The Houdini replay RAM is 4096 samples deep, so the loop (period x upf)
  // must fit; period 512 x8 = 4096 packs the beacon back-to-back (rx period 512).
  const int period = std::stoi(opt(argc, argv, "--period", "512"));  // rx samps
  constexpr size_t kReplayDepth = 4096;
  const bool conj = std::stoi(opt(argc, argv, "--conj", "1")) != 0;
  std::signal(SIGINT, OnSig);
  std::signal(SIGTERM, OnSig);

  // --- Build the REAL beacon: 15 x STS(16) + 2 x GOLD_IFFT(128), as genPilots ---
  auto gold = CommsLib::getSequence(CommsLib::GOLD_IFFT);  // [2][128]
  auto sts = CommsLib::getSequence(CommsLib::STS_SEQ);     // [2][16]
  std::vector<cf32> beacon;
  for (int r = 0; r < 15; ++r)
    for (size_t i = 0; i < sts[0].size(); ++i)
      beacon.emplace_back(sts[0][i], sts[1][i]);
  for (int r = 0; r < 2; ++r)
    for (size_t i = 0; i < gold[0].size(); ++i)
      beacon.emplace_back(gold[0][i], gold[1][i]);
  if (conj)
    for (auto& c : beacon) c = std::conj(c);
  std::printf("beacon: %zu samples (STSx15 + GOLD_IFFTx2)%s\n", beacon.size(),
              conj ? " conjugated" : "");

  // x8 upsample so each rep lands at the UE sample rate, then pad to a fixed
  // received period so the beacon recurs inside every UE detection window.
  auto up = Upsample(beacon, upf);
  std::vector<cf32> loop = up;
  size_t loop_len = std::min<size_t>(static_cast<size_t>(period) * upf, kReplayDepth);
  if (loop_len < up.size()) loop_len = up.size();  // never truncate the beacon
  loop.resize(loop_len, cf32(0, 0));  // guard/pad
  float peak = 1e-30f;
  for (auto& c : loop) peak = std::max(peak, std::abs(c));
  const size_t n_load = loop.size();
  std::vector<int16_t> iq(n_load * 2);
  for (size_t i = 0; i < n_load; ++i) {
    iq[2 * i] = static_cast<int16_t>(std::lround(loop[i].real() / peak * amp * 32767));
    iq[2 * i + 1] = static_cast<int16_t>(std::lround(loop[i].imag() / peak * amp * 32767));
  }

  SoapySDR::Kwargs a;
  a["driver"] = "houdinisdr";
  a["remote"] = "tcp://" + tx_ip + ":" + port;
  a["remote:driver"] = "houdinisdr-device";
  a["remote:type"] = "houdinisdr";
  SoapySDR::Device* dev = SoapySDR::Device::make(a);
  if (dev == nullptr) { std::fprintf(stderr, "open %s failed\n", tx_ip.c_str()); return 1; }

  auto rates = dev->listSampleRates(SOAPY_SDR_TX, tx_ch);
  if (!rates.empty())
    dev->setSampleRate(SOAPY_SDR_TX, tx_ch,
                       *std::max_element(rates.begin(), rates.end()));
  auto* txs = dev->setupStream(SOAPY_SDR_TX, "CS16", {static_cast<size_t>(tx_ch)},
                               {{"tx_mode", "replay"}});
  dev->setFrequency(SOAPY_SDR_TX, tx_ch, nco);
  const void* buffs[1] = {iq.data()};
  long long tns = 0;
  int flags = 0;
  dev->writeStream(txs, buffs, n_load, flags, tns, 1000000);  // load replay RAM
  dev->activateStream(txs);
  std::printf("TX %s ch%d: %zu-sample replay loop @ NCO %.0f MHz (rx period %d) "
              "-- Ctrl-C to stop\n", tx_ip.c_str(), tx_ch, n_load, nco / 1e6, period);
  std::fflush(stdout);

  while (g_stop == 0) std::this_thread::sleep_for(std::chrono::milliseconds(200));

  dev->deactivateStream(txs);
  dev->closeStream(txs);
  SoapySDR::Device::unmake(dev);
  std::printf("beacon TX stopped\n");
  return 0;
}
