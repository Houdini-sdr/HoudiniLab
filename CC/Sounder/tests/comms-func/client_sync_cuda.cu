/**
 * @file client_sync_cuda.cu
 * @brief Real-time client beacon sync on the Houdini radio with find_beacon_cuda.
 *
 * The RENEW ClientRadioSet/Radio abstraction won't drive the Houdini SDR, so this
 * uses the SAME SoapySDR C++ calls Radio.cc uses (Device::make / setupStream /
 * activate / read+writeStream) to (a) TX the beacon from the BS board and (b) run
 * the UE sync loop: EVERY FRAME, radioRx one frame and call the production
 * CommsLib::find_beacon_cuda (GPU) -- i.e. the client re-acquires the beacon in
 * real time, frame after frame, on the GPU. This is the core of receiver.cc's
 * clientSyncBeacon with a Houdini radio backend and no Iris HW correlator.
 *
 * Build on the DGX Spark:
 *   nvcc -O3 -std=c++17 -arch=native -DUSE_CUDA -DHOUDINI_USE_CUDA -I include -I . \
 *     tests/comms-func/client_sync_cuda.cu find_beacon_cuda.cu \
 *     comms-lib-avx.cc comms-lib-portable.cc utils.cc \
 *     -lpthread -lSoapySDR -o client_sync_cuda
 */
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Errors.hpp>

#include <sys/types.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>
#include <vector>

#include "include/comms-lib.h"

namespace {
constexpr int kSeqLen = 128;
using cf32 = std::complex<float>;
using ci16 = std::complex<int16_t>;

std::vector<cf32> MakeMatch(int n) {
  std::mt19937 rng(0x5eedU);
  std::uniform_int_distribution<int> bit(0, 1);
  std::vector<cf32> m(n);
  for (int i = 0; i < n; ++i)
    m[i] = cf32(bit(rng) ? 0.5f : -0.5f, bit(rng) ? 0.5f : -0.5f);
  return m;
}

// band-limited x f upsample via DFT zero-pad (once, at startup -> O(N^2) is fine)
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

SoapySDR::Device* OpenByIp(const std::string& ip, const std::string& port) {
  // C++ SoapyRemote discovery returns nothing here (vs SoapySDRUtil), so make the
  // remote device directly with the full kwargs Python enumerate produces.
  SoapySDR::Kwargs a;
  a["driver"] = "houdinisdr";
  a["remote"] = "tcp://" + ip + ":" + port;
  a["remote:driver"] = "houdinisdr-device";
  a["remote:type"] = "houdinisdr";
  a["HOUDINI_MTU"] = "3512";        // small device frame -> zero-copy RX (SH-259)
  try {
    return SoapySDR::Device::make(a);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "make(%s) failed: %s\n", a["remote"].c_str(), e.what());
    return nullptr;
  }
}

std::string opt(int argc, char** argv, const std::string& k, const std::string& d) {
  for (int i = 1; i < argc - 1; ++i)
    if (k == argv[i]) return argv[i + 1];
  return d;
}
}  // namespace

int main(int argc, char** argv) {
  const std::string tx_ip = opt(argc, argv, "--tx-ip", "168.6.244.21");
  const std::string rx_ip = opt(argc, argv, "--rx-ip", "168.6.244.22");
  const int tx_ch = std::stoi(opt(argc, argv, "--tx-ch", "1"));
  const int rx_ch = std::stoi(opt(argc, argv, "--rx-ch", "1"));
  const double nco = std::stod(opt(argc, argv, "--nco-mhz", "500")) * 1e6;
  const size_t frame = std::stoul(opt(argc, argv, "--frame", "8192"));
  const int iters = std::stoi(opt(argc, argv, "--iters", "40"));
  const std::string port = opt(argc, argv, "--port", "55132");
  const double rx_rate = 122.88e6;

  const auto match = MakeMatch(kSeqLen);          // == the find_beacon match
  // beacon replay loop = 2 reps of the x8-upsampled match + an equal guard
  auto rep = Upsample(match, 8);
  std::vector<cf32> loop;
  loop.insert(loop.end(), rep.begin(), rep.end());
  loop.insert(loop.end(), rep.begin(), rep.end());
  const size_t half = loop.size();
  loop.resize(half * 2, cf32(0, 0));
  float peak = 0;
  for (auto& c : loop) peak = std::max(peak, std::abs(c));
  const size_t n_load = loop.size();
  std::vector<int16_t> beacon(n_load * 2);
  for (size_t i = 0; i < n_load; ++i) {
    beacon[2 * i] = static_cast<int16_t>(std::lround(loop[i].real() / peak * 0.6f * 32767));
    beacon[2 * i + 1] = static_cast<int16_t>(std::lround(loop[i].imag() / peak * 0.6f * 32767));
  }

  // --- BS: TX the beacon (replay) on tx_ip ch tx_ch ---
  SoapySDR::Device* txd = OpenByIp(tx_ip, port);
  SoapySDR::Device* rxd = OpenByIp(rx_ip, port);
  if (!txd || !rxd) { std::fprintf(stderr, "device open failed\n"); return 1; }
  auto rates = txd->listSampleRates(SOAPY_SDR_TX, tx_ch);
  if (!rates.empty()) txd->setSampleRate(SOAPY_SDR_TX, tx_ch, rates.back());
  auto* txs = txd->setupStream(SOAPY_SDR_TX, "CS16", {static_cast<size_t>(tx_ch)},
                               {{"tx_mode", "replay"}});
  txd->setFrequency(SOAPY_SDR_TX, tx_ch, nco);
  {
    const void* buffs[1] = {beacon.data()};
    long long tns = 0;
    int txflags = 0;
    txd->writeStream(txs, buffs, n_load, txflags, tns, 1000000);  // load RAM
  }
  txd->activateStream(txs);
  std::printf("BS TX %s ch%d beacon %zu samp @ NCO %.0f MHz\n", tx_ip.c_str(),
              tx_ch, n_load, nco / 1e6);

  // --- UE: RX sync loop on rx_ip ch rx_ch ---
  rxd->setSampleRate(SOAPY_SDR_RX, rx_ch, rx_rate);
  rxd->setFrequency(SOAPY_SDR_RX, rx_ch, nco);
  auto* rxs = rxd->setupStream(SOAPY_SDR_RX, "CS16", {static_cast<size_t>(rx_ch)},
                               {{"local_port", std::to_string(10001 + rx_ch)}});
  rxd->activateStream(rxs);
  std::printf("UE RX %s ch%d frame %zu, %d frames -- syncing with find_beacon_cuda\n\n",
              rx_ip.c_str(), rx_ch, frame, iters);

  std::vector<int16_t> raw(frame * 2);
  std::vector<ci16> cap(frame);
  int hits = 0;
  for (int it = 0; it < iters; ++it) {
    size_t got = 0;
    while (got < frame) {
      void* buffs[1] = {raw.data() + got * 2};
      int flags = 0;
      long long tns = 0;
      int r = rxd->readStream(rxs, buffs, frame - got, flags, tns, 1000000);
      if (r <= 0) break;
      got += static_cast<size_t>(r);
    }
    if (got < frame) { std::printf("frame %3d: short read %zu\n", it, got); continue; }
    double p = 0;
    for (size_t i = 0; i < frame; ++i)                   // conj: matched-NCO R2C
      cap[i] = ci16(raw[2 * i], static_cast<int16_t>(-raw[2 * i + 1]));
    for (size_t i = 0; i < frame; ++i)
      p += double(raw[2*i])*raw[2*i] + double(raw[2*i+1])*raw[2*i+1];
    const double rms = std::sqrt(p / frame);
    const auto t0 = std::chrono::steady_clock::now();
    ssize_t idx = CommsLib::find_beacon_cuda(cap.data(), match, frame, 1.0f);
    if (idx < 0) {                                       // try the other conj sense
      for (size_t i = 0; i < frame; ++i) cap[i] = ci16(raw[2*i], raw[2*i+1]);
      idx = CommsLib::find_beacon_cuda(cap.data(), match, frame, 1.0f);
    }
    const double us = std::chrono::duration<double, std::micro>(
                          std::chrono::steady_clock::now() - t0).count();
    if (idx >= 0) ++hits;
    std::printf("frame %3d: rms %6.0f  find_beacon_cuda -> %6zd  (%6.0f us)  %s\n",
                it, rms, idx, us, idx >= 0 ? "*** SYNC ***" : "searching");
    std::fflush(stdout);
  }
  std::printf("\n%d/%d frames synced on the beacon (GPU, real time)\n", hits, iters);

  rxd->deactivateStream(rxs); rxd->closeStream(rxs);
  txd->deactivateStream(txs); txd->closeStream(txs);
  SoapySDR::Device::unmake(rxd); SoapySDR::Device::unmake(txd);
  return hits > 0 ? 0 : 1;
}
