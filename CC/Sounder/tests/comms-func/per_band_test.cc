/**
 * @file per_band_test.cc
 * @brief AP-85, per-band channel widths through the REAL Config and the REAL
 *        BS recorder, NO hardware: the X-band at 270 RB (3240 tones) beside
 *        the sub-6 at 133 RB (1596) on one numerology.
 *
 * WHAT IS PINNED (the numbered sections below):
 *   1. Backward compatibility: every buffer Config builds for six shipped
 *      configs hashes to the value the build before per-band widths (d27d5f5)
 *      produced, and so does the prefiltered TX interpolation of the pilot.
 *   2. The channel -> band map, on files/houdini-dualband-xw.json and on a
 *      probe whose lane order differs from the letter order.
 *   3. The wide band's pilot and data, known answers.
 *   4. The two bands' levels.
 *   5. The config's refusals.
 *   6. The BS recorder end to end in view mode (UDP loopback): each antenna's
 *      CSI, channel constants and constellation against its own band.
 * Assertions name the mutation that breaks them where one applies.
 *
 * Run from CC/Sounder (the configs' relative paths): ctest sets the working
 * directory.
 */
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iterator>
#include <map>
#include <string>
#include <vector>

#include "include/comms-lib.h"
#include "include/config.h"
#include "include/houdini/tx_rx_boundary.h"
#include "include/recorder_worker.h"
#include "nlohmann/json.hpp"

namespace {
using json = nlohmann::json;
using cf = std::complex<float>;
using cs16 = std::complex<int16_t>;

int g_fail = 0;
void check(bool ok, const std::string& what) {
  std::printf("%s  %s\n", ok ? "PASS" : "FAIL", what.c_str());
  if (!ok) ++g_fail;
}

std::string g_dir;  // scratch directory: the UL references and the variant configs

uint64_t fnv(const void* p, size_t n, uint64_t h = 1469598103934665603ULL) {
  const auto* b = static_cast<const unsigned char*>(p);
  for (size_t i = 0; i < n; ++i) {
    h ^= b[i];
    h *= 1099511628211ULL;
  }
  return h;
}
template <class T>
uint64_t hv(const std::vector<T>& v) {
  return fnv(v.data(), v.size() * sizeof(T));
}

// Every band-0 buffer Config builds, plus what it writes and what the
// prefiltered interpolator makes of the pilot slot.
std::pair<uint64_t, uint64_t> golden(Config& c) {
  uint64_t h = 1469598103934665603ULL;
  auto mix = [&h](uint64_t x) { h = fnv(&x, sizeof x, h); };
  mix(hv(c.pilot_ci16()));
  mix(hv(c.ue_data_ci16()));
  mix(hv(c.pilot_sym_f().at(0)));
  mix(hv(c.pilot_sym_f().at(1)));
  mix(hv(c.pilot_sym_t().at(0)));
  mix(hv(c.pilot_sym_t().at(1)));
  mix(hv(c.pilot_sc()));
  mix(hv(c.pilot_sc_ind()));
  mix(hv(c.data_ind()));
  mix(hv(c.pilot()));
  mix(hv(c.beacon_ci16()));
  const double ts = c.tx_scale();
  mix(fnv(&ts, sizeof ts));
  const double hb = c.occupied_half_bw_hz();
  mix(fnv(&hb, sizeof hb));
  mix(c.symbol_data_subcarrier_num());
  for (const auto& n : c.ul_tx_fd_data_files()) {
    std::ifstream in(g_dir + "/" + n, std::ios::binary);
    const std::string ref((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    mix(fnv(ref.data(), ref.size()));
  }
  uint64_t hi = 0;
  if (c.mode_v()) {
    houdini::boundary::TxBurstInterpolator ti(true);
    const void* b[1] = {c.pilot_ci16().data()};
    const auto o = ti.run(b, 1, c.pilot_ci16().size());
    hi = fnv(o.buffs[0], o.samples * 4);
  }
  return {h, hi};
}

json load(const char* f) {
  std::ifstream in(f);
  return json::parse(in, nullptr, true, true);
}
std::string write(const json& j, const std::string& name) {
  const std::string p = g_dir + "/" + name + ".json";
  std::ofstream(p) << j.dump(1);
  return p;
}
// The Config's refusal message for `j`, or "" when it loads.
std::string refusal(const json& j, const std::string& name) {
  try {
    Config c(write(j, name), g_dir, false, false, false);
    return "";
  } catch (const std::exception& e) {
    return e.what();
  }
}

// DC-centred spectrum of the fft_size body at `body` in a slot.
std::vector<std::complex<double>> spectrum(const std::vector<cs16>& slot, size_t body, size_t n) {
  std::vector<std::complex<double>> F(n);
  for (size_t k = 0; k < n; ++k) {
    std::complex<double> a(0, 0);
    for (size_t t = 0; t < n; ++t) {
      const double ph = -2.0 * M_PI * static_cast<double>((k * t) % n) / static_cast<double>(n);
      a += std::complex<double>(slot[body + t].real(), slot[body + t].imag()) *
           std::complex<double>(std::cos(ph), std::sin(ph));
    }
    F[(k + n / 2) % n] = a;
  }
  return F;
}
double peakMag(const std::vector<cs16>& v) {
  double p = 0.0;
  for (const auto& s : v) p = std::max(p, std::abs(std::complex<double>(s.real(), s.imag())));
  return p;
}

// ---- the recorder over UDP loopback ---------------------------------------
struct Datagrams {
  std::map<uint32_t, std::vector<uint8_t>> by_magic;  // the last of each kind
};
Datagrams drain(int sock) {
  Datagrams d;
  std::vector<uint8_t> buf(1 << 17);
  for (;;) {
    const ssize_t n = ::recv(sock, buf.data(), buf.size(), MSG_DONTWAIT);
    if (n < 4) break;
    uint32_t magic;
    std::memcpy(&magic, buf.data(), 4);
    d.by_magic[magic].assign(buf.begin(), buf.begin() + n);
  }
  return d;
}
template <class T>
T at(const std::vector<uint8_t>& b, size_t off) {
  T v;
  std::memcpy(&v, b.data() + off, sizeof v);
  return v;
}
// One slot as the Houdini RX path delivers it: the mixer's spectral inversion
// makes it the conjugate of what the UE sent.
std::vector<char> packet(const std::vector<cs16>& sent, uint32_t frame, uint32_t slot, uint32_t ant) {
  std::vector<char> raw(sizeof(Packet) + sent.size() * 2 * sizeof(short));
  auto* p = new (raw.data()) Packet(static_cast<int>(frame), static_cast<int>(slot), 0, static_cast<int>(ant));
  for (size_t k = 0; k < sent.size(); ++k) {
    p->data[2 * k] = sent[k].real();
    p->data[2 * k + 1] = static_cast<short>(-sent[k].imag());
  }
  return raw;
}
}  // namespace

int main() {
  char tmpl[] = "/tmp/per_band_test.XXXXXX";
  if (::mkdtemp(tmpl) == nullptr) {
    std::perror("mkdtemp");
    return 1;
  }
  g_dir = tmpl;

  // ---- 1. backward compatibility ------------------------------------------
  // Measured on d27d5f5 (the parent of AP-85) with this same hash.
  const std::map<std::string, std::pair<uint64_t, uint64_t>> kBaseline = {
      {"files/houdini-dualband.json", {0x6ae48f30c67f58adULL, 0x3240b8504cc6cd56ULL}},
      {"files/houdini-dualband-40.json", {0x98930859a657d09fULL, 0x56854dfb102b263bULL}},
      {"files/houdini-dualband-r3a.json", {0x6ae48f30c67f58adULL, 0x3240b8504cc6cd56ULL}},
      {"files/houdini-dualband-r2.json", {0x90c874c4c6d4114aULL, 0x7a9f051a47e3d60cULL}},
      {"files/houdini-dualband-r1.json", {0x90c874c4c6d4114aULL, 0x7a9f051a47e3d60cULL}},
      {"files/houdini-r0.json", {0xa903ccc33404f877ULL, 0x0ULL}}};
  for (const auto& [f, want] : kBaseline) {
    Config c(f, g_dir, false, false, false);
    const auto got = golden(c);
    std::printf("%s: 0x%016llx / 0x%016llx, %zu band(s)\n", f.c_str(), static_cast<unsigned long long>(got.first),
                static_cast<unsigned long long>(got.second), c.num_bands());
    check(got == want && c.num_bands() == 1,
          f + ": every buffer and the TX interpolation byte-identical to the baseline "
              "[mutation: the default band built with another data seed]");
  }

  {
    // The Iris scale branch (x4 back-off) through a derived Iris variant of
    // houdini-r0.json, and the ZC generator on both sides of its prime
    // table's end (2039): both measured on d27d5f5.
    json j = load("files/houdini-r0.json");
    j["radio_type"] = "iris";
    Config iris(write(j, "iris"), g_dir, false, false, false);
    check(golden(iris).first == 0x83e1f9ef3058a45bULL,
          "Iris variant of houdini-r0.json: every buffer byte-identical to the baseline "
          "[mutation: the Houdini back-off for every platform]");
    const std::map<size_t, uint64_t> kZc = {{96, 0x080b5ad38b3740fcULL},   {304, 0x80cfc6883e8e5579ULL},
                                            {512, 0x11cb08622d354058ULL},  {1272, 0xe2a36d08fc5929b9ULL},
                                            {1596, 0x9b36fc869e789e8cULL}, {2039, 0xc889ece72b5ee7d6ULL},
                                            {2040, 0x7b4e42ad0ce8e5dfULL}, {2048, 0x06f37ea14b7995b9ULL}};
    bool same = true;
    for (const auto& [n, want] : kZc) {
      const auto z = CommsLib::getSequence(CommsLib::LTE_ZADOFF_CHU_F, n);
      same = same && fnv(z[1].data(), z[1].size() * 4, hv(z[0])) == want;
    }
    check(same, "ZC generator unchanged for 96 to 2048 tones, both sides of its table's end "
                "[mutation: the new length rule applied from 2039 (>=)]");
  }

  // ---- 2. the channel -> band map -----------------------------------------
  const char* kXw = "files/houdini-dualband-xw.json";
  Config xw(kXw, g_dir, false, false, false);
  check(golden(xw) == kBaseline.at("files/houdini-dualband.json"),
        "xw: channel A's band and every band-0 buffer equal houdini-dualband.json's "
        "[mutation: the override built into band 0]");
  check(xw.num_bands() == 2 && xw.band(0).data_num == 1596 && xw.band(1).data_num == 3240 &&
            &xw.band(1) == &xw.band(2),
        "xw: A carries 1596 tones, B and C share one 3240-tone band [mutation: one band per channel letter]");
  check(xw.bsRxBand(0).data_num == 1596 && xw.bsRxBand(1).data_num == 3240 && xw.ueTxBand(0).data_num == 1596 &&
            xw.ueTxBand(1).data_num == 3240,
        "xw: BS antenna 1 (RX C) and UE TX lane 1 (B) carry 3240 tones");
  {
    // Lanes in another order than the letters: the UE transmits B then A, the
    // BS receives C then A.
    json j = load(kXw);
    j["ue_tx_channel"] = "BA";
    j["rx_channel"] = "CA";
    Config probe(write(j, "probe"), g_dir, false, false, false);
    check(probe.ueTxBand(0).data_num == 3240 && probe.ueTxBand(1).data_num == 1596 &&
              probe.bsRxBand(0).data_num == 3240 && probe.bsRxBand(1).data_num == 1596,
          "probe: the lane's band follows its channel letter, not the lane index "
          "[mutation: band(lane) in bsRxBand / ueTxBand]");
  }
  {
    const auto m = xw.channel_half_bw_hz();
    check(m.size() == 2 && m.at(1) == 48.6e6 && m.at(2) == 48.6e6 && xw.occupied_half_bw_hz() == 23.94e6,
          "xw: the radio's per-channel half widths are B and C at 48.6 MHz, the default 23.94 "
          "[mutation: the map filled from band 0]");
  }

  // ---- 3. the wide band -----------------------------------------------------
  const OfdmBand& A = xw.band(0);
  const OfdmBand& X = xw.band(2);
  const size_t N = xw.fft_size();
  {
    const size_t start = (N - 3240) / 2;  // 428
    bool unit = true, empty = true;
    for (size_t k = 0; k < N; ++k) {
      const double m = std::hypot(X.pilot_sym_f[0][k], X.pilot_sym_f[1][k]);
      if (k >= start && k < start + 3240) unit = unit && std::fabs(m - 1.0) < 1e-5;
      else empty = empty && m == 0.0;
    }
    check(start == 428 && unit && empty, "X-band pilot: unit tones on grid indices 428..3667, nothing else");
    auto tone = [&](size_t i) { return cf(X.pilot_sym_f[0][start + i], X.pilot_sym_f[1][start + i]); };
    std::printf("X-band ZC: |tone 3229 - tone 0| %.2e, |tone 2039 - tone 0| %.2e\n", std::abs(tone(3229) - tone(0)),
                std::abs(tone(2039) - tone(0)));
    check(std::abs(tone(3229) - tone(0)) < 1e-6 && std::abs(tone(2039) - tone(0)) > 1e-3,
          "X-band pilot: the ZC period is 3229, the largest prime below 3240 "
          "[mutation: the prime table's cap at 2039]");
    double pk = 0, pw = 0;
    for (size_t k = 0; k < N; ++k) {
      const double p = std::norm(std::complex<double>(X.pilot_sym_t[0][k], X.pilot_sym_t[1][k]));
      pk = std::max(pk, p);
      pw += p / static_cast<double>(N);
    }
    const double papr = 10.0 * std::log10(pk / pw);
    std::printf("X-band pilot PAPR %.2f dB\n", papr);
    check(papr < 4.0, "X-band pilot PAPR under 4 dB (3.69 by design) [mutation: the prime table's cap, 5.26 dB]");
    check(X.pilot_ci16.size() == xw.samps_per_slot() && X.ue_data_ci16.size() == xw.samps_per_slot(),
          "X-band pilot and data slots are one slot each");
    // One pilot symbol's body in the built slot: in band and out of band.
    const size_t body = static_cast<size_t>(xw.prefix()) + xw.cp_size();
    const auto F = spectrum(X.pilot_ci16, body, N);
    double in = 0, out = 0;
    for (size_t k = 0; k < N; ++k) (k >= start && k < start + 3240 ? in : out) += std::norm(F[k]);
    const double oob = 10.0 * std::log10(out / in + 1e-30);
    std::printf("X-band pilot energy outside +-48.6 MHz: %.1f dB\n", oob);
    check(oob < -40.0, "X-band pilot sits inside +-48.6 MHz (outside < -40 dB)");
  }
  {
    check(X.data_ind.size() == 2970 && X.pilot_sc_ind.size() == 270 && X.pilot_sc_ind.front() == 434 &&
              X.pilot_sc_ind[1] - X.pilot_sc_ind[0] == 12 && X.data_ind.front() == 428 && X.data_ind.back() == 3667,
          "X-band data: 2970 data tones and 270 pilot tones (each RB's centre, 434 + 12 k)");
    const size_t body = static_cast<size_t>(xw.prefix()) + xw.cp_size();
    const auto F = spectrum(X.ue_data_ci16, body, N);
    double strong = 0, weak = 1e30;
    for (const size_t k : X.data_ind) strong = std::max(strong, std::abs(F[k]));
    for (const size_t k : X.pilot_sc_ind) weak = std::min(weak, std::abs(F[k]));
    std::printf("X-band U slot: weakest pilot tone %.1f dB under the strongest data tone\n",
                20 * std::log10(strong / weak));
    check(20 * std::log10(strong / weak) < 20.0, "X-band U slot transmits every one of its pilot tones");
    const double pp = peakMag(X.pilot_ci16), dp = peakMag(X.ue_data_ci16);
    std::printf("X-band peaks: pilot %.1f, data %.1f counts\n", pp, dp);
    check(std::fabs(pp - dp) <= 2.0,
          "X-band U slot peaks at its own pilot's peak [mutation: the data slot not normalized to its pilot's peak]");
  }

  // ---- 4. levels --------------------------------------------------------------
  {
    const double pa = peakMag(A.pilot_ci16), px = peakMag(X.pilot_ci16);
    const double per_tone_db = 20.0 * std::log10(static_cast<double>(X.tx_scale) / A.tx_scale);
    std::printf("pilot peaks: sub-6 %.1f, X-band %.1f counts; per-tone amplitude X-band vs 133 RB: %.3f dB\n", pa, px,
                per_tone_db);
    check(std::fabs(pa - 16384.0) <= 3.0 && std::fabs(px - 16384.0) <= 3.0,
          "each band's pilot peaks at half full scale, peak |z| [mutation: the X-band built with band 0's tx_scale]");
    auto rms = [](const std::vector<cs16>& v) {
      double p = 0;
      for (const auto& s : v) p += std::norm(std::complex<double>(s.real(), s.imag()));
      return std::sqrt(p / static_cast<double>(v.size()));
    };
    const double u_db = 20.0 * std::log10(rms(X.ue_data_ci16) / rms(A.ue_data_ci16));
    const double p_db = 20.0 * std::log10(rms(X.pilot_ci16) / rms(A.pilot_ci16));
    std::printf("slot RMS X-band vs 133 RB: pilot %+.3f dB, U slot %+.3f dB (per data tone %+.3f dB)\n", p_db, u_db,
                u_db - 10.0 * std::log10(3240.0 / 1596.0));
    check(p_db > 0.1 && p_db < 0.4 && u_db > 0.7 && u_db < 1.2,
          "slot RMS X-band vs 133 RB: pilot +0.23 dB, U slot +0.93 dB (the P10 prediction's source)");
    check(per_tone_db > -3.0 && per_tone_db < -2.7,
          "the X-band's per-tone amplitude is 2.7 to 3.0 dB below the sub-6's (the P5 prediction's source)");
  }

  // ---- 5. refusals --------------------------------------------------------------
  {
    auto refused = [](const std::string& msg, const char* want) { return msg.find(want) != std::string::npos; };
    {
      // The Houdini framer cuts one pilot slot; the slots mode needs the beacon slot.
      json k = load("files/houdini-dualband-xw-steer-slots.json");
      std::string two = k["frame_schedule"][0].get<std::string>();
      two[two.find('G', two.find('P'))] = 'P';
      k["frame_schedule"] = {two};
      k["ue_frame_schedule"] = {std::string(two.size(), 'G')};
      check(refused(refusal(k, "r_twop"), "exactly one pilot slot"),
            "refused: a Houdini schedule with two pilot slots [mutation: the one-pilot check removed]");
      k = load("files/houdini-dualband-xw-steer-slots.json");
      std::string nob = k["frame_schedule"][0].get<std::string>();
      nob[nob.find('B')] = 'G';
      k["frame_schedule"] = {nob};
      check(refused(refusal(k, "r_nobeacon"), "has none"),
            "refused: a slots-mode schedule without a beacon slot [mutation: the beacon-slot check removed]");
      check(refusal(load("files/houdini-dualband-xw-steer-slots.json"), "r_demo_ok").empty(),
            "the demo config itself loads [mutation: the checks refuse a valid schedule]");
    }
    json j = load("files/houdini-r0.json");
    j["channel_ofdm_data_num"] = {{"B", 1596}};
    check(refused(refusal(j, "r_nomodev"), "needs the mode-V converter plan"),
          "refused: the key without mode V [mutation: the mode-V check removed]");
    j = load(kXw);
    j["channel_ofdm_data_num"] = {{"B", 3240}};
    const std::string one = refusal(j, "r_bonly");
    check(refused(one, "channels B and C share the NCO"),
          "refused: B widened without C, naming both [mutation: the same-NCO rule removed] (" + one + ")");
    j["channel_ofdm_data_num"] = {{"B", 3300}, {"C", 3300}};
    check(refused(refusal(j, "r_wide"), "beyond the decimator's passband"),
          "refused: 3300 tones (+-49.5 MHz) [mutation: the passband rule removed]");
    j["channel_ofdm_data_num"] = {{"B", 3276}, {"C", 3276}};
    check(refusal(j, "r_273").empty(), "273 RB (3276 tones, +-49.14 MHz) loads");
    for (const json& bad : {json{{"B", 0}, {"C", 0}}, json{{"B", 5004}, {"C", 5004}}, json{{"B", 3240.5}},
                            json{{"B", -3}}, json{{"E", 3240}}, json{{"BC", 3240}}, json{{"B", 3241}, {"C", 3241}},
                            json{{"B", 3246}, {"C", 3246}}}) {
      j["channel_ofdm_data_num"] = bad;
      check(refused(refusal(j, "r_bad"), "must be ONE letter A-D"),
            "refused: channel_ofdm_data_num " + bad.dump() + " [mutation: the key/value check removed]");
    }
    j = load(kXw);
    j["fft_size"] = 64;
    j["ofdm_data_num"] = 52;
    j["cp_size"] = 16;
    check(refused(refusal(j, "r_fft64"), "needs a Zadoff-Chu pilot"), "refused: the key at fft 64");
  }

  {
    // Recording mode refuses two bands at startup (the recorder's init runs
    // on the main thread, in the RecorderThread constructor).
    ::unsetenv("HOUDINI_CSI_UDP");
    bool threw = false;
    try {
      Sounder::RecorderWorker w(&xw, 0, 2);
      w.init();
    } catch (const std::invalid_argument&) {
      threw = true;
    }
    check(threw, "recording mode (HDF5) is refused with two bands [mutation: the refusal removed]");
  }

  // ---- 6. the BS recorder, end to end ---------------------------------------
  {
    const int rx = ::socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    socklen_t al = sizeof a;
    const int rcv = 4 << 20;
    ::setsockopt(rx, SOL_SOCKET, SO_RCVBUF, &rcv, sizeof rcv);
    if (rx < 0 || ::bind(rx, reinterpret_cast<sockaddr*>(&a), sizeof a) != 0 ||
        ::getsockname(rx, reinterpret_cast<sockaddr*>(&a), &al) != 0) {
      check(false, "recorder: a loopback UDP socket");
    } else {
      const std::string dst = "127.0.0.1:" + std::to_string(ntohs(a.sin_port));
      ::setenv("HOUDINI_CSI_UDP", dst.c_str(), 1);
      Sounder::RecorderWorker w(&xw, 0, 2);
      w.init();
      const uint32_t kFrame = 7, kP = 2, kU = 3;  // the schedule's P and U slots
      check(xw.isPilot(0, 0, kP) && xw.isUlData(0, 0, kU), "recorder: slots 2 and 3 are the schedule's P and U");
      struct Want {
        uint32_t ant, occ, ch;
        double bw, fc;
      };
      std::vector<double> level;  // each antenna's median |H|
      for (const Want& want : {Want{0, 1596, 0, 47.88e6, 2425e6}, Want{1, 3240, 2, 97.2e6, 4380e6}}) {
        const OfdmBand& b = xw.bsRxBand(want.ant);
        auto pp = packet(b.pilot_ci16, kFrame, kP, want.ant);
        w.record(0, reinterpret_cast<Packet*>(pp.data()), kBS);
        auto d = drain(rx);
        const auto& csi = d.by_magic[0x43534932u];
        const auto& met = d.by_magic[0x4D455431u];
        // |H| is the received tone over the known pilot tone: flat at the
        // transmitted per-tone amplitude (the FFT is unnormalized) on the
        // band's tones, and exactly 0 where the reference has no tone.
        size_t on = 0, off_zero = 0;
        std::vector<double> mags;
        if (csi.size() >= 24) {
          const uint32_t nsc = at<uint32_t>(csi, 12);
          for (uint32_t k = 0; k < nsc; ++k) {
            const double m = std::hypot(at<float>(csi, 24 + 8 * k), at<float>(csi, 28 + 8 * k));
            if (m > 1e-9) {
              ++on;
              mags.push_back(m);
            } else {
              ++off_zero;
            }
          }
        }
        std::sort(mags.begin(), mags.end());
        const double med = mags.empty() ? 0.0 : mags[mags.size() / 2];
        const double worst = mags.empty() ? 1.0 : std::max(med - mags.front(), mags.back() - med) / med;
        level.push_back(med);
        std::printf("recorder ant %u: CSI2 |H| on %zu tones (median %.1f, worst deviation %.2e), MET1 %u tones %.2f MHz ch %u\n",
                    want.ant, on, med, worst, met.size() >= 44 ? at<uint32_t>(met, 16) : 0u,
                    met.size() >= 44 ? at<double>(met, 36) / 1e6 : 0.0, met.size() >= 44 ? at<uint32_t>(met, 8) : 0u);
        check(on == want.occ && off_zero == N - want.occ && worst < 1e-2,
              "recorder ant " + std::to_string(want.ant) + ": |H| flat (1 %) on exactly its band's " +
                  std::to_string(want.occ) + " tones [mutation: band 0's pilot for every antenna]");
        check(met.size() >= 44 && at<uint32_t>(met, 8) == want.ch && at<uint32_t>(met, 16) == want.occ &&
                  std::fabs(at<double>(met, 36) - want.bw) < 1.0 && at<double>(met, 20) == want.fc,
              "recorder ant " + std::to_string(want.ant) + ": MET1 channel, occupied tones, bandwidth and NCO");
        auto up = packet(b.ue_data_ci16, kFrame, kU, want.ant);
        w.record(0, reinterpret_cast<Packet*>(up.data()), kBS);
        d = drain(rx);
        const auto& cns = d.by_magic[0x434E5331u];
        double err = 0;
        uint32_t npt = cns.size() >= 20 ? at<uint32_t>(cns, 12) : 0u;
        for (uint32_t i = 0; i < npt; ++i) {
          const cf x(at<float>(cns, 20 + 8 * i), at<float>(cns, 24 + 8 * i));
          const cf ideal((x.real() >= 0 ? 1.0f : -1.0f) / std::sqrt(2.0f), (x.imag() >= 0 ? 1.0f : -1.0f) / std::sqrt(2.0f));
          err += std::norm(x - ideal);
        }
        const double mer = npt > 0 ? -10.0 * std::log10(err / npt + 1e-30) : 0.0;
        std::printf("recorder ant %u: constellation %u points, MER %.1f dB\n", want.ant, npt, mer);
        check(npt > 100 && mer > 40.0,
              "recorder ant " + std::to_string(want.ant) +
                  ": its U slot decodes as clean QPSK (MER > 40 dB) [mutation: band 0's data and pilot tones for every antenna]");
      }
      const double ratio_db = level.size() == 2 ? 20.0 * std::log10(level[1] / level[0]) : 0.0;
      const double want_db = 20.0 * std::log10(static_cast<double>(X.tx_scale) / A.tx_scale);
      std::printf("recorder: X-band |H| %.3f dB against the sub-6's, the per-tone amplitude ratio %.3f dB\n", ratio_db,
                  want_db);
      check(std::fabs(ratio_db - want_db) < 0.02,
            "recorder: the X-band antenna's |H| sits at its per-tone amplitude ratio to the sub-6's (P5's instrument)");
      ::unsetenv("HOUDINI_CSI_UDP");
    }
    if (rx >= 0) ::close(rx);
  }

  std::printf("%s: %d failure(s)\n", g_fail == 0 ? "ALL PASS" : "FAILED", g_fail);
  return g_fail == 0 ? 0 : 1;
}
