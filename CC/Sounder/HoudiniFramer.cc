/** @file HoudiniFramer.cc
  * @brief The Houdini base-station framer, moved out of BaseRadioSet.cc
  *        (seam step S2). The mechanics and their measured reasons are the
  *        ones recorded in DEMO_VERIFICATION.md 3.x and 4.x; nothing here
  *        changed in the move.
  *
  * RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
*/
#include "include/HoudiniFramer.h"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <complex>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <SoapySDR/Errors.hpp>

#include "include/logger.h"
#include "include/macros.h"
#include "include/rx_gap_sink.h"
#include "include/utils.h"
#include "sync/beacon_shape.h"

void HoudiniFramer::buildBeacon(std::vector<int16_t>& iq) {
  constexpr int kReplayDepth = 4096;  // Houdini TX replay RAM depth (samples)

  // Rebuild the STS+gold core of config's beacon (indices [prefix, prefix+
  // beacon_size) skip the zero pre/postfix). Conjugate it: the matched-NCO R2C
  // mixer delivers the beacon conjugated and receiver.cc::syncSearch feeds the
  // raw RX to find_beacon, so pre-conjugating the TX cancels it.
  //
  // NO host upsampling: the beacon replays at the APP rate (BaseRadioSet opens
  // the beacon Radio's TX at cfg_->rate(), not the DAC max), so the RFDC's own
  // interpolation carries it to the DAC. Placing the beacon_size (~496) core at
  // the head of the 4096-deep RAM and leaving the rest SILENT makes an ISOLATED
  // beacon that recurs once per FRAME (122880 samples, 1 ms): it fits inside the
  // client's
  // detect window AND keeps find_beacon's trailing-energy threshold low, so the
  // SHARP native-rate 2-rep gold peak clears it at corr_scale=1. (The old x8
  // upsample + DAC-max replay recurred every 512 samples -- dense -- and smeared
  // the gold, forcing corr_scale~100.)
  const auto& bc = cfg_->beacon_ci16();
  const int p = cfg_->prefix();
  const int n = cfg_->beacon_size();
  std::vector<std::complex<float>> loop(kReplayDepth, std::complex<float>(0, 0));
  for (int k = 0; k < n && k < kReplayDepth; ++k) {
    loop[k] = std::conj(std::complex<float>(
        static_cast<float>(bc.at(p + k).real()),
        static_cast<float>(bc.at(p + k).imag())));
  }
  float peak = 1e-30f;
  for (const auto& v : loop) peak = std::max(peak, std::abs(v));
  // TRANSMIT AMPLITUDE, AS A FRACTION OF FULL SCALE. 0.6 was the only value
  // that had ever been used, hard-coded, and that made the received level the
  // one axis this bench could not vary -- so a detector claim that depends on
  // level (AP-34: the shipped threshold is 4th order in amplitude over 2nd, and
  // therefore not scale invariant) could be measured offline and never on
  // silicon. HOUDINI_BEACON_FS makes it a knob.
  //
  // DIAGNOSTIC, NOT A LINK BUDGET. Lowering it weakens the beacon and nothing
  // else, which is the point: it is the cheapest available stand-in for path
  // loss on a cabled bench. Clamped to (0, 1] because above 1.0 the samples
  // would clip into a spectrally-splattered beacon that measures the clipping
  // rather than the level (the conversion below clamps, it does not wrap).
  // sync.beacon.tx_full_scale (HOUDINI_BEACON_FS as a logged override while
  // allow_env_overrides holds), range-checked by SyncConfig to (0.001, 1].
  const float fs_frac = static_cast<float>(cfg_->sync().beacon.tx_full_scale);
  if (cfg_->sync().wasSet("beacon.tx_full_scale") &&
      std::fabs(static_cast<double>(fs_frac) - 0.6) > 1e-6) {
    MLPD_WARN("beacon transmitted at %.1f%% of full scale instead of the "
              "default 60%% (sync.beacon.tx_full_scale). Diagnostic setting.\n",
              static_cast<double>(fs_frac) * 100.0);
  }
  const size_t n_load = loop.size();
  iq.assign(n_load * 2, 0);
  // Peak-scaled to fs_frac of full scale, then clamped: with fs_frac <= 1 the
  // clamp never fires, and if it ever did the beacon would clip rather than
  // wrap (the same rule as Utils::saturateToInt16).
  const auto clamp16 = [](double v) {
    if (v != v) return static_cast<int16_t>(0);
    return static_cast<int16_t>(std::lround(std::max(-32768.0, std::min(32767.0, v))));
  };
  for (size_t k = 0; k < n_load; ++k) {
    iq[2 * k] = clamp16(loop[k].real() / peak * fs_frac * 32767);
    iq[2 * k + 1] = clamp16(loop[k].imag() / peak * fs_frac * 32767);
  }
  if (std::getenv("HOUDINI_DUMP_BEACON")) {
    const std::string path = Utils::dumpPath("beacon_ram.bin");
    FILE* f = std::fopen(path.c_str(), "wb");
    if (f == nullptr) {
      MLPD_WARN("HOUDINI_DUMP_BEACON: cannot open %s (%s)\n", path.c_str(), std::strerror(errno));
    }
    if (f) {
      std::fwrite(iq.data(), sizeof(int16_t), iq.size(), f);
      std::fclose(f);
      MLPD_INFO("HOUDINI_DUMP_BEACON: wrote %zu int16 to %s\n",
                iq.size(), path.c_str());
    }
  }
}

void HoudiniFramer::armReplayBeacon(void) {
  std::vector<int16_t> iq;
  buildBeacon(iq);
  const size_t n_load = iq.size() / 2;

  // Load the replay RAM + arm free-running on the beacon radio's TX. The TX
  // stream is bound to the BS channel (the wired DAC), so xmit targets it. RX
  // is NOT activated here: it would sit unread (overflowing) until the caller is
  // ready to receive -- activateHoudiniRx() starts it on demand.
  const void* buffs[1] = {iq.data()};
  long long t0 = 0;
  for (size_t c = 0; c < radios_.size(); ++c) {
    for (size_t i = 0; i < radios_.at(c).size(); ++i) {
      if (i != cfg_->beacon_radio()) continue;
      Radio* r = radios_.at(c).at(i).get();
      // The load is the beacon: a refused or short load (the plugin returns an
      // error while the replay bank's level arm is still set) must stop the
      // bring-up here, not arm the loop over stale RAM and play a beacon that
      // is not the one built above (DEMO_VERIFICATION 4.24, SH-348).
      const int loaded = r->xmit(buffs, static_cast<int>(n_load), 0, t0);
      if (loaded != static_cast<int>(n_load)) {
        throw std::runtime_error(
            "Houdini beacon replay RAM load refused: " +
            std::string(loaded < 0 ? SoapySDR::errToStr(loaded) : "short load") +
            " (" + std::to_string(loaded) + " of " + std::to_string(n_load) +
            " samples)");
      }
      r->activateXmit();  // arm free-running loop
      MLPD_INFO(
          "Houdini BS beacon armed: %zu-sample app-rate replay loop (isolated "
          "beacon, period %zu) on cell %zu radio %zu\n",
          n_load, n_load, c, i);
    }
  }
}

void HoudiniFramer::start(void) {
  // Start the BS RX streams on demand (the reverse link / UE pilots). Kept
  // separate from beacon arming so the RX stream is not left overflowing while
  // the caller is busy elsewhere (e.g. waiting for the UE to acquire).
  for (size_t c = 0; c < radios_.size(); ++c)
    for (size_t i = 0; i < radios_.at(c).size(); ++i)
      radios_.at(c).at(i)->activateRecv();
}

// ---- Houdini native-TDD framer (bs_hw_framer + radio_type=houdini) ----------
namespace {
// 3.125 us anchor/strobe grid. NB the same driver constant appears as
// houdini::sync::kHoudiniStrobeOffsetTicks (the one definition), a local kTddGridTicks in
// clientTxPilots, and 3125 ns in ClientRadioSet::radioTx -- all four must
// move together if the driver grid ever changes (Opus review LOW).
constexpr long long kTddGridTicks = houdini::sync::kHoudiniStrobeOffsetTicks;
constexpr long long kTddArmMargin = 36864000;  // ~300 ms of ticks
}  // namespace

// The full teardown ladder. Abort alone is NOT enough, twice over (measured,
// DEMO_VERIFICATION.md 3.2 + 4.24): aborting a RUNNING framer latches
// gates_held and every later arm is REFUSED until gate_release; and skipping
// TX_CLEAR can leave the TX bank in a state where strobe bursts ack and count
// as played but NO RF leaves the DAC ("a consumed arm parks the source until
// the arm clears or TX_CLEAR").
void HoudiniFramer::tddLadder(SoapySDR::Device* dev, bool skip_tx_clear) {
  dev->writeSetting("TDD_CMD", "abort");
  if (skip_tx_clear) {
    // AP-73 / SH-348: the measurement that asks whether the reload after an
    // abort lands without this pulse. A diagnostic, never a shipped setting.
    MLPD_WARN("DIAGNOSTIC houdini_diag_skip_tx_clear: TX_CLEAR skipped in the ladder\n");
  } else {
    dev->writeRegister("RFCORE", 0x24, 1);  // TX_CLEAR_ALL pulse
    dev->writeRegister("RFCORE", 0x24, 0);
  }
  dev->writeSetting("TDD_CMD", "gate_release");
}

long long HoudiniFramer::armTddOnce(SoapySDR::Device* dev,
                                      const std::function<void()>& resetup,
                                      long long symbol_ticks,
                                      long long symbols_per_frame) {
  const std::string arm = "symbol_ticks=" + std::to_string(symbol_ticks) +
                          ",symbols_per_frame=" + std::to_string(symbols_per_frame) +
                          ",margin=" + std::to_string(kTddArmMargin);
  long long epoch = 0;
  bool accepted = false;
  for (int attempt = 0; attempt < 4 && !accepted; ++attempt) {
    // On the current stack a refused arm THROWS (SH-333) instead of returning
    // accepted=0. A throwing WRITE must never be trusted via the readback:
    // failures before the device stores its last-arm record leave a STALE
    // string (possibly a previous run's accepted=1), so a throw always
    // re-ladders and retries (Opus review finding 3).
    try {
      dev->writeSetting("TDD_ARM", arm);
    } catch (const std::exception& e) {
      MLPD_WARN("TDD_ARM attempt %d refused: %s\n", attempt, e.what());
      tddLadder(dev, cfg_->diag_skip_tx_clear());
      // The ladder's TX_CLEAR invalidates the loaded replay RAM / schedule /
      // strobe state (the setup path itself runs the ladder BEFORE loading
      // them, for exactly that reason). Retrying the arm without re-running
      // the setup could arm a framer with a cleared beacon: counters healthy,
      // "armed" logged, no RF -- the silent class the ladder exists to
      // prevent (Opus review H1).
      if (resetup) resetup();
      continue;
    }
    std::stringstream ss(dev->readSetting("TDD_ARM"));
    std::string tok;
    while (ss >> tok) {
      const auto eq = tok.find('=');
      if (eq == std::string::npos) continue;
      const std::string k = tok.substr(0, eq), v = tok.substr(eq + 1);
      if (k == "epoch") epoch = std::stoll(v);
      if (k == "accepted") accepted = (v == "1");
    }
    if (!accepted) {
      tddLadder(dev, cfg_->diag_skip_tx_clear());
      if (resetup) resetup();  // same reason as the throw path above
    }
  }
  if (!accepted) throw std::runtime_error("Houdini TDD_ARM rejected");
  return epoch;
}

void HoudiniFramer::armTdd(void) {
  // Slot-granular ring: one TDD symbol per sounder slot (symbol_ticks =
  // samps_per_slot, symbols_per_frame = slot_per_frame), '6' on the beacon
  // slot, '2' on every other slot. Verified on silicon 2026-08-30
  // (DEMO_VERIFICATION.md 4.12): the ring arms and the strobe plays exactly
  // one burst per frame. Every non-beacon entry must keep the rx bit set:
  // a gate close ABANDONS a running continuous capture (driver contract,
  // D4 window-pump + overlength-abandon), and per-window host pumping costs
  // ~100 ms RPC per window, unusable at 1 ms frames. True rx-only-in-P/U
  // gating therefore needs a driver capability (hardware-chained windowed
  // RX); until then the wire carries the whole frame and the guards are
  // silent AIR, not absent DATA (DEMO_VERIFICATION.md section 3/4).
  //
  // The strobe plays ONE beacon copy per frame (loops=1, len = beacon core):
  // the old loops=forever filled a 0.5 ms symbol with ~15 copies, which made
  // the UE's frame anchor ambiguous by k x 4096 samples per restart.
  //
  // NB (houdini_beacon_ab.py, .21->.22): the SAME beacon RAM scored by the
  // client's gold correlation gives 44.5 dB via the framer strobe vs only 10.2 dB
  // via a continuous activateXmit replay -- the strobe is SHARPER, not distorted
  // (an earlier "strobe distorted ~11 dB" note was the continuous mode mislabeled).
  // And a continuous replay can't coexist with the framer anyway: arming the framer
  // silences activateXmit (-33 dB) and any tx_gate schedule is arm-rejected without
  // a strobe. So the strobe is the only way to get beacon + rx_gate on one board.
  htdd_symbol_ticks_ = static_cast<long long>(cfg_->samps_per_slot());
  htdd_tick_rate_ = cfg_->rate();

  std::vector<int16_t> iq;
  buildBeacon(iq);
  const size_t n_load = iq.size() / 2;
  const void* buffs[1] = {iq.data()};

  for (size_t c = 0; c < radios_.size(); ++c) {
    for (size_t i = 0; i < radios_.at(c).size(); ++i) {
      if (i != cfg_->beacon_radio()) continue;
      Radio* r = radios_.at(c).at(i).get();
      auto* dev = r->RawDev();
      tddLadder(dev, cfg_->diag_skip_tx_clear());  // full ladder, never abort alone (3.2 + 4.24)

      // The sounder pilot/uplink slots to receive (tag with these indices).
      const std::string& sched = cfg_->bs_array_frames().at(c).at(i);
      htdd_rx_slots_.clear();
      for (size_t s = 0; s < sched.size(); ++s) {
        const char ch = sched.at(s);
        if (ch == 'P' || ch == 'R' || ch == 'U' || ch == 'N')
          htdd_rx_slots_.push_back(s);
        if (ch == 'P') htdd_pilot_slot_ = s;  // CSI reference slot
      }
      // TDD frame must EQUAL the sounder frame: the beacon fires once per TDD
      // frame, so a longer TDD frame makes the beacon period differ from the UE's
      // (sounder) frame and the pilot/data walk relative to the beacon -> noisy CSI.
      // One symbol per sounder slot, beacon strobe on the schedule's B slot,
      // rx bit on EVERY entry (see the function comment: a closed gate kills
      // the continuous capture; the rx slots are still extracted from the
      // continuous read in houdiniTddRx).
      const size_t spf_tdd = cfg_->slot_per_frame();
      const size_t b_pos = sched.find('B');
      const size_t beacon_slot = (b_pos == std::string::npos) ? 0 : b_pos;
      // Driver register field widths: symbol_ticks u16, symbols_per_frame
      // 13 bits (TDD framer contract).
      if (htdd_symbol_ticks_ > 0xFFFF || spf_tdd > 0x1FFF) {
        throw std::runtime_error(
            "armHoudiniTdd: slot-granular ring out of framer range "
            "(symbol_ticks=" + std::to_string(htdd_symbol_ticks_) +
            ", spf=" + std::to_string(spf_tdd) + ")");
      }
      std::string tdd(spf_tdd, '2');    // every slot rx-gates
      // NB the ring's last rx entry abuts the tx-ish '6' across the frame
      // wrap, so the driver logs the HS-184 warm-return warning twice per
      // arm. ACCEPTED deliberately: a '0' guard would close the rx gate and
      // abandon the continuous capture (see the function comment); the
      // warning is about X-band T/R-switch timing, moot on this cabled
      // bench. (The guarded probe ring in DEMO_VERIFICATION.md 4.12 avoided
      // the warning; the shipped ring does not.)
      tdd.at(beacon_slot) = '6';        // + beacon strobe on the B slot
      htdd_frame_ticks_ = static_cast<long long>(spf_tdd) * htdd_symbol_ticks_;

      // PHYSICAL TX channel for the strobe (beacon_channel() is the logical index
      // within bs_channel; TDD_REPLAY_STROBE and the loaded RAM are on the real
      // DAC, e.g. bs_channel "B" -> ch1 = the cabled DAC_A). Using the logical 0
      // fired the strobe on ch0 (DAC_B, not cabled) so the beacon never reached
      // the UE.
      const auto bs_chans = Utils::strToChannels(cfg_->bs_channel());
      const size_t tx_ch =
          bs_chans.empty()
              ? 0
              : bs_chans.at(std::min(static_cast<size_t>(cfg_->beacon_channel()),
                                     bs_chans.size() - 1));
      // The load/schedule/strobe sequence, re-runnable: the arm retry loop
      // re-invokes it after every teardown ladder (Opus review H1 -- a
      // ladder invalidates this state, so a bare arm retry could arm a
      // beaconless framer).
      const auto setup_framer = [&]() {
        long long t0 = 0;
        // Explicitly disarm any strobe left armed by a previous (e.g. killed)
        // run -- TDD_CMD abort alone doesn't release it, and the replay RAM
        // can't be filled while strobe mode is enabled ("Disarm first").
        // Safe when nothing is armed.
        // Not fatal (nothing armed is the normal case) but never silent: a
        // strobe-off that does not take leaves the strobe bit set, the fill
        // below is then refused, and the strobe-on after it is re-accepted --
        // the one path that arms over stale RAM with every write answered
        // (SH-348's candidate for 4.24). The plugin's refusal text names the
        // bit, so the warning and the load's throw together say which.
        try {
          dev->writeSetting("TDD_REPLAY_STROBE",
                            "ch" + std::to_string(tx_ch) + ":off");
        } catch (const std::exception& e) {
          MLPD_WARN("TDD_REPLAY_STROBE ch%zu:off before the load refused: %s\n",
                    tx_ch, e.what());
        } catch (...) {
          MLPD_WARN("TDD_REPLAY_STROBE ch%zu:off before the load refused\n", tx_ch);
        }
        // A refused or short load (the plugin refuses the fill while the
        // replay bank's level arm is set) stops the sequence here: the arm
        // retry loop runs the ladder, which clears that arm, and re-invokes
        // this setup. Arming over stale RAM would play a beacon that is not
        // the one built above (DEMO_VERIFICATION 4.24, SH-348).
        const int loaded = r->xmit(buffs, static_cast<int>(n_load), 0, t0);
        if (loaded != static_cast<int>(n_load)) {
          throw std::runtime_error(
              "Houdini beacon replay RAM load refused: " +
              std::string(loaded < 0 ? SoapySDR::errToStr(loaded)
                                     : "short load") +
              " (" + std::to_string(loaded) + " of " +
              std::to_string(n_load) + " samples)");
        }
        dev->writeSetting("TDD_SCHED", tdd);
      // ONE burst per frame (loops=1) spanning the usable symbol: the RAM is
      // [beacon core 496][zeros], and len (2-sample units, driver contract)
      // covers (symbol - offs) samples, so the slot's DAC input is the beacon
      // followed by OUR zeros up to the window close, not engine-idle output.
      // (D5 measured post-burst idle as silent on silicon, but explicit zeros
      // remove the reliance.) Single-copy removes the k x 4096 anchor
      // ambiguity the old loops=forever multi-copy fill created; the UE's
      // acquisition just needs more detect windows to first see it
      // (~1 in 12.9 windows carries the beacon now). Samples == ticks at the
      // one supported rate (122.88 MSPS; the whole layer assumes it).
      const size_t span_units =
          static_cast<size_t>((htdd_symbol_ticks_ - kTddGridTicks) / 2);
      const size_t len_units = std::max<size_t>(
          (static_cast<size_t>(cfg_->beacon_size()) + 1) / 2,
          std::min(n_load / 2, span_units));
        dev->writeSetting("TDD_REPLAY_STROBE",
                          "ch" + std::to_string(tx_ch) +
                              ":len=" + std::to_string(len_units) +
                              ",loops=1,offs=" + std::to_string(kTddGridTicks));
      };
      setup_framer();
      htdd_epoch_ = armTddOnce(dev, setup_framer, htdd_symbol_ticks_,
                                  static_cast<long long>(spf_tdd));
      htdd_rx_cursor_ = 0;
      htdd_last_win_tick_ = 0;
      MLPD_INFO(
          "Houdini BS TDD armed: sched=%s epoch=%lld frame=%lld ticks, "
          "%zu pilot slot(s) %s beacon strobe %zu samp\n",
          tdd.c_str(), htdd_epoch_, htdd_frame_ticks_, htdd_rx_slots_.size(),
          htdd_rx_slots_.empty() ? "(none)" : "", n_load);
    }
  }
}

int HoudiniFramer::rx(size_t radio_id, void* const* buffs,
                               long long& frameTime) {
  if (htdd_rx_slots_.empty()) return 0;
  // Bound the BS capture run: loopRecv (unlike the client loop) doesn't stop at
  // max_frame, and an unbounded frame_id would grow the recorder's HDF5 dataset
  // without limit (-> extend crash at close). Returning <0 makes loopRecv set
  // running(false) and shut down cleanly.
  const long long max_frame = static_cast<long long>(cfg_->max_frame());
  if (max_frame > 0 && htdd_frame_counter_ >= max_frame) return -1;
  Radio* r = radios_.at(0).at(radio_id).get();
  const int n = static_cast<int>(cfg_->samps_per_slot());
  const size_t K = htdd_rx_slots_.size();  // rx slots/frame (pilot P + uplink U...)
  const size_t cur = htdd_rx_cursor_;

  // Non-first rx slot: serve it from the per-frame cache filled on cursor 0 (one
  // continuous read yields every rx slot of the frame). Shares the frame_id so the
  // recorder places P and U in the same frame.
  if (cur != 0) {
    std::memcpy(buffs[0], htdd_slot_cache_.data() + cur * static_cast<size_t>(n) * 2,
                static_cast<size_t>(n) * 4);
    const size_t slot = htdd_rx_slots_.at(cur);
    htdd_rx_cursor_ = (cur + 1) % K;
    frameTime = (htdd_cache_frame_ << 32) | (static_cast<long long>(slot) << 16);
    return n;
  }

  // cursor 0: CONTINUOUS framer receive (the Iris model -- framer armed once, RX
  // activated once in radioStart, no per-window arm/teardown). Read a bit more than
  // one frame + the pilot->last-slot span so every rx slot is fully contained
  // regardless of the (unaligned) read phase.
  const int span = (K > 1)
                       ? static_cast<int>(htdd_rx_slots_.back() -
                                          htdd_rx_slots_.front())
                       : 0;
  const int fn = static_cast<int>(htdd_frame_ticks_) + (span + 3) * n;
  htdd_cap_buf_.resize(static_cast<size_t>(fn) * 2);
  void* cb[1] = {htdd_cap_buf_.data()};
  long long ft = 0;
  const int cg = r->recv(cb, fn, ft);
  // This one read backs every rx slot of the frame, so its padding applies to all of
  // them; latch it before any later recv on this radio overwrites the radio's copy.
  htdd_frame_pad_ = r->lastPadSamples();
  if (cg < fn) {
    // Short read: the frame is not fully covered. align_slot() clamps a slot start
    // to cg, so any slot past the received data would be extracted from the tail
    // and look perfectly valid downstream. Fold the shortfall into the frame's
    // untrusted count so consumers refuse those slots instead of trusting them.
    // With rx_gap_break on (the driver default) a short return is how a dropped
    // packet surfaces, so this is the normal loss path, not an exotic one (AP-10).
    htdd_frame_pad_ += static_cast<size_t>(fn - cg);
  }
  if (cg < n) return (cg < 0) ? cg : 0;
  const int16_t* s = htdd_cap_buf_.data();
  std::vector<double> cse(static_cast<size_t>(cg) + 1, 0.0);
  for (int i = 0; i < cg; ++i) {
    const double re = s[2 * i], im = s[2 * i + 1];
    cse[i + 1] = cse[i] + re * re + im * im;
  }
  double best = 0.0;
  double worst = -1.0;
  int at = 0;
  for (int t = 0; t + n <= cg; t += 128) {
    const double e = cse[t + n] - cse[t];
    if (e > best) { best = e; at = t; }
    if (worst < 0.0 || e < worst) worst = e;
  }
  // The read spans ~1.17 frames, so when the pilot lands in the first few
  // slots of the buffer a SECOND copy (next frame) is also fully contained
  // near the tail -- and the densest-window search picks between two
  // equal-energy copies by noise. The tail copy leaves no room for the
  // frame's later rx slots: u_start ran past cg and align_slot's clamp
  // served tail junk (noise, partial bursts, or the pilot itself) as the
  // data slot -- the ~2% garbage-constellation class (measured: frame 5220
  // p_start=139160 pu_spacing_err=-8088 with the pilot burst in the "U"
  // dump). Re-map to the earlier copy, which always fits with its whole
  // rx-slot span.
  {
    const int fr_t = static_cast<int>(htdd_frame_ticks_);
    const int span_n = (static_cast<int>(htdd_rx_slots_.back()) -
                        static_cast<int>(htdd_rx_slots_.front()) + 2) * n;
    while (at + span_n > cg && at >= fr_t) at -= fr_t;
  }
  const double pilot_rms = std::sqrt(best / n);
  // Noise floor from the QUIETEST slot-length window of the same read (27 of
  // 30 slots are guard, so it measures the true floor, ~6 rms on this bench).
  // The old gate compared against 4x the WHOLE-read mean, but that mean
  // includes the pilot+data burst energy itself, which put the threshold
  // right on top of a healthy pilot (measured: rms 1392 vs 4x355 = 1420) --
  // the gate flapped on ~half of all healthy frames, and the quiet path's
  // delivery painted the 1-2 s garbage blips on the dashboard
  // [user 2026-08-30]. Densest-vs-quietest separates by ~47 dB instead.
  const double floor_rms = std::sqrt(std::max(worst, 0.0) / n);
  // Presence gate: skip frames where no UE signal is on-air (don't advance the
  // frame counter -> the first real frame lands at recorder frame 0). A LOSS
  // of pilots mid-run is reported loudly [user 2026-08-30]: the UE pausing
  // its schedule (e.g. the AP-18 resync escalation hunting for a lost beacon)
  // shows up here as a quiet streak, and the BS should say so rather than
  // skip silently.
  if (pilot_rms < 120.0 || pilot_rms < 4.0 * floor_rms) {
    ++htdd_quiet_streak_;
    constexpr size_t kQuietWarnFrames = 200;  // ~0.2 s at 1 kHz frames
    if (htdd_frame_counter_ > 0 &&
        (htdd_quiet_streak_ == kQuietWarnFrames ||
         (htdd_quiet_streak_ > kQuietWarnFrames &&
          htdd_quiet_streak_ % 2000 == 0))) {
      htdd_quiet_warned_ = true;
      MLPD_WARN(
          "BS: UE PILOT LOST for %zu consecutive frames (last good frame "
          "%lld) -- UE schedule paused or link down\n",
          htdd_quiet_streak_, htdd_frame_counter_);
    }
    // Deliver NOTHING: loopRecv's rx_ret==0 path releases the reserved
    // buffers and continues cleanly (receiver.cc, the no-slot branch), so a
    // quiet frame no longer has to ship a noise buffer tagged as the pilot
    // slot -- which both painted garbage (pre-gate-fix) and emitted
    // duplicate (frame,slot) packets for the whole length of a UE pause
    // (Opus review M7). htdd_frame_counter_ intentionally does not advance:
    // the first REAL frame still lands at recorder frame 0.
    static std::atomic<unsigned> quiet_single_count{0};
    const unsigned qc = quiet_single_count.fetch_add(1) + 1;
    if ((qc & (qc - 1)) == 0) {  // 1,2,4,8,... then quiet
      MLPD_WARN(
          "BS: no UE burst in frame read (rms %.0f vs floor %.0f, occurrence "
          "%u) -- frame skipped\n",
          pilot_rms, floor_rms, qc);
    }
    return 0;
  }
  if (htdd_quiet_warned_) {
    MLPD_WARN("BS: UE pilot RETURNED after %zu quiet frames (frame %lld)\n",
              htdd_quiet_streak_, htdd_frame_counter_);
    htdd_quiet_warned_ = false;
  }
  htdd_quiet_streak_ = 0;
  // The densest slot `at` is a UE slot -- pilot OR data. Identify it: the pilot is
  // identical repeated LTS symbols (high self-similarity at lag cp+fft); data is
  // distinct symbols (low). This keeps P/U tagged correctly so CSI comes from the
  // pilot and equalization from the data.
  auto selfsim = [&](int off) -> double {
    const int lag = static_cast<int>(cfg_->cp_size() + cfg_->fft_size());
    if (off < 0 || off + n > cg) return 0.0;
    double sr = 0, si = 0, sp = 0;
    for (int m = 0; m + lag < n; ++m) {
      const double a = s[2 * (off + m)], b = s[2 * (off + m) + 1];
      const double c = s[2 * (off + m + lag)], d = s[2 * (off + m + lag) + 1];
      sr += a * c + b * d;
      si += b * c - a * d;
      sp += a * a + b * b;
    }
    return sp > 0 ? std::sqrt(sr * sr + si * si) / sp : 0.0;
  };
  const int gap =
      static_cast<int>(htdd_rx_slots_.back() - htdd_pilot_slot_) * n;
  int p_at = at;
  if (selfsim(at) < 0.5) {  // `at` is a data slot -> the pilot is `gap` earlier
    if (selfsim(at - gap) >= 0.4) p_at = at - gap;
    else if (selfsim(at + gap) >= 0.4) p_at = at + gap;
  }
  // The chosen candidate must actually look like repeated LTS symbols. When
  // the pilot is damaged and every candidate fails, the old code kept its
  // best guess and delivered the DATA slot (or worse) as the pilot -- a
  // poison H rendered as a whole-panel garbage blip [user 2026-08-30]. Keep
  // the caller's P/U lockstep but mark the frame fully padded so view mode
  // refuses it, and count occurrences for the mechanism hunt.
  const double pilot_ss = selfsim(p_at);
  if (pilot_ss < 0.4) {
    htdd_frame_pad_ += static_cast<size_t>(n);
    // Recording mode ignores rx_pad (only the view refuses on it), so also
    // push the extent into the gap sink: the HDF5's /Data/Gaps then records
    // that this frame's slots are untrusted (Opus review M8).
    Sounder::RxGapSink::instance().push(
        {r->rxSamplePos() - cg + p_at, static_cast<int64_t>(n),
         Sounder::kGapUntrustedPilot});
    static std::atomic<unsigned> bad_pilot_count{0};
    const unsigned bc = bad_pilot_count.fetch_add(1) + 1;
    if ((bc & (bc - 1)) == 0) {
      MLPD_WARN(
          "BS: pilot slot failed the LTS check (selfsim %.2f, occurrence %u) "
          "-- frame marked untrusted\n",
          pilot_ss, bc);
    }
  }
  // Centroid-align a slot's energy near `guess` -> transmitted [prefix][energy]
  // [postfix] layout (energy edge at ~prefix). Window ~1.25 slots so it can't
  // reach into an adjacent slot and drag the centroid.
  auto align_slot = [&](long long guess) -> long long {
    long long w0 = guess - n / 8;
    if (w0 < 0) w0 = 0;
    long long w1 = w0 + 5 * n / 4;
    if (w1 > cg) w1 = cg;
    double peak = 0.0;
    for (long long i = w0 + 64; i + 64 <= w1; ++i) {
      const double m = cse[i + 64] - cse[i - 64];
      if (m > peak) peak = m;
    }
    const double thr = 0.15 * peak;
    long long cnt = 0;
    double isum = 0.0;
    for (long long i = w0 + 64; i + 64 <= w1; ++i)
      if (cse[i + 64] - cse[i - 64] > thr) { ++cnt; isum += static_cast<double>(i); }
    long long st = (cnt > 0 ? std::llround(isum / cnt) : (guess + n / 2)) - n / 2;
    if (st < 0) st = 0;
    if (st + n > cg) st = cg - n;
    return st;
  };
  const long long p_start = align_slot(p_at);  // aligned pilot slot start
  // Fill the per-frame cache. Centroid-align EACH rx slot to its OWN energy near
  // its expected offset from the pilot -- the UE snaps each slot's tx time to the
  // 384-tick TDD grid independently, so the pilot->data spacing isn't exactly an
  // integer number of slots; extracting the data at pilot+gap would leave it
  // ~260 samples off. Aligning each slot lands every slot at [prefix..] so the
  // recorded data lines up with the pilot for offline equalization.
  htdd_slot_cache_.resize(K * static_cast<size_t>(n) * 2);
  long long u_start = -1;  // aligned start of the uplink-data slot, if present
  for (size_t k = 0; k < K; ++k) {
    const long long guess = p_start +
        (static_cast<long long>(htdd_rx_slots_.at(k)) -
         static_cast<long long>(htdd_pilot_slot_)) * n;
    const long long st = align_slot(guess);
    if (htdd_rx_slots_.at(k) != htdd_pilot_slot_) u_start = st;
    std::memcpy(htdd_slot_cache_.data() + k * static_cast<size_t>(n) * 2,
                s + st * 2, static_cast<size_t>(n) * 4);
  }
  if (getenv("HOUDINI_BS_RX_DEBUG") != nullptr) {
    // Throttle is its OWN knob. HOUDINI_BS_RX_DEBUG=1 has meant "on" in the
    // runbook, the walkthrough, ap15_correlate.py and run_pad_campaign.sh
    // since it was added, and redefining it as a period would turn every one
    // of those into a 1 kHz flood and invalidate ledger 4.60's measured line
    // rate. So: DEBUG stays on/off, EVERY sets the period.
    //
    // AP-51 needs every frame for a while: `pilot_grid_off` is one of the two
    // observables in the two-way transfer, and a slope fit over 1-in-20 at a
    // 260 ms cadence has too few points to separate the clock term from the
    // range-rate term.
    static const int bs_rx_every = [] {
      const char* e = getenv("HOUDINI_BS_RX_EVERY");
      const int v = (e != nullptr) ? atoi(e) : 0;
      return (v > 0) ? v : 20;
    }();
    static std::atomic<int> dc{0};
    if ((dc.fetch_add(1) % bs_rx_every) == 0) {
      // Rederive the UE's realized schedule on the BS grid: the read stamp
      // (ns) + in-buffer position - epoch, folded into the frame, gives the
      // pilot slot's absolute offset from its scheduled slot boundary. The
      // number decomposes as prefix + round-trip latency - tx_advance +
      // grid/snap residuals; it must be CONSTANT within a run, and it is the
      // direct input for deriving tx_advance (DEMO_VERIFICATION.md 4.29).
      const long long stamp_ticks =
          llround(static_cast<double>(ft) * htdd_tick_rate_ / 1e9);
      const long long fr = htdd_frame_ticks_;
      long long grid_off =
          ((stamp_ticks + p_start - htdd_epoch_) % fr + fr) % fr;
      long long rel_pilot = grid_off -
          static_cast<long long>(htdd_pilot_slot_) * n;
      if (rel_pilot > fr / 2) rel_pilot -= fr;
      if (rel_pilot < -fr / 2) rel_pilot += fr;
      long long pu_err = -99999;
      if (u_start >= 0) {
        const long long slots_gap =
            static_cast<long long>(htdd_rx_slots_.back()) -
            static_cast<long long>(htdd_pilot_slot_);
        pu_err = (u_start - p_start) - slots_gap * n;
      }
      // stamp_ticks is carried explicitly: `pilot_grid_off` is an offset and
      // AP-51 needs its SLOPE against time, which the frame counter cannot
      // give (it counts PROCESSED frames, and the BS drops none only when it
      // keeps up). Both numbers on one line so an offline fit needs no join.
      MLPD_INFO("HOUDINI_BS_RX: frame=%lld stamp_ticks=%lld cg=%d "
                "pilot-rms=%.0f selfsim=%.2f p_start=%lld rx_slots=%zu "
                "pilot_grid_off=%lld pu_spacing_err=%lld\n",
                htdd_frame_counter_, stamp_ticks, cg, pilot_rms, selfsim(at),
                p_start, K, rel_pilot, pu_err);
    }
  }
  // Landing-map instrument (phase 5-7 walk): dump the raw continuous read plus
  // the grid metadata so the offline analyzer can place every burst on the
  // ABSOLUTE slot grid. Ground truth for zero prefix/postfix sizing -- the CSI
  // dump stores centroid-ALIGNED slots and cannot serve here.
  const char* lm_dir = getenv("HOUDINI_BS_DUMP_FRAME");
  if (lm_dir != nullptr) {
    static std::atomic<int> lm_dumped{0};
    static std::atomic<long long> lm_next{200};
    if (htdd_frame_counter_ >= lm_next.load() && lm_dumped.load() < 8) {
      lm_next.store(htdd_frame_counter_ + 500);
      const int dk = lm_dumped.fetch_add(1);
      char pb[512];
      snprintf(pb, sizeof(pb), "%s/bsframe_%02d.bin", lm_dir, dk);
      FILE* fb = fopen(pb, "wb");
      if (fb != nullptr) {
        fwrite(s, sizeof(int16_t), static_cast<size_t>(cg) * 2, fb);
        fclose(fb);
      }
      snprintf(pb, sizeof(pb), "%s/bsframe_%02d.txt", lm_dir, dk);
      FILE* fg = fopen(pb, "w");
      if (fg != nullptr) {
        fprintf(fg,
                "ft_ns %lld\ncg %d\nepoch %lld\ntick_rate %.1f\n"
                "frame_ticks %lld\npilot_slot %lld\nn %d\np_start %lld\n"
                "u_start %lld\npad %lld\nframe %lld\nrx_slots",
                ft, cg, static_cast<long long>(htdd_epoch_), htdd_tick_rate_,
                static_cast<long long>(htdd_frame_ticks_),
                static_cast<long long>(htdd_pilot_slot_), n, p_start, u_start,
                static_cast<long long>(htdd_frame_pad_),
                static_cast<long long>(htdd_frame_counter_));
        for (size_t rk = 0; rk < K; ++rk)
          fprintf(fg, " %lld", static_cast<long long>(htdd_rx_slots_.at(rk)));
        fprintf(fg, "\n");
        fclose(fg);
      }
    }
  }
  std::memcpy(buffs[0], htdd_slot_cache_.data(), static_cast<size_t>(n) * 4);
  htdd_cache_frame_ = htdd_frame_counter_;
  ++htdd_frame_counter_;
  htdd_rx_cursor_ = (K > 1) ? 1 : 0;
  frameTime = (htdd_cache_frame_ << 32) |
              (static_cast<long long>(htdd_rx_slots_.at(0)) << 16);
  return n;
}


void HoudiniFramer::arm() {
  // bs_hw_framer=true -> native TDD framer (beacon replay strobe + rx_gate on
  // the pilot slots; loopRecv's HW-framer true-path receives only those
  // slots, tagged). bs_hw_framer=false -> free-running replay beacon + the
  // software-framer read-every-slot path.
  if (cfg_->bs_hw_framer()) {
    armTdd();
    MLPD_INFO("BaseRadioSet done (Houdini native TDD framer)!\n");
  } else {
    armReplayBeacon();
    MLPD_INFO("BaseRadioSet done (Houdini replay beacon)!\n");
  }
}

void HoudiniFramer::stop() {
  // Native TDD teardown so the next run can re-arm. Full ladder, not abort
  // alone: abort latches gates_held on a running framer and skips TX_CLEAR
  // (3.2 + 4.24 in DEMO_VERIFICATION.md).
  for (size_t c = 0; c < radios_.size(); c++)
    for (size_t i = 0; i < radios_.at(c).size(); i++)
      if (radios_.at(c).at(i) != nullptr) {
        try {
          tddLadder(radios_.at(c).at(i)->RawDev(), cfg_->diag_skip_tx_clear());
        } catch (...) {
        }
      }
}

int HoudiniFramer::txBeacon(size_t radio_id, size_t cell_id, const void* const* buffs,
                            int flags, long long& frameTime) {
  // The beacon is a free-running device replay armed at arm(), so loopRecv's
  // per-frame software beacon TX (baseTxBeacon) is a no-op here -- writing to
  // the replay-mode TX stream would corrupt the loaded beacon. Report the full
  // slot as "sent" so baseTxBeacon doesn't log BAD Transmit.
  (void)radio_id; (void)cell_id; (void)buffs; (void)flags; (void)frameTime;
  return static_cast<int>(cfg_->samps_per_slot());
}
