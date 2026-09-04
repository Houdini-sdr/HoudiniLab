/** @file RadioSetInterfaces.h
  * @brief What the receiver asks of a base-station set and of a client set:
  *        the interfaces the Soapy sets (Iris and Houdini) and the native-UHD
  *        sets implement, so the receiver holds one type and a factory picks
  *        the implementation the build and the configuration provide.
  *
  * docs/RADIO_PLATFORM_SEAM.md step S3. Agora's shape (RadioSet / RadioSetBs /
  * RadioSetUhd), reduced to the calls this receiver makes. The native UHD
  * sets are their own implementations because they hold one multi-board
  * device, not one radio per board; that structure is theirs to keep.
  *
  * RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
*/
#ifndef RADIO_SET_INTERFACES_H_
#define RADIO_SET_INTERFACES_H_

#include <cstddef>
#include <memory>

class Config;

class IBaseRadioSet {
 public:
  virtual ~IBaseRadioSet() = default;
  virtual int radioTx(size_t radio_id, size_t cell_id, const void* const* buffs, int flags,
                      long long& frameTime) = 0;
  virtual int radioRx(size_t radio_id, size_t cell_id, void* const* buffs,
                      long long& frameTime) = 0;
  virtual int radioRx(size_t radio_id, size_t cell_id, void* const* buffs, int numSamps,
                      long long& frameTime) = 0;
  /// Samples zero-padded into the window backing the LAST radioRx (0 = clean,
  /// and 0 always for a set without a gap ledger). See AP-10.
  virtual size_t lastRxPadSamples(size_t radio_id, size_t cell_id) const = 0;
  virtual void radioStart() = 0;
  virtual void radioStop() = 0;
  virtual bool getRadioNotFound() = 0;
};

class IClientRadioSet {
 public:
  virtual ~IClientRadioSet() = default;
  virtual int triggers(int i) = 0;
  virtual int radioRx(size_t radio_id, void* const* buffs, int numSamps, long long& frameTime) = 0;
  virtual int radioTx(size_t radio_id, const void* const* buffs, int numSamps, int flags,
                      long long& frameTime) = 0;
  /// Drain asynchronous TX status for one client radio; 0 for a set whose
  /// stream reports none.
  virtual int drainTxStatus(size_t radio_id) = 0;
  virtual void radioStop() = 0;
  virtual bool getRadioNotFound() = 0;
};

/// The sets this build provides for this configuration: the native-UHD sets
/// when built with RADIO_TYPE=PURE_UHD, the Soapy sets (Iris, Soapy UHD,
/// Houdini) otherwise. The one place that knows.
std::unique_ptr<IBaseRadioSet> makeBaseRadioSet(Config* cfg, bool calibrate_proc);
std::unique_ptr<IClientRadioSet> makeClientRadioSet(Config* cfg);

#endif  // RADIO_SET_INTERFACES_H_
