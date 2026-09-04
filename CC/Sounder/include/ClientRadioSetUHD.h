/**
 * @file ClientRadioSetUHD.h
 * @brief Declaration file for the ClientRadioSetUHD class.
 */
#ifndef CLIENT_RADIO_SET_UHD_H_
#define CLIENT_RADIO_SET_UHD_H_

#include <atomic>
#include <cstddef>

#include "RadioUHD.h"
#include "RadioSetInterfaces.h"
#include "config.h"

class ClientRadioSetUHD : public IClientRadioSet {
 public:
  ClientRadioSetUHD(Config* cfg);
  ~ClientRadioSetUHD(void) override;
  int triggers(int i) override;
  int radioRx(size_t radio_id, void* const* buffs, int numSamps,
              long long& frameTime) override;
  int radioTx(size_t radio_id, const void* const* buffs, int numSamps,
              int flags, long long& frameTime) override;
  void radioStop(void) override;
  /// Drain asynchronous TX status; UHD reports it through its own stream and
  /// there is nothing to drain here. Kept so the receiver's call compiles
  /// under RADIO_TYPE=PURE_UHD (baseline assessment B1).
  int drainTxStatus(size_t /*radio_id*/) override { return 0; }
  bool getRadioNotFound() override { return radioNotFound; }

 private:
  struct ClientRadioContext {
    ClientRadioSetUHD* crs;
    std::atomic_ulong* thread_count;
    size_t tid;
  };
  void init(ClientRadioContext* context);
  static void* init_launch(void* in_context);

  Config* _cfg;
  RadioUHD* radio_;
  bool radioNotFound;
};

#endif /* CLIENT_RADIO_SET_UHD_H_ */
