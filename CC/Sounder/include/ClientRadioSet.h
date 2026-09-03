/**
 * @file ClientRadioSet.h
 * @brief Declaration file for the ClientRadioSet class.
 */
#ifndef CLIENT_RADIO_SET_H_
#define CLIENT_RADIO_SET_H_

#include <atomic>
#include <cstddef>
#include <memory>

#include "Radio.h"
#include "RadioSetInterfaces.h"
#include "config.h"

class ClientRadioSet : public IClientRadioSet {
 public:
  ClientRadioSet(Config* cfg);
  ~ClientRadioSet(void) override;
  int triggers(int i) override;
  int radioRx(size_t radio_id, void* const* buffs, int numSamps,
              long long& frameTime) override;
  int radioTx(size_t radio_id, const void* const* buffs, int numSamps,
              int flags, long long& frameTime) override;
  // Drain asynchronous TX status for one client radio; see Radio::drainTxStatus.
  int drainTxStatus(size_t radio_id) override;
  void radioStop(void) override;
  bool getRadioNotFound() override { return radioNotFound; }

 private:
  struct ClientRadioContext {
    ClientRadioSet* crs;
    std::atomic_ulong* thread_count;
    size_t tid;
  };
  void init(ClientRadioContext* context);
  static void* init_launch(void* in_context);

  Config* _cfg;
  std::vector<std::unique_ptr<Radio>> radios;
  bool radioNotFound;
};

#endif /* CLIENT_RADIO_SET_H_ */
