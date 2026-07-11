// Verifies finding-1's fix: an H5::Exception from a bad output path must be
// catchable exactly as rx_recorder_main.cc's handlers catch it -- not
// std::terminate. Mirrors main(): H5::Exception first, std::exception after.
#include <H5Cpp.h>
#include <cstdio>
#include <fstream>
#include <memory>
#include "include/recorder_thread.h"
#include "include/rx_recorder_config.h"
#include "include/rx_recorder_worker.h"

int main() {
  std::ofstream("/tmp/rx_bad_conf.json")
      << "{ \"channels\": [0], \"output_file\": \"/no/such/dir/cap.h5\" }";
  Sounder::RxRecorderConfig cfg("/tmp/rx_bad_conf.json", "/tmp");
  Sounder::RxCaptureMeta meta;
  meta.actual_rate = 1e6;
  meta.total_slots = 10;
  try {
    Sounder::RecorderThread recorder(
        std::make_unique<Sounder::RxRecorderWorker>(&cfg, meta),
        cfg.getPacketDataLength(), 0, -1, 16, true);
    std::fprintf(stderr, "FAIL: constructor did not throw\n");
    return 1;
  } catch (H5::Exception& e) {
    std::printf("PASS: caught H5::Exception cleanly: %s\n", e.getCDetailMsg());
    return 0;
  } catch (const std::exception& e) {
    std::printf("PASS (std::exception path): %s\n", e.what());
    return 0;
  }
}
