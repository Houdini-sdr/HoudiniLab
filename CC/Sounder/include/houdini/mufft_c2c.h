/**
 * @file houdini/mufft_c2c.h
 * @brief One N-point complex muFFT transform that owns its aligned buffers and
 *        its plan: the CSI view's forward spectrum (dc_fft.h) and inverse CIR
 *        (cir.h). Each member frees itself, so a constructor that fails part
 *        way leaks nothing.
 *
 * RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 */
#pragma once

#include <complex>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>

extern "C" {
#include "fft.h"  // muFFT
}

namespace houdini {

class MufftC2C {
 public:
  /// direction MUFFT_FORWARD or MUFFT_INVERSE; `who` names the owner in errors.
  MufftC2C(int n, int direction, const std::string& who)
      : n_(checked(n, who)),
        in_(alloc(n), &mufft_free),
        out_(alloc(n), &mufft_free),
        plan_(mufft_create_plan_1d_c2c(static_cast<unsigned>(n), direction, MUFFT_FLAG_CPU_ANY), &mufft_free_plan_1d) {
    if (!in_ || !out_ || !plan_) throw std::runtime_error(who + ": muFFT allocation failed");
  }

  int size() const { return n_; }
  std::complex<float>* in() { return in_.get(); }
  const std::complex<float>* out() const { return out_.get(); }
  /// out = the transform of in.
  void execute() { mufft_execute_plan_1d(plan_.get(), out_.get(), in_.get()); }

 private:
  using Buf = std::unique_ptr<std::complex<float>, void (*)(void*)>;
  static int checked(int n, const std::string& who) {
    if (n <= 0 || (n & (n - 1)) != 0) throw std::invalid_argument(who + ": n must be a power of two");
    return n;
  }
  static std::complex<float>* alloc(int n) {
    return static_cast<std::complex<float>*>(mufft_alloc(static_cast<size_t>(n) * sizeof(std::complex<float>)));
  }
  int n_;
  Buf in_, out_;
  std::unique_ptr<mufft_plan_1d, void (*)(mufft_plan_1d*)> plan_;
};

}  // namespace houdini
