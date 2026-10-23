#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include "cpg_controller/learning_interface.hpp"

namespace cpg_controller {
constexpr double kTwoPi = 6.2831853071795864769;
inline double wrap_phase(double v) { return std::remainder(v, kTwoPi); }
enum class LearningState { WAITING = 0, CAPTURE = 1, LOCKED = 2, LOST = 3 };

struct AdaptiveConfig {
  double omega_initial{5.720866}, omega_min{2.5}, omega_max{9.0};
  double phase_gain{6.0}, frequency_gain{3.0}, shape_gain{4.0};
  double measurement_timeout{0.12};
  bool shape_enabled{true}, events_enabled{true};
  // Experimental ablation: false learns shape during valid capture as well.
  // Both policies always adapt phase and frequency together.
  bool shape_requires_lock{true};
  double phase_error_limit{0.45}, phase_acceleration{16.0};
  double frequency_rate_limit{1.5}, rate_time_constant{0.05};
  double shape_time_constant{0.2}, prior_pull{0.005};
  double lock_error{0.16}, lock_duration{1.5};
  double event_gain{2.0}, event_frequency_gain{2.0}, event_timeout{0.3};
};

struct AdaptiveOutput {
  std::array<double, 6> position{}, velocity{};
  double phase{0}, omega_base{0}, phase_rate{0}, phase_error{0};
  double residual_rms{0}, measurement_age{std::numeric_limits<double>::infinity()};
  LearningState state{LearningState::WAITING};
  bool shape_active{false};
};

// Shared, unit-radius Hopf clock with normalized Fourier readouts. No second
// observer oscillator: every prediction, gradient and reference uses this clock.
class AdaptiveCPG {
public:
  using Weights = std::array<Coefficients, 6>;
  explicit AdaptiveCPG(const Weights &prior, const AdaptiveConfig &config, double start_time)
  : cfg_(config), prior_(prior), weights_(prior), now_(start_time),
    omega_(config.omega_initial), rate_(config.omega_initial) {
    if (!std::isfinite(start_time) || !valid_config())
      throw std::invalid_argument("Invalid adaptive CPG configuration");
    for (size_t j = 0; j < 6; ++j) {
      double energy = 0;
      if (!std::isfinite(prior_[j].offset.value)) throw std::invalid_argument("Nonfinite prior");
      prior_[j].offset.rate = weights_[j].offset.rate = 0;
      for (size_t n = 1; n <= 3; ++n) {
        if (!std::isfinite(prior_[j].x[n].value) || !std::isfinite(prior_[j].y[n].value))
          throw std::invalid_argument("Nonfinite prior");
        energy += prior_[j].x[n].value * prior_[j].x[n].value +
          prior_[j].y[n].value * prior_[j].y[n].value;
        prior_[j].x[n].rate = weights_[j].x[n].rate = 0;
        prior_[j].y[n].rate = weights_[j].y[n].rate = 0;
      }
      // Synthetic defaults: the floor prevents small ankle signals dominating.
      // A hardware deployment must replace these scales with calibrated noise/ROM.
      scales_[j] = std::max(0.06, std::sqrt(energy / 2));
    }
    record();
  }

  void step(double now) {
    if (!std::isfinite(now) || now <= now_) return;
    const double dt = now - now_;
    if (dt > 0.05) {  // discontinuous clock: do not integrate an unbounded catch-up
      now_ = now;
      lose();
      history_count_ = history_head_ = 0;
      record();
      return;
    }
    now_ = now;
    const bool fresh = last_measurement_ > 0 && now_ - last_measurement_ <= cfg_.measurement_timeout;
    if (!fresh && state_ != LearningState::WAITING) lose();
    const double z = fresh && observable_ ? phase_error_ : 0;
    double omega_dot = fresh && moving_ ? cfg_.frequency_gain * z : 0;
    if (fresh && cfg_.events_enabled && event_period_stamp_ > 0 &&
        now_ - event_period_stamp_ < 1.8 * kTwoPi / omega_)
      omega_dot += cfg_.event_frequency_gain * (event_omega_ - omega_);
    omega_dot = std::clamp(omega_dot, -cfg_.frequency_rate_limit, cfg_.frequency_rate_limit);
    omega_ = std::clamp(omega_ + dt * omega_dot, cfg_.omega_min, cfg_.omega_max);

    // Event phase innovation is a bounded remaining budget, never a phase reset.
    const double event_target = fresh && cfg_.events_enabled ?
      std::clamp(cfg_.event_gain * pending_event_, -0.8, 0.8) : 0;
    const double continuous = cfg_.phase_gain * z;
    const double target = std::clamp(omega_ + continuous + event_target,
      cfg_.omega_min, cfg_.omega_max);
    const double old_rate = rate_;
    const double change = (target - rate_) * (1 - std::exp(-dt / cfg_.rate_time_constant));
    rate_ = std::clamp(rate_ + std::clamp(change, -cfg_.phase_acceleration * dt,
      cfg_.phase_acceleration * dt), cfg_.omega_min, cfg_.omega_max);
    // Attribute only the actually applied part of the speed budget to the event.
    const double base_continuous = std::clamp(omega_ + continuous, cfg_.omega_min, cfg_.omega_max);
    const double event_applied = std::clamp(rate_ - base_continuous,
      std::min(0.0, event_target), std::max(0.0, event_target));
    if (event_applied * pending_event_ > 0)
      pending_event_ -= std::copysign(std::min(std::abs(pending_event_),
        dt * std::abs(event_applied)), pending_event_);

    // This is the exact tangential Hopf flow on r=1 (mu=1):
    // dx=-v*y, dy=v*x, psi_dot=v; radial dynamics are at their equilibrium.
    phase_ += dt * (old_rate + rate_) / 2;
    x_ = std::cos(phase_); y_ = std::sin(phase_);
    const bool shape_ready = !cfg_.shape_requires_lock ||
      (state_ == LearningState::LOCKED && std::abs(phase_error_) < cfg_.lock_error);
    const bool learn_shape = fresh && cfg_.shape_enabled && moving_ && observable_ && shape_ready;
    shape_active_ = learn_shape;
    const double blend_rate = 1 - std::exp(-dt / cfg_.shape_time_constant);
    for (size_t j = 0; j < 6; ++j) {
      apply_weight(weights_[j].offset, prior_[j].offset, shape_target_[j].offset,
        learn_shape && shape_valid_[j], blend_rate, dt, j);
      for (size_t n = 1; n <= 3; ++n) {
        apply_weight(weights_[j].x[n], prior_[j].x[n], shape_target_[j].x[n],
          learn_shape && shape_valid_[j], blend_rate, dt, j);
        apply_weight(weights_[j].y[n], prior_[j].y[n], shape_target_[j].y[n],
          learn_shape && shape_valid_[j], blend_rate, dt, j);
      }
    }
    record();
  }

  // Called once per unique source sample, after step(arrival_time). Historical W
  // and phase ensure sensor delay is not mistaken for phase error.
  bool observe(const MeasurementFrame &measurement, double arrival_time) {
    const double stamp = measurement.stamp;
    if (!std::isfinite(arrival_time) || arrival_time > now_ + 1e-6 ||
        !std::isfinite(stamp) || stamp <= 0 || stamp <= last_measurement_ ||
        stamp > arrival_time + 1e-9 || arrival_time - stamp > cfg_.measurement_timeout) return false;
    History predicted;
    if (!at(stamp, predicted)) return false;
    std::array<bool, 6> valid{};
    size_t count = 0;
    for (size_t j = 0; j < 6; ++j) {
      valid[j] = measurement.valid[j] && std::isfinite(measurement.position[j]) &&
        std::abs(measurement.position[j] - prior_[j].offset.value) < 8 * scales_[j];
      if (valid[j]) ++count;
    }
    if (count < 2) return false;
    const double sample_dt = last_measurement_ > 0 ? stamp - last_measurement_ : 0;
    const bool acquiring = state_ == LearningState::WAITING || state_ == LearningState::LOST;
    const double shift = acquiring ? wrap_phase(coarse_phase(measurement, valid, predicted.weights) -
      predicted.phase) : 0;
    predicted.phase += shift;
    const auto basis = harmonics(std::cos(predicted.phase), std::sin(predicted.phase), 0, 0);
    // Validate the residual mask before committing any capture, history or motion
    // state. Rejected frames cannot rotate the clock or create motion evidence.
    count = 0;
    for (size_t j = 0; j < 6; ++j) {
      if (valid[j] && std::abs(measurement.position[j] -
          readout(predicted.weights[j], basis).value) > 3 * scales_[j]) valid[j] = false;
      if (valid[j]) ++count;
    }
    if (count < 2) return false;
    if (acquiring) {
      // Global acquisition is allowed only while reference validity is false.
      // Discard the old history rather than rewriting previously emitted phase.
      phase_ += shift; x_ = std::cos(phase_); y_ = std::sin(phase_);
      history_count_ = history_head_ = 0;
      if (stamp < now_) {
        // Seed only the invalid acquisition interval, from source time to now.
        // Without this, a valid long sensor delay can cause a LOST/capture loop.
        history_[0] = {stamp, predicted.phase, predicted.weights};
        history_head_ = history_count_ = 1;
      }
      record();
      state_ = LearningState::CAPTURE;
      stable_time_ = unstable_time_ = 0;
      phase_error_ = 0;
      last_event_ = {};
      event_period_stamp_ = 0;
      last_anchor_stamp_ = last_anchor_error_ = 0;
      pending_event_ = 0;
    }
    double numerator = 0, denominator = 0, squared = 0, norm_squared = 0;
    size_t used = 0;
    shape_valid_ = {};
    if (sample_dt > 0 && sample_dt <= cfg_.measurement_timeout) {
      double motion = 0; size_t motion_count = 0;
      for (size_t j = 0; j < 6; ++j) if (valid[j] && previous_valid_[j]) {
        const double speed = (measurement.position[j] - previous_position_[j]) / (sample_dt * scales_[j]);
        motion += std::min(speed * speed, 100.0); ++motion_count;
      }
      if (motion_count) motion_power_ += (1 - std::exp(-sample_dt / 0.2)) *
        (motion / motion_count - motion_power_);
    }
    moving_ = motion_power_ > 0.15;
    for (size_t j = 0; j < 6; ++j) {
      previous_valid_[j] = valid[j];
      if (!valid[j]) continue;
      previous_position_[j] = measurement.position[j];
      const double residual = measurement.position[j] - readout(predicted.weights[j], basis).value;
      const double robust = std::clamp(residual, -1.5 * scales_[j], 1.5 * scales_[j]);
      double derivative = 0;
      for (size_t n = 1; n <= 3; ++n)
        derivative += n * (-predicted.weights[j].x[n].value * basis.y[n].value +
          predicted.weights[j].y[n].value * basis.x[n].value);
      const double rho = 1 / (scales_[j] * scales_[j]);
      numerator += rho * derivative * robust;
      denominator += rho * derivative * derivative;
      squared += residual * residual;
      norm_squared += rho * robust * robust;
      ++used;
      shape_valid_[j] = true;
      // Negative-gradient sign for e=measurement-model. Conservative higher
      // harmonics and DC; rates, not parameter jumps, are applied by step().
      shape_target_[j].offset = shape_direction(robust, 1, weights_[j].offset,
        prior_[j].offset, 0.25);
      for (size_t n = 1; n <= 3; ++n) {
        shape_target_[j].x[n] = shape_direction(robust, basis.x[n].value,
          weights_[j].x[n], prior_[j].x[n], 1.0 / n);
        shape_target_[j].y[n] = shape_direction(robust, basis.y[n].value,
          weights_[j].y[n], prior_[j].y[n], 1.0 / n);
      }
    }
    if (used < 2) return false;
    residual_rms_ = std::sqrt(squared / used);
    observable_ = denominator > 0.08 && moving_;
    const double raw = observable_ ? std::clamp(numerator / (0.03 + denominator),
      -cfg_.phase_error_limit, cfg_.phase_error_limit) : 0;
    const double filter_dt = sample_dt > 0 && sample_dt <= cfg_.measurement_timeout ? sample_dt : 0.01;
    phase_error_ += (1 - std::exp(-filter_dt / 0.04)) * (raw - phase_error_);
    const bool event_ok = last_anchor_stamp_ <= 0 || arrival_time - last_anchor_stamp_ >
      2 * kTwoPi / omega_ || std::abs(last_anchor_error_) < 0.3;
    // Capture and retention have hysteresis. Requiring an entire 1.5 s of
    // pointwise tiny residual would starve shape adaptation whenever a harmless
    // within-stride template mismatch repeats once per ~1.1 s cycle.
    const bool stable = observable_ && std::abs(phase_error_) < 2 * cfg_.lock_error &&
      norm_squared / used < 1.5 && event_ok;
    stable_time_ = stable ? stable_time_ + std::min(filter_dt, 0.03) :
      std::max(0.0, stable_time_ - 2 * filter_dt);
    if (state_ != LearningState::LOCKED && stable_time_ >= cfg_.lock_duration)
      state_ = LearningState::LOCKED;
    const bool bad = !moving_ || std::abs(phase_error_) > 3 * cfg_.lock_error ||
      norm_squared / used > 2 || !event_ok;
    unstable_time_ = bad ? unstable_time_ + filter_dt : std::max(0.0, unstable_time_ - filter_dt);
    if (state_ == LearningState::LOCKED && unstable_time_ > 0.3) {
      state_ = LearningState::CAPTURE; stable_time_ = 0;
    }
    last_measurement_ = stamp;
    // Future gradients use this sample; history continues to describe the actual
    // coefficients used at each past time, including their applied rates.
    return true;
  }

  bool event(double stamp, size_t foot, uint64_t id, double confidence,
      double target_phase, double arrival_time) {
    if (!cfg_.events_enabled || state_ == LearningState::WAITING || state_ == LearningState::LOST ||
        foot > 1 || id == 0 || !std::isfinite(stamp) || !std::isfinite(arrival_time) ||
        !std::isfinite(target_phase) || !std::isfinite(confidence) || confidence < 0.7 || confidence > 1 ||
        stamp <= 0 || stamp > arrival_time || arrival_time > now_ + 1e-6 ||
        arrival_time - stamp > cfg_.event_timeout || now_ - last_measurement_ > cfg_.measurement_timeout)
      return false;
    auto &old = last_event_[foot];
    if (id <= old.id || stamp <= old.stamp) return false;
    History predicted;
    if (!at(stamp, predicted)) return false;
    const double error = wrap_phase(target_phase - predicted.phase);
    if (std::abs(error) > 1.2) { state_ = LearningState::CAPTURE; stable_time_ = 0; return false; }
    if (old.stamp > 0) {
      const double period = stamp - old.stamp;
      const double expected = kTwoPi / omega_;
      // Alternating feet are never used as a full period; reject doubled periods.
      if (period >= kTwoPi / cfg_.omega_max && period <= kTwoPi / cfg_.omega_min &&
          period > 0.6 * expected && period < 1.6 * expected) {
        event_omega_ = kTwoPi / period;
        event_period_stamp_ = stamp;
      }
    }
    old = {stamp, id};
    last_anchor_error_ = error;
    last_anchor_stamp_ = stamp;
    pending_event_ = std::clamp(error, -0.6, 0.6);  // replace stale innovation, do not accumulate
    if (std::abs(error) > 0.3) { state_ = LearningState::CAPTURE; stable_time_ = 0; }
    return true;
  }

  AdaptiveOutput output() const {
    AdaptiveOutput result;
    result.phase = phase_; result.omega_base = omega_; result.phase_rate = rate_;
    result.phase_error = phase_error_; result.residual_rms = residual_rms_;
    result.state = state_; result.shape_active = shape_active_;
    if (last_measurement_ > 0) result.measurement_age = now_ - last_measurement_;
    const auto basis = harmonics(x_, y_, -rate_ * y_, rate_ * x_);
    for (size_t j = 0; j < 6; ++j) {
      const auto q = readout(weights_[j], basis);
      result.position[j] = q.value; result.velocity[j] = q.rate;
    }
    return result;
  }
  const Weights &weights() const { return weights_; }

private:
  struct Rates { double offset{0}; std::array<double, 4> x{}, y{}; };
  struct History { double time{0}, phase{0}; Weights weights{}; };
  struct Anchor { double stamp{0}; uint64_t id{0}; };
  static constexpr size_t kHistory = 512;  // 1.024 s at 500 Hz, fixed storage
  bool valid_config() const {
    const std::array<double, 18> positive{cfg_.omega_initial, cfg_.omega_min, cfg_.omega_max,
      cfg_.measurement_timeout, cfg_.phase_error_limit, cfg_.phase_acceleration,
      cfg_.frequency_rate_limit, cfg_.rate_time_constant, cfg_.shape_time_constant,
      cfg_.lock_error, cfg_.lock_duration, cfg_.event_timeout, cfg_.event_gain,
      cfg_.event_frequency_gain, cfg_.phase_gain + 1, cfg_.frequency_gain + 1,
      cfg_.shape_gain + 1, cfg_.prior_pull + 1};
    for (double value : positive) if (!std::isfinite(value) || value <= 0) return false;
    return cfg_.omega_min < cfg_.omega_max && cfg_.omega_initial >= cfg_.omega_min &&
      cfg_.omega_initial <= cfg_.omega_max && cfg_.phase_gain >= 0 &&
      cfg_.frequency_gain >= 0 && cfg_.shape_gain >= 0 && cfg_.prior_pull >= 0;
  }
  void lose() {
    if (state_ != LearningState::WAITING) state_ = LearningState::LOST;
    phase_error_ = 0; observable_ = false; moving_ = false;
    shape_active_ = false; stable_time_ = unstable_time_ = 0; pending_event_ = 0;
    motion_power_ = 0;
    shape_valid_ = {}; previous_valid_ = {};
  }
  double shape_direction(double residual, double basis, Sample w, Sample prior, double factor) const {
    return cfg_.shape_gain * factor * (residual * basis -
      cfg_.prior_pull * (w.value - prior.value)) / 4.0;
  }
  void apply_weight(Sample &weight, Sample prior, double target, bool enabled,
      double alpha, double dt, size_t joint) {
    const double bound = 0.8 * scales_[joint] + 0.02;
    target = enabled ? std::clamp(target, -0.25 * scales_[joint], 0.25 * scales_[joint]) : 0;
    const double old_rate = weight.rate;
    weight.rate += alpha * (target - weight.rate);
    const double next = weight.value + dt * (old_rate + weight.rate) / 2;
    weight.value = std::clamp(next, prior.value - bound, prior.value + bound);
    if (next != weight.value) weight.rate = 0;  // derivative of applied projection
  }
  void record() {
    history_[history_head_] = {now_, phase_, weights_};
    history_head_ = (history_head_ + 1) % kHistory;
    history_count_ = std::min(history_count_ + 1, kHistory);
  }
  bool at(double stamp, History &result) const {
    if (!history_count_) return false;
    const size_t oldest = (history_head_ + kHistory - history_count_) % kHistory;
    const auto &first = history_[oldest];
    const auto &last = history_[(history_head_ + kHistory - 1) % kHistory];
    if (stamp < first.time - 1e-8 || stamp > last.time + 1e-8) return false;
    if (stamp <= first.time) { result = first; return true; }
    for (size_t i = 1; i < history_count_; ++i) {
      const auto &b = history_[(oldest + i) % kHistory];
      if (b.time + 1e-9 < stamp) continue;
      const auto &a = history_[(oldest + i - 1) % kHistory];
      const double alpha = std::clamp((stamp - a.time) / (b.time - a.time), 0.0, 1.0);
      result.time = stamp; result.phase = a.phase + alpha * (b.phase - a.phase);
      for (size_t j = 0; j < 6; ++j) {
        result.weights[j].offset = blend(a.weights[j].offset, b.weights[j].offset, {alpha, 0});
        for (size_t n = 1; n <= 3; ++n) {
          result.weights[j].x[n] = blend(a.weights[j].x[n], b.weights[j].x[n], {alpha, 0});
          result.weights[j].y[n] = blend(a.weights[j].y[n], b.weights[j].y[n], {alpha, 0});
        }
      }
      return true;
    }
    result = last; return true;
  }
  double coarse_phase(const MeasurementFrame &measurement, const std::array<bool, 6> &valid,
      const Weights &weights) const {
    double best = 0, cost = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < 256; ++i) {
      const double candidate = kTwoPi * i / 256;
      const auto basis = harmonics(std::cos(candidate), std::sin(candidate), 0, 0);
      double error = 0;
      for (size_t j = 0; j < 6; ++j) if (valid[j]) {
        const double e = (measurement.position[j] - readout(weights[j], basis).value) / scales_[j];
        error += std::min(e * e, 9.0);
      }
      if (error < cost) { cost = error; best = candidate; }
    }
    // Refine the grid optimum locally; exact template input should not create
    // artificial coefficient drift from the finite acquisition grid spacing.
    for (size_t iteration = 0; iteration < 8; ++iteration) {
      const auto basis = harmonics(std::cos(best), std::sin(best), 0, 0);
      double numerator = 0, denominator = 1e-12;
      for (size_t j = 0; j < 6; ++j) if (valid[j]) {
        const double residual = measurement.position[j] - readout(weights[j], basis).value;
        if (std::abs(residual) > 3 * scales_[j]) continue;
        double derivative = 0;
        for (size_t n = 1; n <= 3; ++n)
          derivative += n * (-weights[j].x[n].value * basis.y[n].value +
            weights[j].y[n].value * basis.x[n].value);
        const double rho = 1 / (scales_[j] * scales_[j]);
        numerator += rho * derivative * residual;
        denominator += rho * derivative * derivative;
      }
      best += std::clamp(numerator / denominator, -0.05, 0.05);
    }
    return best;
  }
  AdaptiveConfig cfg_;
  Weights prior_, weights_;
  std::array<Rates, 6> shape_target_{};
  std::array<bool, 6> shape_valid_{}, previous_valid_{};
  std::array<double, 6> scales_{}, previous_position_{};
  double now_{0}, phase_{0}, x_{1}, y_{0}, omega_{0}, rate_{0};
  double phase_error_{0}, residual_rms_{0}, last_measurement_{0}, stable_time_{0}, unstable_time_{0}, motion_power_{0};
  double pending_event_{0}, event_omega_{0}, event_period_stamp_{0};
  double last_anchor_stamp_{0}, last_anchor_error_{0};
  LearningState state_{LearningState::WAITING};
  bool observable_{false}, moving_{false}, shape_active_{false};
  std::array<Anchor, 2> last_event_{};
  std::array<History, kHistory> history_{};
  size_t history_head_{0}, history_count_{0};
};
}  // namespace cpg_controller
