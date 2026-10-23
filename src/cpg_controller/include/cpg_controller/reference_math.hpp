#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace cpg_controller {
// Value and its time derivative always travel together, including constraints.
struct Sample { double value{0.0}; double rate{0.0}; };
inline Sample operator+(Sample a, Sample b) { return {a.value+b.value, a.rate+b.rate}; }
inline Sample operator*(Sample a, Sample b) {
  return {a.value*b.value, a.rate*b.value+a.value*b.rate};
}
inline Sample blend(Sample a, Sample b, Sample s) {
  return {a.value+(b.value-a.value)*s.value,
    (1-s.value)*a.rate+s.value*b.rate+s.rate*(b.value-a.value)};
}
inline Sample constrain(Sample q, Sample low, Sample high) {
  if (q.value <= low.value) return low;
  if (q.value >= high.value) return high;
  return q;
}
inline Sample progress(double elapsed, double duration) {
  const double t=std::clamp(elapsed/duration,0.0,1.0);
  return {t*t*t*(10+t*(-15+6*t)),30*t*t*(1-t)*(1-t)/duration};
}

// A retargetable quintic preserves value, velocity and acceleration at the splice.
class Transition {
public:
  Transition() = default;
  explicit Transition(double value) : target_(value) { c_[0]=value; }
  Sample sample(double now) const {
    const double t=std::clamp(now-start_,0.0,duration_);
    if (t >= duration_) return {target_,0.0};
    double q=c_[5], v=5*c_[5];
    for (int i=4;i>=0;--i) q=q*t+c_[i];
    for (int i=4;i>=1;--i) v=v*t+i*c_[i];
    return {q,v};
  }
  double acceleration(double now) const {
    const double t=std::clamp(now-start_,0.0,duration_);
    if (t>=duration_) return 0.0;
    return 2*c_[2]+t*(6*c_[3]+t*(12*c_[4]+t*20*c_[5]));
  }
  void set(double target, double now, double duration) {
    if (!std::isfinite(target) || !std::isfinite(now) ||
        !std::isfinite(duration) || duration<=0) throw std::invalid_argument("Invalid transition");
    if (target==target_) return;  // repeated targets must not postpone convergence
    const auto old=sample(now);
    const double acc=acceleration(now), d=target-old.value, T=duration;
    c_={old.value,old.rate,acc/2,
      10*d/(T*T*T)-6*old.rate/(T*T)-1.5*acc/T,
      -15*d/std::pow(T,4)+8*old.rate/std::pow(T,3)+1.5*acc/(T*T),
      6*d/std::pow(T,5)-3*old.rate/std::pow(T,4)-.5*acc/std::pow(T,3)};
    target_=target; start_=now; duration_=T;
  }
private:
  std::array<double,6> c_{};
  double target_{0}, start_{0}, duration_{0};
};

// Positive modulation is interpolated in log space so rapid retargeting cannot
// reverse frequency or invert the waveform through polynomial overshoot.
class PositiveTransition {
public:
  Sample sample(double now) const {
    const auto log_value = curve_.sample(now);
    const double value = std::exp(log_value.value);
    return {value, value * log_value.rate};
  }
  void set(double target, double now, double duration) {
    if (!std::isfinite(target) || target <= 0)
      throw std::invalid_argument("Positive transition target required");
    curve_.set(std::log(target), now, duration);
  }
private:
  Transition curve_;
};

struct Harmonics { std::array<Sample,4> x{}, y{}; };
inline Harmonics harmonics(double x,double y,double dx,double dy) {
  Harmonics h;
  h.x[1]={x,dx}; h.y[1]={y,dy};
  // Complex powers (x+iy)^n; retains the legacy radial behaviour.
  for (int n=2;n<=3;++n) {
    const auto a=h.x[n-1]*Sample{x,dx};
    const auto b=h.y[n-1]*Sample{y,dy};
    h.x[n]={a.value-b.value,a.rate-b.rate};
    h.y[n]=h.x[n-1]*Sample{y,dy}+h.y[n-1]*Sample{x,dx};
  }
  return h;
}
struct Coefficients {
  Sample offset{};
  std::array<Sample,4> x{},y{};
};
inline Sample readout(const Coefficients &w,const Harmonics &h) {
  Sample q=w.offset;
  for (int n=1;n<=3;++n) q=q+w.x[n]*h.x[n]+w.y[n]*h.y[n];
  return q;
}
}  // namespace cpg_controller
