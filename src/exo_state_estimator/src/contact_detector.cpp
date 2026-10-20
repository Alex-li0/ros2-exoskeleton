#include "exo_state_estimator/contact_detector.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace exo_state_estimator {

ContactState SupportState::state() const {
  if (!left.valid || !right.valid) return ContactState::UNKNOWN;
  return static_cast<ContactState>((right.contact ? 1 : 0) | (left.contact ? 2 : 0));
}

ContactState SupportState::stableState() const {
  if (!left.valid || !right.valid) return ContactState::UNKNOWN;
  return static_cast<ContactState>((right.stable ? 1 : 0) | (left.stable ? 2 : 0));
}

GaitContactTracker::GaitContactTracker(double timeout, double min_confidence)
    : timeout_(timeout), min_confidence_(min_confidence) {
  if (!(timeout > 0) || !std::isfinite(timeout) ||
      !std::isfinite(min_confidence) || min_confidence < 0 || min_confidence > 1) {
    throw std::invalid_argument("Invalid gait contact timeout/confidence");
  }
}

bool GaitContactTracker::enqueue(const Record& record) {
  if (record.stamp <= committed_stamp_) {
    late_rejection_ = true;
    // A delayed exit cannot repair an already optimized state. Revoke support.
    feet_[record.foot - 1].support.valid = false;
    feet_[record.foot - 1].support.stable = false;
    feet_[record.foot - 1].interval_open = false;
    return false;
  }
  if (records_.size() >= 2000) {
    records_.clear();
    for (auto& foot : feet_) { foot.support = FootSupport{}; foot.interval_open = false; }
    return false;
  }
  auto it = std::upper_bound(records_.begin(), records_.end(), record.stamp,
      [](double stamp, const Record& value) { return stamp < value.stamp; });
  records_.insert(it, record);
  return true;
}

bool GaitContactTracker::addEvent(double stamp, uint8_t foot, GaitEventType event,
                                   uint64_t id, bool valid, double confidence) {
  late_rejection_ = false;
  if (foot < 1 || foot > 2 || !std::isfinite(stamp) || stamp < 0 ||
      static_cast<uint8_t>(event) < 1 || static_cast<uint8_t>(event) > 7) return false;
  auto& history = feet_[foot - 1];
  if (history.has_event && (id <= history.event_id || stamp < history.event_stamp)) {
    return false;
  }
  if (!enqueue({stamp, foot, true, event, false, false,
                valid && std::isfinite(confidence) && confidence >= min_confidence_ &&
                confidence <= 1.0})) return false;
  history.has_event = true;
  history.event_id = id;
  history.event_stamp = stamp;
  return true;
}

bool GaitContactTracker::addObservation(double stamp, uint8_t foot, bool contact, bool stable,
                                         bool valid, double confidence) {
  late_rejection_ = false;
  if (foot < 1 || foot > 2 || !std::isfinite(stamp) || stamp < 0) return false;
  auto& history = feet_[foot - 1];
  if (stamp <= history.state_stamp) return false;
  if (!enqueue({stamp, foot, false, GaitEventType::INITIAL_CONTACT, stable, contact,
                valid && (!stable || contact) && std::isfinite(confidence) && confidence >= min_confidence_ &&
                confidence <= 1.0})) return false;
  history.state_stamp = stamp;
  return true;
}

SupportState GaitContactTracker::at(double stamp) {
  if (!std::isfinite(stamp) || stamp < committed_stamp_) return {};
  while (!records_.empty() && records_.front().stamp <= stamp) {
    const auto record = records_.front();
    records_.pop_front();
    auto& foot = feet_[record.foot - 1];
    if (foot.observation_stamp >= 0 && record.stamp - foot.observation_stamp > timeout_) {
      foot.support.stable = false;
      foot.interval_open = false;
    }
    if (!record.valid) {
      foot.support.valid = false;
      foot.support.stable = false;
      foot.interval_open = false;
    } else if (!record.is_event) {
      foot.support.valid = true;
      // A heartbeat may sustain, but never open/reopen, a support interval.
      foot.support.contact = record.contact;
      const bool stable = foot.interval_open && record.stable;
      if (stable && !foot.support.stable) ++foot.support.session;
      foot.support.stable = stable;
      foot.stability_stamp = record.stamp;
      if (!record.contact) foot.interval_open = false;
    } else if (record.event == GaitEventType::FULL_FOOT_CONTACT) {
      foot.support.valid = true;
      foot.support.contact = true;
      foot.interval_open = true;
      foot.support.stable = false;
    } else if (record.event == GaitEventType::HEEL_OFF ||
               record.event == GaitEventType::TOE_OFF ||
               record.event == GaitEventType::FINAL_CONTACT_OFF ||
               record.event == GaitEventType::INITIAL_CONTACT) {
      foot.support.valid = true;
      foot.support.stable = false;
      foot.interval_open = false;
      if (record.event == GaitEventType::FINAL_CONTACT_OFF) foot.support.contact = false;
      if (record.event == GaitEventType::INITIAL_CONTACT) foot.support.contact = true;
    }
    foot.observation_stamp = record.stamp;
  }
  committed_stamp_ = stamp;
  for (auto& foot : feet_) {
    if (foot.stability_stamp < 0 || stamp - foot.stability_stamp > timeout_) {
      foot.support.stable = false;
    }
    if (foot.observation_stamp < 0 || stamp - foot.observation_stamp > timeout_) {
      foot.support.valid = false;
      foot.support.stable = false;
      foot.interval_open = false;
    }
  }
  return {feet_[0].support, feet_[1].support};
}

void GaitContactTracker::reset() {
  feet_ = {};
  records_.clear();
  committed_stamp_ = -1.0;
  late_rejection_ = false;
}

}  // namespace exo_state_estimator
