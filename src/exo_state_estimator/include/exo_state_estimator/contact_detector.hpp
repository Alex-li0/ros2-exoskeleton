#pragma once

#include <array>
#include <cstdint>
#include <deque>

namespace exo_state_estimator {

// Physical contact mask; UNKNOWN is distinct from confirmed no contact.
enum class ContactState : uint8_t {
  SWING = 0,
  RIGHT_STANCE = 1,
  LEFT_STANCE = 2,
  DOUBLE_STANCE = 3,
  UNKNOWN = 4
};

enum class GaitEventType : uint8_t {
  HEEL_CONTACT = 1, HEEL_OFF = 2, TOE_CONTACT = 3, TOE_OFF = 4,
  FULL_FOOT_CONTACT = 5, INITIAL_CONTACT = 6, FINAL_CONTACT_OFF = 7
};

struct FootSupport {
  bool valid = false;
  bool stable = false;
  bool contact = false;
  uint64_t session = 0;
};

struct SupportState {
  FootSupport left;
  FootSupport right;
  ContactState state() const;
  ContactState stableState() const;
};

// Events open/close support intervals; state observations confirm freshness.
// The estimator queries in measurement-time order after a short fusion delay.
class GaitContactTracker {
 public:
  explicit GaitContactTracker(double timeout = 0.15, double min_confidence = 0.8);
  bool addEvent(double stamp, uint8_t foot, GaitEventType event, uint64_t id,
                bool valid, double confidence);
  bool addObservation(double stamp, uint8_t foot, bool contact, bool stable,
                      bool valid, double confidence);
  SupportState at(double stamp);
  void reset();
  bool lastRejectionWasLate() const { return late_rejection_; }

 private:
  struct Record {
    double stamp;
    uint8_t foot;
    bool is_event;
    GaitEventType event;
    bool stable;
    bool contact;
    bool valid;
  };
  struct Foot {
    FootSupport support;
    bool interval_open = false;
    double observation_stamp = -1.0;
    double stability_stamp = -1.0;
    uint64_t event_id = 0;
    bool has_event = false;
    double event_stamp = -1.0;
    double state_stamp = -1.0;
  };
  bool enqueue(const Record& record);
  std::array<Foot, 2> feet_;
  std::deque<Record> records_;
  bool late_rejection_ = false;
  double timeout_;
  double min_confidence_;
  double committed_stamp_ = -1.0;
};

}  // namespace exo_state_estimator
