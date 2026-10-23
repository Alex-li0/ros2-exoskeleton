// Offline adapter for the exact same core used by the ROS shadow node.
// Input has no truth labels: now,stamp,q0..q5,event_stamp,event_foot,event_id,event_phase.
#include <array>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include "cpg_controller/adaptive_cpg.hpp"

int main(int argc, char **argv) {
  try {
    if (argc < 5) throw std::runtime_error(
        "Usage: adaptive_learning_probe TEMPLATE INPUT OUTPUT frozen|sync|full|joint|no_events|joint_no_events [kp ki kw]");
    std::ifstream model(argv[1]), input(argv[2]);
    std::ofstream output(argv[3]);
    if (!model || !input || !output) throw std::runtime_error("Cannot open experiment file");
    cpg_controller::AdaptiveConfig cfg;
    model >> cfg.omega_initial;
    std::array<cpg_controller::Coefficients, 6> prior{};
    for (auto &joint : prior) {
      model >> joint.offset.value;
      for (size_t n = 1; n <= 3; ++n) model >> joint.x[n].value >> joint.y[n].value;
    }
    if (!model) throw std::runtime_error("Invalid prior template");
    const std::string mode(argv[4]);
    if (mode != "frozen" && mode != "sync" && mode != "full" && mode != "joint" && mode != "no_events" && mode != "joint_no_events")
      throw std::runtime_error("Unknown learning ablation");
    cfg.shape_enabled = mode != "sync" && mode != "frozen";
    cfg.events_enabled = mode != "no_events" && mode != "joint_no_events" && mode != "frozen";
    cfg.shape_requires_lock = mode != "joint" && mode != "joint_no_events";
    if (argc >= 8) {
      cfg.phase_gain = std::stod(argv[5]);
      cfg.frequency_gain = std::stod(argv[6]);
      cfg.shape_gain = std::stod(argv[7]);
    }
    std::string line;
    std::getline(input, line);  // CSV header
    bool initialized = false;
    std::unique_ptr<cpg_controller::AdaptiveCPG> core;
    output << "now,phase,omega_base,phase_rate,phase_error,residual_rms,state,shape_active,measurement_age";
    for (size_t j=0;j<6;++j) output << ",q" << j;
    for (size_t j=0;j<6;++j) output << ",qd" << j;
    for (size_t j=0;j<6;++j) {
      output << ",b" << j;
      for (size_t n=1;n<=3;++n) output << ",a" << j << n << ",c" << j << n;
    }
    output << '\n' << std::setprecision(12);
    size_t index = 0;
    while (std::getline(input, line)) {
      if (line.empty()) continue;
      for (char &c : line) if (c == ',') c=' ';
      std::istringstream row(line);
      double now, stamp, event_stamp, event_phase;
      int foot;
      uint64_t id;
      cpg_controller::MeasurementFrame frame;
      row >> now >> stamp;
      frame.stamp = stamp;
      for (size_t j=0;j<6;++j) {
        std::string field;
        row >> field;
        frame.position[j] = std::stod(field);
        frame.valid[j] = std::isfinite(frame.position[j]);
      }
      row >> event_stamp >> foot >> id >> event_phase;
      if (!row) throw std::runtime_error("Malformed measurement CSV row");
      if (!initialized) {
        core = std::make_unique<cpg_controller::AdaptiveCPG>(prior, cfg, now);
        initialized = true;
      }
      core->step(now);
      if (mode != "frozen") {
        if (stamp > 0) core->observe(frame, now);
        if (foot >= 0) core->event(event_stamp, static_cast<size_t>(foot), id, .98, event_phase, now);
      }
      if (index++ % 5 != 0) continue;  // 100 Hz exported trace; integration remains 500 Hz
      const auto state = core->output();
      output << now << ',' << state.phase << ',' << state.omega_base << ',' << state.phase_rate
             << ',' << state.phase_error << ',' << state.residual_rms << ',' << static_cast<int>(state.state)
             << ',' << state.shape_active << ',' << state.measurement_age;
      for (auto v : state.position) output << ',' << v;
      for (auto v : state.velocity) output << ',' << v;
      for (const auto &joint : core->weights()) {
        output << ',' << joint.offset.value;
        for (size_t n=1;n<=3;++n) output << ',' << joint.x[n].value << ',' << joint.y[n].value;
      }
      output << '\n';
    }
    if (!initialized) throw std::runtime_error("Empty experiment input");
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
  return 0;
}
