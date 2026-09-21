import math
import unittest

import numpy as np

from impedance_tuning.metrics import analyze_cycle, estimate_phase_lag_ms


class MetricsTest(unittest.TestCase):
    def test_phase_lag_is_positive_for_delayed_measurement(self):
        t = np.arange(0.0, 1.0, 0.002)
        reference = np.sin(2.0 * np.pi * t)
        measured = np.sin(2.0 * np.pi * (t - 0.020))
        self.assertLess(
            abs(estimate_phase_lag_ms(t, reference, measured) - 20.0), 3.0)

    def test_cycle_metrics(self):
        t = np.arange(0.0, 1.0, 0.002)
        samples = []
        for value in t:
            q_ref = math.sin(2.0 * math.pi * value)
            samples.append({
                't': value, 'q_ref': q_ref, 'q_meas': q_ref - 0.1,
                'iq_cmd': 2.0, 'iq_meas': 1.5, 'loop_dt': 0.002,
                'ref_age': 0.001,
            })
        result = analyze_cycle(samples, torque_limit=10.0)
        self.assertAlmostEqual(result['rms_rad'], 0.1)
        self.assertAlmostEqual(result['peak_rad'], 0.1)
        self.assertEqual(result['saturation_pct'], 0.0)
        self.assertAlmostEqual(result['loop_dt_p99_ms'], 2.0)


if __name__ == '__main__':
    unittest.main()
