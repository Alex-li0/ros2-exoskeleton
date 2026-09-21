import os
import tempfile
import unittest

import yaml

from impedance_tuning.config_io import save_controller_parameters


class ConfigIoTest(unittest.TestCase):
    def test_saves_gains_without_rewriting_comments(self):
        contents = """# keep this comment
exo_impedance_controller:
  ros__parameters:
    joints: [a, b]
    default_kp: [1.0, 2.0]  # kp comment
    default_kd: [3.0, 4.0]
    default_tube_half_width: [0.0, 0.0]  # tube comment
    default_tube_transition_width: [0.0, 0.0]
other_section:
  untouched: true
"""
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, 'controllers.yaml')
            with open(path, 'w', encoding='utf-8') as stream:
                stream.write(contents)
            save_controller_parameters(
                path, ['a', 'b'], [10.0, 20.0], [30.0, 40.0],
                [0.1, 0.2], [0.01, 0.02])
            with open(path, encoding='utf-8') as stream:
                saved_text = stream.read()
            saved = yaml.safe_load(saved_text)

        self.assertIn('# keep this comment', saved_text)
        self.assertIn('# kp comment', saved_text)
        self.assertIn('# tube comment', saved_text)
        self.assertIn('default_kp: [10.0, 20.0]', saved_text)
        self.assertIn('default_kd: [30.0, 40.0]', saved_text)
        self.assertEqual(
            saved['exo_impedance_controller']['ros__parameters']['default_kp'],
            [10.0, 20.0])
        self.assertEqual(
            saved['exo_impedance_controller']['ros__parameters']['default_kd'],
            [30.0, 40.0])
        self.assertEqual(
            saved['exo_impedance_controller']['ros__parameters'][
                'default_tube_half_width'], [0.1, 0.2])
        self.assertEqual(
            saved['exo_impedance_controller']['ros__parameters'][
                'default_tube_transition_width'], [0.01, 0.02])
        self.assertTrue(saved['other_section']['untouched'])

    def test_rejects_joint_order_mismatch(self):
        contents = """exo_impedance_controller:
  ros__parameters:
    joints: [a, b]
    default_kp: [1.0, 2.0]
    default_kd: [3.0, 4.0]
    default_tube_half_width: [0.0, 0.0]
    default_tube_transition_width: [0.0, 0.0]
"""
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, 'controllers.yaml')
            with open(path, 'w', encoding='utf-8') as stream:
                stream.write(contents)
            with self.assertRaises(ValueError):
                save_controller_parameters(
                    path, ['b', 'a'], [10.0, 20.0], [30.0, 40.0],
                    [0.1, 0.2], [0.01, 0.02])


if __name__ == '__main__':
    unittest.main()
