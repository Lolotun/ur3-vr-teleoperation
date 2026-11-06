import json
import math
from pathlib import Path
import tempfile
import unittest

from teleop_core.rtde_diagnostics import TRACE_TOPIC, json_safe, summarize
from teleop_core.ur3_model import UR3_ARM_JOINTS


class DiagnosticsTests(unittest.TestCase):
    def test_nonfinite_is_flagged_and_encoded_as_standard_json(self):
        result = json_safe({'q': (float('nan'), float('inf'), -float('inf'), 1.), 'nonfinite': True})
        self.assertEqual(result, {'q': [None, None, None, 1.], 'nonfinite': True})
        json.dumps(result, allow_nan=False)

    def test_summary_separates_preview_from_sent_commands_and_reports_gaps(self):
        rows = []
        for seq, sent, lead in [(1, False, 1.), (2, True, .02), (4, True, .14)]:
            rows.append({'topic': TRACE_TOPIC, 'data': {
                'sequence': seq, 'command_sent': sent, 'status': 'TRACKING' if sent else 'PREVIEW',
                'q_actual': [0.] * 6, 'q_cmd': [lead] * 6,
                'qd_actual': [.1] * 6, 'qd_servo': [.2] * 6, 'servo_status': 1,
                'cycle_interval_s': .008, 'timings_s': {'session_step_total': .001},
                'getSpeedScaling': .5, 'getTargetSpeedFraction': .78,
                'diagnostics_dropped': 1 if seq == 4 else 0}})
        rows.append({'topic': '/teleop/servo/joint_trajectory', 'data': {'nonfinite': True}})
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'trace.jsonl'
            path.write_text(''.join(json.dumps(row) + '\n' for row in rows))
            result = summarize(path)
        self.assertEqual(result['sent_samples'], 2)
        self.assertEqual(result['sequence_gaps'], 1)
        self.assertEqual(result['diagnostics_dropped_in_bridge'], 1)
        self.assertEqual(result['nonfinite_trajectory_messages'], 1)
        self.assertEqual(result['cycle_interval_ms']['max'], 8.)
        self.assertEqual(result['command_step_ms']['max'], 1.)
        self.assertEqual(result['joints'][UR3_ARM_JOINTS[0]]['abs_lead_rad']['max'], .14)
        self.assertEqual(result['speed_scaling_during_commands']['max'], .5)


if __name__ == '__main__':
    unittest.main()
