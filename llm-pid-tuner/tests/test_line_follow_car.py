import unittest

from core.buffer import AdvancedDataBuffer
from pid_safety import score_metrics, should_rollback_to_best
from hw.bridge import SerialBridge


def add_samples(buffer, positions):
    for index, position in enumerate(positions):
        buffer.add({
            "hardware_profile": "mspm0_line_follow_car",
            "timestamp": index * 20,
            "setpoint": 0,
            "input": position,
            "pwm": -position,
            "error": -position,
            "p": 50,
            "i": 0,
            "d": 16,
        })


class LineFollowMetricsTests(unittest.TestCase):
    def test_running_field_is_parsed_from_ninth_column(self):
        bridge = SerialBridge("DEMO", 115200, emit_console=False)
        bridge.hardware_profile = "mspm0_line_follow_car"
        stopped = bridge.parse_data("100,0,14,2,-14,50,0,16,0")
        running = bridge.parse_data("150,0,14,2,-14,50,0,16,1")
        self.assertFalse(stopped["running"])
        self.assertTrue(running["running"])

    def test_lost_line_is_detected_and_heavily_penalized(self):
        stable = AdvancedDataBuffer(max_size=10)
        lost = AdvancedDataBuffer(max_size=10)
        add_samples(stable, [14] * 10)
        add_samples(lost, [0] * 9 + [110])

        stable_metrics = stable.calculate_advanced_metrics()
        lost_metrics = lost.calculate_advanced_metrics()

        self.assertEqual(lost_metrics["status"], "LINE_LOST")
        self.assertEqual(lost_metrics["lost_count"], 1)
        self.assertGreater(score_metrics(lost_metrics), score_metrics(stable_metrics))
        self.assertTrue(should_rollback_to_best(lost_metrics, stable_metrics))

    def test_edge_hits_are_not_reported_as_stable(self):
        buffer = AdvancedDataBuffer(max_size=10)
        add_samples(buffer, [71, 43, 14, -14, -43, -71, -43, -14, 14, 43])
        metrics = buffer.calculate_advanced_metrics()
        self.assertEqual(metrics["status"], "EDGE_RISK")
        self.assertEqual(metrics["edge_count"], 2)


if __name__ == "__main__":
    unittest.main()
