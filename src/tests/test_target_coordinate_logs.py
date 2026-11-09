"""Verify diagnostics explain the actual gates without changing selection."""

from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from util.dedup import match_rechecked_targets, select_new_targets
from util.target_log import log_target, target_snapshot


def detection(x):
    return {"world_frame": "world", "position_world_m": [x, 0, 1],
            "position_camera_m": [0, 0, 5], "confidence": 0.8, "box": [1, 1, 10, 15]}


def records(output):
    return [json.loads(line.split("[TIMING] ", 1)[1]) for line in output.getvalue().splitlines()]


class CoordinateLogTests(unittest.TestCase):
    def test_dedup_reports_reference_and_skip_then_new_record_at_boundary(self):
        history = {}
        out = io.StringIO()
        with redirect_stdout(out):
            first = select_new_targets([detection(0)], history)[0]
            first["_record"]["observed"] = True
            self.assertEqual(select_new_targets([detection(-1)], history), [])
            second = select_new_targets([detection(1.8)], history)[0]
        rows = records(out)
        self.assertEqual(rows[1]["reason"], "already_observed")
        self.assertEqual(rows[1]["comparisons"][0]["delta_xyz_m"], [-1, 0, 0])
        self.assertEqual(second["target_id"], 2)
        self.assertEqual(rows[2]["reason"], "new_target")
        self.assertFalse(rows[2]["comparisons"][0]["within_gate"])

    def test_recheck_explains_distance_and_ambiguity(self):
        original = {**detection(0), "target_id": 1, "_record": {}}
        img = np.zeros((20, 20, 3), dtype=np.uint8)
        out = io.StringIO()
        with redirect_stdout(out):
            matched, missing = match_rechecked_targets([original], [detection(2)], img)
            match_rechecked_targets([original], [detection(0), detection(1)], img)
        self.assertFalse(matched)
        self.assertEqual(missing, [original])
        rows = records(out)
        self.assertEqual(rows[0]["reason"], "no_candidate_within_gate")
        self.assertEqual(rows[0]["comparisons"][0]["distance_m"], 2)
        self.assertEqual(rows[1]["reason"], "ambiguous_candidates")

    def test_invalid_numbers_are_json_safe_and_images_are_excluded(self):
        value = {**detection(0), "confidence": np.float32(0.8),
                 "position_camera_m": np.array([np.nan, np.inf, 1]),
                 "exposure_img": np.ones((20, 20, 3)), "_record": {"positions": []}}
        out = io.StringIO()
        with redirect_stdout(out):
            log_target("example", target=target_snapshot(value))
        saved = records(out)[0]["target"]
        self.assertEqual(saved["position_camera_m"], [None, None, 1])
        self.assertNotIn("exposure_img", saved)
        self.assertNotIn("_record", saved)
        self.assertTrue(np.isnan(value["position_camera_m"][0]))


if __name__ == "__main__":
    unittest.main()
