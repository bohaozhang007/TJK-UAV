"""Offline clock-fit and conservative-match regression tests."""
import unittest
import json
import tempfile
import subprocess
from pathlib import Path
import cv2
import numpy as np
from replace_detection_frames import (fit_clock, synchronize, load_records,
                                      unpaired_images, replacement_slots, render, find_ffmpeg, parse_args)


class SynchronizationTests(unittest.TestCase):
    def test_timestamp_resolves_static_scene_with_precise_clock(self):
        rng = np.random.default_rng(73)
        matrix = rng.normal(size=(1201, 64)).astype(np.float32)
        matrix /= np.linalg.norm(matrix, axis=1, keepdims=True)
        slots = np.array([100, 300, 500, 700, 900, 1100])
        queries = matrix[slots].copy()
        # A static burst cannot provide an exact visual frame, but has a capture timestamp.
        matrix[497:504] = queries[2]
        records = [dict(timestamp_s=1000. + i * 20, image='unused', kind='track') for i in range(6)]
        report = synchronize(records, queries, matrix, np.arange(1201) / 10, .94, .01)
        row = report['frames'][2]
        # The earliest identical frame should not contaminate the clock fit.
        self.assertEqual(row['status'], 'matched')
        self.assertEqual(row['video_frame'], 500)
        self.assertEqual(row['frame_selection'], 'calibrated_timestamp')

    def test_cli_default_and_custom_output(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            video = root / 'flight.mkv'
            video.touch()
            imgs = root / 'session' / 'vis'
            imgs.mkdir(parents=True)
            args = parse_args(['--video', str(video), '--imgs', str(imgs)])
            self.assertEqual(args.video, video.resolve())
            self.assertEqual(args.output, imgs.parent / 'merge.mp4')
            custom = root / 'custom.mp4'
            args = parse_args(['--video', str(video), '--imgs', str(imgs), '--output', str(custom)])
            self.assertEqual(args.output, custom.resolve())

    def test_track_uses_timestamp_metadata_and_unpaired_is_skipped(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            for i in range(4):
                (root / f'track_{i:02d}.png').touch()
                (root / f'track_{i:02d}.json').write_text(json.dumps({'timestamp_s': 1000 + i * 10}))
            (root / 'track_04.png').touch()
            (root / 'track_04.json').write_text('{}')
            rows = load_records(root)
            self.assertEqual(len(rows), 4)
            self.assertTrue(all(r['kind'] == 'track' and r['alignment_method'] == 'clock_and_visual' for r in rows))
            self.assertEqual([r['timestamp_s'] for r in rows], [1000., 1010., 1020., 1030.])
            skipped = unpaired_images(root, rows)
            self.assertEqual(len(skipped), 1)
            self.assertEqual(Path(skipped[0]['image']).name, 'track_04.png')
            self.assertEqual(skipped[0]['status'], 'skipped')

    def test_trigger_jpg_and_reacquire_metadata_loaded(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            for i, name in enumerate(['trigger', 'reacquire_001', 'patrol_002', 'patrol_003']):
                image = name + ('.jpg' if name == 'trigger' else '_top3.png')
                (root / image).touch()
                (root / (name + '.json')).write_text(json.dumps({'timestamp_s': 1000 + i}))
            rows = load_records(root)
            self.assertEqual(len(rows), 4)
            self.assertEqual(rows[0]['kind'], 'trigger')
            self.assertEqual(rows[1]['kind'], 'reacquire')

    def test_same_frame_annotation_priority_and_reporting(self):
        rows = [dict(status='matched', kind=kind, video_s=1., score=score)
                for kind, score in [('patrol', .999), ('trigger', .998), ('track', .995)]]
        slots = replacement_slots(rows, 30.)
        self.assertEqual(len(slots), 1)
        self.assertEqual(slots[30]['kind'], 'track')
        self.assertEqual([r['status'] for r in rows], ['superseded', 'superseded', 'matched'])

    @unittest.skipUnless(find_ffmpeg(), 'FFmpeg required')
    def test_output_420_and_actual_replacement(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            source, output = root / 'source.mp4', root / 'output.mp4'
            writer = cv2.VideoWriter(str(source), cv2.VideoWriter_fourcc(*'mp4v'), 30., (128, 72))
            self.assertTrue(writer.isOpened())
            for i in range(30):
                writer.write(np.full((72, 128, 3), 30, np.uint8))
            writer.release()
            image = root / 'track_00.png'
            cv2.imwrite(str(image), np.full((72, 128, 3), 220, np.uint8))
            report = dict(frames=[dict(status='matched', kind='track', video_s=.5,
                                      score=1., image=str(image))])
            render(source, output, report, np.arange(30) / 30., 30., find_ffmpeg())
            probe = subprocess.run([find_ffmpeg(), '-hide_banner', '-i', str(output)],
                                   capture_output=True, text=True)
            self.assertIn('yuv420p', probe.stderr)
            self.assertIn('h264 (High)', probe.stderr)
            cap = cv2.VideoCapture(str(output))
            cap.set(cv2.CAP_PROP_POS_FRAMES, 15)
            ok, frame = cap.read()
            cap.release()
            self.assertTrue(ok)
            self.assertGreater(frame.mean(), 210)
            self.assertEqual(report['replaced_frames'], 1)

    def test_clock_offset_drift_and_outliers(self):
        x = np.arange(20, dtype=float) * 5
        y = 1.002 * x + 13.4
        y[[3, 9, 17]] += [20, -15, 25]
        a, b, mask = fit_clock(x, y)
        self.assertAlmostEqual(a, 1.002, places=6)
        self.assertAlmostEqual(b, 13.4, places=6)
        self.assertEqual(int(mask.sum()), 17)

    def test_inconsistent_clock_rejected(self):
        with self.assertRaises(ValueError):
            fit_clock(np.arange(8) * 10., np.arange(8) * 25.)

    def test_flat_static_images_rejected(self):
        records = [dict(timestamp_s=1000. + i * 10, image='unused') for i in range(6)]
        with self.assertRaises(ValueError):
            synchronize(records, np.ones((6, 1)), np.ones((101, 1)),
                        np.arange(101, dtype=float), .94, .01)

    def test_exact_matches_with_unrelated_image_skipped(self):
        rng = np.random.default_rng(42)
        matrix = rng.normal(size=(1201, 64)).astype(np.float32)
        matrix /= np.linalg.norm(matrix, axis=1, keepdims=True)
        slots = np.array([100, 300, 500, 700, 900, 1100])
        queries = matrix[slots].copy()
        queries[2] *= -1
        records = [dict(timestamp_s=1000. + i * 20, image='unused') for i in range(6)]
        result = synchronize(records, queries, matrix, np.arange(1201) / 10, .94, .01)
        self.assertAlmostEqual(result['video_offset_s'], 10.)
        self.assertAlmostEqual(result['clock_scale'], 1.)
        self.assertEqual(sum(r['status'] == 'matched' for r in result['frames']), 5)
        self.assertEqual(result['frames'][2]['status'], 'skipped')


if __name__ == '__main__':
    unittest.main()
