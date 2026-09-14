import json
from pathlib import Path
import sys
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"src"))
from agent.tjk.track_log import TrackImageLog


class TrackLogTests(unittest.TestCase):
    def test_metadata_mask_roundtrip_and_snapshot(self):
        with tempfile.TemporaryDirectory() as directory:
            event=Mock(); writer=TrackImageLog(event)
            masks=[np.zeros((10,12),bool),np.ones((10,12),bool),np.eye(10,12,dtype=bool)]
            for i,mask in enumerate(masks):
                obs=SimpleNamespace(rgb=np.zeros((10,12,3),np.uint8),frame_id=str(i),
                                    timestamp_s=10+i,pose=dict(z=100),localization_epoch='epoch')
                depth=np.full((10,12),123.)
                writer.submit(obs,[0,0,11,9],mask,directory=directory,depth=depth)
                obs.pose['z']=999
                depth[:]=999
            writer.close()
            for i,path in enumerate(sorted(Path(directory).glob('*.json'))):
                meta=json.loads(path.read_text())
                self.assertEqual(meta['timestamp_s'],10+i)
                self.assertEqual(meta['pose']['z'],100)
                counts=meta['mask']['counts']
                decoded=np.repeat(np.arange(len(counts))%2,counts).reshape(meta['mask']['size'])
                np.testing.assert_array_equal(decoded,masks[i])
                np.testing.assert_array_equal(np.load(path.parent/meta['depth_file']),123.)
                self.assertTrue((path.parent/meta['image_file']).is_file())
            event.assert_not_called()

    def test_slow_disk_does_not_block_submit(self):
        with tempfile.TemporaryDirectory() as directory:
            entered=threading.Event(); release=threading.Event()
            writer=TrackImageLog(Mock())
            obs=SimpleNamespace(rgb=np.zeros((10,12,3),np.uint8),frame_id='f',
                                timestamp_s=1,pose={},localization_epoch='e')
            import cv2
            original=cv2.imwrite
            def slow(*args):
                entered.set(); release.wait(3)
                return original(*args)
            with patch('agent.tjk.track_log.cv2.imwrite',side_effect=slow):
                try:
                    writer.submit(obs,[0,0,11,9],np.zeros((10,12),bool),directory=directory)
                    self.assertTrue(entered.wait(1))
                    writer.submit(obs,[0,0,11,9],np.zeros((10,12),bool),directory=directory)
                    self.assertEqual(writer.sequence,2)
                finally:
                    release.set();writer.close()


if __name__ == '__main__':
    unittest.main()
