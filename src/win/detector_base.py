import pickle
import sys

import numpy as np


WARMUP_SHAPE = (1080, 1920, 3)


class Detector:
    def warmup(self):
        self.detect(np.zeros(WARMUP_SHAPE, dtype=np.uint8))


def run_detector(
    model_class,
    name,
    output,
):
    try:
        args = pickle.load(sys.stdin.buffer)
        print(f"[{name}] Loading model...", flush=True)
        model = model_class(*args)
        print(f"[{name}] Model loaded.", flush=True)
        print(f"[{name}] Warming up at 1920 x 1080...", flush=True)
        model.warmup()
        print(f"[{name}] Warmup complete.", flush=True)
    except Exception as exc:
        pickle.dump((None, str(exc)), output)
        output.flush()
        return
    pickle.dump((None, None), output)
    output.flush()

    while True:
        try:
            img, = pickle.load(sys.stdin.buffer)
        except EOFError:
            return
        try:
            result = model.detect(img)
        except Exception as exc:
            pickle.dump((None, str(exc)), output)
            output.flush()
        else:
            pickle.dump((result, None), output)
            output.flush()
