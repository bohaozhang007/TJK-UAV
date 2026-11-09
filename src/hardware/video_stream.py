"""RTSP setup shared by the camera driver and the standalone connection check."""

import os

import cv2


RTSP_URL = "rtsp://192.168.144.64:558/live/single"
STREAM_OPEN_TIMEOUT_S = 3.0
STREAM_READ_TIMEOUT_S = 1.0


def open_stream():
    """Open the K40T stream with support for Jetson's OpenCV 4.2."""
    cap = cv2.VideoCapture()
    try:
        if all(hasattr(cv2, name) for name in (
            "CAP_PROP_OPEN_TIMEOUT_MSEC",
            "CAP_PROP_READ_TIMEOUT_MSEC",
        )):
            opened = cap.open(
                RTSP_URL,
                cv2.CAP_FFMPEG,
                [
                    cv2.CAP_PROP_OPEN_TIMEOUT_MSEC,
                    int(STREAM_OPEN_TIMEOUT_S * 1000),
                    cv2.CAP_PROP_READ_TIMEOUT_MSEC,
                    int(STREAM_READ_TIMEOUT_S * 1000),
                ],
            )
        else:
            # OpenCV 4.2 accepts only URL and backend. Its FFmpeg RTSP
            # implementation uses stimeout (microseconds) for socket I/O.
            # Retain other caller options, but enforce bounded socket waits.
            options = os.environ.get("OPENCV_FFMPEG_CAPTURE_OPTIONS", "")
            options = [option for option in options.split("|") if option]
            options = [option for option in options if not option.startswith("stimeout;")]
            if not any(option.startswith("rtsp_transport;") for option in options):
                options.append("rtsp_transport;tcp")
            options.append(f"stimeout;{int(STREAM_READ_TIMEOUT_S * 1000000)}")
            os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "|".join(options)
            opened = cap.open(RTSP_URL, cv2.CAP_FFMPEG)
        if not opened:
            raise RuntimeError(f"Failed to open K40T stream: {RTSP_URL} (OpenCV {cv2.__version__})")
        return cap
    except Exception:
        cap.release()
        raise
