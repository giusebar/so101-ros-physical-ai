"""
Shared utilities for so101_safety's perception-driven safety monitors.

Both ``depth_safety_monitor`` (proximity via monocular depth) and
``person_safety_monitor`` (presence via object detection) share the same
overall shape: parse a normalised center ROI, and debounce a per-frame
boolean "near"/"present" signal with hysteresis before asserting/releasing
``/safety/protective_stop``. Factored out here so both stay in sync and
neither duplicates this logic.
"""

from collections import deque


def parse_roi(value):
    """Parse a normalised "x1,y1,x2,y2" ROI string (or 4-item sequence) into
    a 4-tuple of floats in [0, 1]."""
    vals = (
        [float(v) for v in value.split(",")]
        if isinstance(value, str)
        else [float(v) for v in value]
    )
    if len(vals) != 4 or not all(0.0 <= v <= 1.0 for v in vals):
        raise ValueError("roi must be 4 normalised values x1,y1,x2,y2 in [0,1]")
    return vals


class HysteresisDebouncer:
    """Debounces a per-frame boolean signal with separate assert/release
    windows, to avoid flickering the safety-stop state on single noisy
    frames.

    Stop is asserted once the last ``frames_to_block`` samples are all
    True, and released once the last ``frames_to_clear`` samples are all
    False. Otherwise the previous state is held.
    """

    def __init__(self, frames_to_block: int, frames_to_clear: int):
        self._n_block = int(frames_to_block)
        self._n_clear = int(frames_to_clear)
        self._hist = deque(maxlen=max(self._n_block, self._n_clear))
        self._state = False

    def update(self, is_triggered: bool) -> bool:
        """Push one frame's boolean reading and return the current
        (debounced) stop state."""
        self._hist.append(bool(is_triggered))
        block = list(self._hist)[-self._n_block:]
        clear = list(self._hist)[-self._n_clear:]
        if len(block) == self._n_block and all(block):
            self._state = True
        elif len(clear) == self._n_clear and not any(clear):
            self._state = False
        return self._state

    @property
    def state(self) -> bool:
        return self._state


def roi_pixel_bounds(roi, width: int, height: int):
    """Convert a normalised (x1, y1, x2, y2) ROI into integer pixel bounds
    for an image of the given width/height."""
    x1, y1, x2, y2 = roi
    return int(x1 * width), int(y1 * height), int(x2 * width), int(y2 * height)


def draw_roi_border(u8_image, roi, marker: int, thickness: int = 2):
    """Draw a bright/dark rectangle border along the ROI boundary directly
    into a mono8 numpy array (plain-numpy, no OpenCV dependency), in place."""
    h, w = u8_image.shape[:2]
    ix1, iy1, ix2, iy2 = roi_pixel_bounds(roi, w, h)
    u8_image[iy1:iy1 + thickness, ix1:ix2] = marker
    u8_image[max(iy2 - thickness, 0):iy2, ix1:ix2] = marker
    u8_image[iy1:iy2, ix1:ix1 + thickness] = marker
    u8_image[iy1:iy2, max(ix2 - thickness, 0):ix2] = marker
    return u8_image
