"""Capture one calibrated block-number template on OpenMV.

Set ``BLOCK_ID`` below, run the script from OpenMV IDE, center the printed block
number in the yellow rectangle, and keep the camera still until the countdown
finishes. The script saves ``/templates/<BLOCK_ID>.pgm``.
"""

import os
import sensor
import time


# Change this value before each capture. Valid block numbers are 1..28.
BLOCK_ID = 21

# QVGA is 320x240. Keep this ROI smaller than the search ROI in main.py.
TEMPLATE_ROI = (100, 60, 120, 120)
PREVIEW_SECONDS = 8
TEMPLATE_DIR = '/templates'


def init_sensor():
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.skip_frames(time=1500)
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)


def ensure_template_dir():
    try:
        os.mkdir(TEMPLATE_DIR)
    except OSError:
        pass


def validate_roi(img):
    x, y, width, height = TEMPLATE_ROI
    if BLOCK_ID < 1 or BLOCK_ID > 28:
        raise ValueError('BLOCK_ID must be in range 1..28')
    if x < 0 or y < 0 or width <= 0 or height <= 0:
        raise ValueError('TEMPLATE_ROI is invalid')
    if x + width > img.width() or y + height > img.height():
        raise ValueError('TEMPLATE_ROI is outside the image')


def preview_countdown():
    start = time.ticks_ms()
    duration_ms = PREVIEW_SECONDS * 1000
    last_second = -1

    while time.ticks_diff(time.ticks_ms(), start) < duration_ms:
        img = sensor.snapshot()
        validate_roi(img)
        img.draw_rectangle(TEMPLATE_ROI, color=(255, 255, 0), thickness=2)
        remaining = PREVIEW_SECONDS - (
            time.ticks_diff(time.ticks_ms(), start) // 1000
        )
        if remaining != last_second:
            print('Capturing block %d in %d seconds' % (BLOCK_ID, remaining))
            last_second = remaining


def capture_template():
    img = sensor.snapshot()
    validate_roi(img)
    gray = img.to_grayscale(copy=True)
    gray.histeq()
    template = gray.copy(roi=TEMPLATE_ROI)
    path = '%s/%d.pgm' % (TEMPLATE_DIR, BLOCK_ID)
    template.save(path)
    print('Template saved:', path, template.width(), template.height())
    return template


def main():
    init_sensor()
    ensure_template_dir()
    preview_countdown()
    capture_template()
    print('Capture complete. Open the saved PGM file in OpenMV IDE to inspect it.')


main()
