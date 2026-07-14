"""OpenMV-side recognition and result transmitter.

Copy this file to ``main.py`` on the OpenMV device. The host receives only the
recognition result; image pixels never leave OpenMV.
"""

import gc
import image
import sensor

try:
    from machine import UART
except ImportError:
    # Compatibility with older STM32-based OpenMV firmware.
    from pyb import UART


# Communication. UART pins depend on the OpenMV model; check its pinout.
UART_ID = 3
BAUDRATE = 115200

# OpenMV uses LAB thresholds: (L min, L max, A min, A max, B min, B max).
# Tune this tuple in OpenMV IDE under the actual field lighting.
GREEN_LAB_THRESHOLD = (20, 95, -80, -8, -25, 80)

# Search the central field of view for templates /templates/1.pgm ... 28.pgm.
TEMPLATE_THRESHOLD = 0.82
TEMPLATE_STEP = 4
OCR_EVERY_N_FRAMES = 6
DIGIT_HOLD_FRAMES = 3
TEMPLATE_DIR = '/templates'
TEMPLATE_IDS = tuple(range(10, 29)) + tuple(range(1, 10))


def init_sensor():
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.skip_frames(time=1500)
    # Freeze color controls after settling so the LAB threshold stays stable.
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)


def init_output():
    return UART(UART_ID, BAUDRATE, timeout_char=20)


def green_ratio(img):
    """Estimate the fraction of green pixels without exporting the image."""
    blobs = img.find_blobs(
        [GREEN_LAB_THRESHOLD],
        pixels_threshold=1,
        area_threshold=1,
        x_stride=1,
        y_stride=1,
        merge=True,
        margin=1,
    )
    green_pixels = 0
    for blob in blobs:
        green_pixels += blob.pixels()
    ratio = green_pixels / float(img.width() * img.height())
    if ratio < 0.0:
        return 0.0
    if ratio > 1.0:
        return 1.0
    return ratio


def recognize_block_number(img):
    """Recognize block 1..28 from calibrated grayscale templates.

    Templates should be captured at the normal flight height and stored as
    ``/templates/1.pgm`` through ``/templates/28.pgm``. Missing templates are
    skipped, so they can be added incrementally during calibration.
    """
    gray = img.to_grayscale(copy=True)
    margin_x = gray.width() // 8
    margin_y = gray.height() // 8
    roi = (
        margin_x,
        margin_y,
        gray.width() - margin_x * 2,
        gray.height() - margin_y * 2,
    )

    for block_id in TEMPLATE_IDS:
        template = None
        try:
            template = image.Image('%s/%d.pgm' % (TEMPLATE_DIR, block_id))
            match = gray.find_template(
                template,
                TEMPLATE_THRESHOLD,
                roi=roi,
                step=TEMPLATE_STEP,
                search=image.SEARCH_EX,
            )
            if match is not None:
                return block_id
        except Exception:
            # Missing or incompatible templates are skipped during calibration.
            pass
        finally:
            template = None
            gc.collect()
    return None


def xor_checksum(text):
    checksum = 0
    for char in text:
        checksum ^= ord(char)
    return checksum


def build_result_frame(sequence, ratio, digit):
    green_per_mille = int(max(0, min(1000, round(ratio * 1000))))
    digit_value = -1 if digit is None else digit
    body = 'OMV1,%d,%d,%d' % (sequence, green_per_mille, digit_value)
    return '$%s*%02X\r\n' % (body, xor_checksum(body))


init_sensor()
output = init_output()
sequence = 0
frame_count = 0
last_digit = None
digit_hold = 0

while True:
    frame = sensor.snapshot()
    ratio = green_ratio(frame)

    if frame_count % OCR_EVERY_N_FRAMES == 0:
        detected = recognize_block_number(frame)
        if detected is not None:
            last_digit = detected
            digit_hold = DIGIT_HOLD_FRAMES
        else:
            last_digit = None
            digit_hold = 0
    elif digit_hold > 0:
        digit_hold -= 1
        if digit_hold <= 0:
            last_digit = None

    output.write(build_result_frame(sequence, ratio, last_digit))
    sequence = (sequence + 1) & 0xFFFF
    frame_count += 1
