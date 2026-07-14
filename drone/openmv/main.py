"""OpenMV-side recognition and result transmitter.

Copy this file to ``main.py`` on the OpenMV device. The host receives only the
recognition result; image pixels never leave OpenMV.
"""

import gc
import image
import os
import sensor
import time

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
ENABLE_GREEN_DETECTION = True

# Search the central field of view for templates /templates/1.pgm ... 28.pgm.
ENABLE_TEMPLATE_OCR = True
TEMPLATE_THRESHOLD = 0.82
TEMPLATE_STEP = 4
OCR_EVERY_N_FRAMES = 2
OCR_TEMPLATES_PER_PASS = 1
DIGIT_HOLD_FRAMES = 3
LOOP_DELAY_MS = 5
GC_EVERY_N_FRAMES = 30
TEMPLATE_DIR = '/templates'
TEMPLATE_IDS = tuple(range(10, 29)) + tuple(range(1, 10))
DEBUG_PRINT = True


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


def find_available_template_ids():
    """Scan template files once so missing files do not slow every OCR pass."""
    available = []
    for block_id in TEMPLATE_IDS:
        try:
            os.stat('%s/%d.pgm' % (TEMPLATE_DIR, block_id))
            available.append(block_id)
        except OSError:
            pass
    return available


def green_ratio(img):
    """Estimate the fraction of green pixels without exporting the image."""
    blobs = img.find_blobs(
        [GREEN_LAB_THRESHOLD],
        pixels_threshold=20,
        area_threshold=20,
        x_stride=2,
        y_stride=2,
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


def recognize_block_number(img, template_ids, start_index):
    """Recognize block 1..28 from calibrated grayscale templates.

    Templates should be captured at the normal flight height and stored as
    ``/templates/1.pgm`` through ``/templates/28.pgm``. Missing templates are
    skipped, so they can be added incrementally during calibration. Only a
    small batch is checked per call to keep the IDE and USB connection alive.
    """
    if not template_ids:
        return None, 0, True

    gray = img.to_grayscale(copy=True)
    gray.histeq()
    margin_x = gray.width() // 8
    margin_y = gray.height() // 8
    roi = (
        margin_x,
        margin_y,
        gray.width() - margin_x * 2,
        gray.height() - margin_y * 2,
    )

    end_index = min(start_index + OCR_TEMPLATES_PER_PASS, len(template_ids))
    detected = None
    for index in range(start_index, end_index):
        block_id = template_ids[index]
        template = None
        try:
            template = image.Image('%s/%d.pgm' % (TEMPLATE_DIR, block_id))
            if template.width() > roi[2] or template.height() > roi[3]:
                continue
            match = gray.find_template(
                template,
                TEMPLATE_THRESHOLD,
                roi=roi,
                step=TEMPLATE_STEP,
            )
            if match is not None and detected is None:
                detected = block_id
        except Exception:
            # Missing or incompatible templates are skipped during calibration.
            pass
        finally:
            template = None
            gc.collect()

    cycle_finished = end_index >= len(template_ids)
    next_index = 0 if cycle_finished else end_index
    return detected, next_index, cycle_finished


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


def main():
    init_sensor()
    # output = init_output()
    template_ids = find_available_template_ids()
    sequence = 0
    frame_count = 0
    last_digit = None
    digit_hold = 0
    template_cursor = 0
    cycle_had_detection = False

    print('OpenMV vision started: UART%d @ %d' % (UART_ID, BAUDRATE))
    print('Templates found: %d/28' % len(template_ids))

    while True:
        try:
            frame = sensor.snapshot()
            ratio = green_ratio(frame) if ENABLE_GREEN_DETECTION else 0.0

            if (
                ENABLE_TEMPLATE_OCR
                and template_ids
                and frame_count % OCR_EVERY_N_FRAMES == 0
            ):
                detected, template_cursor, cycle_finished = recognize_block_number(
                    frame, template_ids, template_cursor
                )
                if detected is not None:
                    last_digit = detected
                    digit_hold = DIGIT_HOLD_FRAMES
                    cycle_had_detection = True
                if cycle_finished and not cycle_had_detection:
                    last_digit = None
                    digit_hold = 0
                if cycle_finished:
                    cycle_had_detection = False
            elif digit_hold > 0 and not cycle_had_detection:
                digit_hold -= 1
                if digit_hold <= 0:
                    last_digit = None

            result_frame = build_result_frame(sequence, ratio, last_digit)
            # output.write(result_frame)
            if DEBUG_PRINT and frame_count % 10 == 0:
                print(result_frame.strip())

            sequence = (sequence + 1) & 0xFFFF
            frame_count += 1
            if frame_count % GC_EVERY_N_FRAMES == 0:
                gc.collect()
            time.sleep_ms(LOOP_DELAY_MS)
        except Exception as exc:
            # Do not send a fabricated result. The host will invalidate stale data.
            print('vision loop error:', exc)
            time.sleep_ms(100)


main()
