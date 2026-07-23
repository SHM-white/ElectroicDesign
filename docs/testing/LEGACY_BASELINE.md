# Legacy Test Baseline

## Capture

This baseline was captured before task-1 behavior changes with the repository
virtual environment. The verbatim command output is retained under
`.omo/evidence/task-1/characterization/`:

- `git-status.txt` records the initial untracked `往年赛题/` source material.
- `protected-files.sha256`, `protected-files.diff`, and
  `protected-files.numstat` record the protected launch-script state.
- `tool-versions.txt` records Python 3.12.3, pytest 7.4.4, Git 2.43.0, and
  GNU sha256sum 9.4.
- `pytest-collect.txt` contains all 479 pre-change collected node IDs.
- `pytest-full.txt` contains the complete pre-change run and failure details.

## Protected Files

The captured diff and numstat are empty. The protected files must retain these
SHA-256 values until a task explicitly owns them:

| File | SHA-256 |
| --- | --- |
| `drone/start.sh` | `9658f7ea196c5801cb743e56db7495134d16104180382d1e0ac6c4d47518054e` |
| `drone/debug_start.sh` | `af24ba8afbffa6483ade8dd87a78a2d2f688243c5d57c486924dce45b00af85d` |
| `drone/field_test.sh` | `dda7ecb3348be65dc01356eb626420c4c3794c4aef75baa359a6fcff3bb1432b` |

## Original Pytest Result

The original command `./.venv/bin/python -m pytest` collected 479 nodes and
finished with 471 passed and 8 failed. `drone/test/test_all.py` accounted for
225 duplicate aggregate nodes; it remains runnable with `unittest` but is
ignored by pytest. `test_vision_regression.py` remains included in pytest.

The original failures were:

1. `drone/test/test_gray_marker.py::TestGrayMarkerDetector::test_detects_dark_saved_sample_center`
2. `drone/test/test_home_cross.py::TestHomeCrossDetector::test_detects_complete_saved_home_sample`
3. `drone/test/test_home_cross.py::TestHomeCrossDetector::test_detects_saved_home_sequence`
4. `drone/test/test_vision_regression.py::TestMvsSampleRecognition::test_all_saved_field_frames_are_readable`
5. `drone/test/test_vision_regression.py::TestMvsSampleRecognition::test_expected_digit_rejects_low_light_noise_ocr`
6. `drone/test/test_vision_regression.py::TestMvsSampleRecognition::test_finds_a_marker_in_both_block_21_samples`
7. `drone/test/test_vision_regression.py::TestMvsSampleRecognition::test_finds_a_marker_in_low_light_start_samples`
8. `drone/test/test_vision_regression.py::TestMvsSampleRecognition::test_recognizes_saved_low_light_two_digit_samples`

The `field_data` marker identifies all tests that depend on saved field images.
The first, second, third, fourth, fifth, seventh, and eighth failures reflect
unavailable `mission_vision_*.png` input. The sixth uses available images and
remains an algorithm-regression result when the strict field-data surface runs.
The checker reports image availability and hashes only; it never converts an
algorithm result into a fixture pass.

## Current External Data Gate

`drone/test/fixtures/field-images.json` explicitly records every absent
`mission_vision_*.png` fixture and preserves its OCR/marker expectation. Check
the state with:

```bash
./.venv/bin/python tools/check_field_fixtures.py \
  --manifest drone/test/fixtures/field-images.json \
  --expect-current-state
```

The command succeeds only when the filesystem exactly matches the manifest. It
prints every declared absence, and a restored, changed, unreadable, or missing
present fixture is a nonzero mismatch rather than a false success.
