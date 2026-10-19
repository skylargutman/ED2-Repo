import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from telemetry_payload import parse_payload

TEXT = '{"time": "0.100", "position": "0.0000", "angle": "3.1416", "control": "3.142"}'
# The R2026b Raspberry Pi Blockset MQTT Publish block sends every byte as "<decimal> "
CODED = " ".join(str(b) for b in TEXT.encode()) + " "


def test_plain_json():
    assert parse_payload(TEXT)["angle"] == "3.1416"


def test_unquoted_keys_still_work():
    assert parse_payload("{time: 1.5, angle: 3.1}") == {"time": 1.5, "angle": 3.1}


def test_byte_codes_from_r2026b_models_are_decoded():
    assert parse_payload(CODED) == {"time": "0.100", "position": "0.0000", "angle": "3.1416", "control": "3.142"}


def test_byte_codes_without_trailing_space():
    assert parse_payload(CODED.strip())["control"] == "3.142"


def test_plain_numbers_are_not_mistaken_for_byte_codes():
    # a lone number or values above 255 are not byte codes
    assert parse_payload("42") == 42
    assert parse_payload("300 400") == {"raw": "300 400"}


def test_garbage_is_returned_raw():
    assert parse_payload("not json") == {"raw": "not json"}
