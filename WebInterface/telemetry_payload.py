"""Parse telemetry payloads from the Pi (MQTT topic raspi/to_django).

Kept free of Django so it can be tested on its own (tests_telemetry/).
"""
import json
import re

# The R2026b Raspberry Pi Blockset MQTT Publish block sends a text message as the
# decimal code of every byte followed by a space: '{"a": 1}' -> "123 34 97 34 ...".
_BYTE_CODES = re.compile(r'^\s*\d{1,3}(?:\s+\d{1,3})+\s*$')


def _decode_byte_codes(message_text):
    """'123 34 ...' -> '{"...', or None if the text is not a run of byte codes."""
    if not _BYTE_CODES.match(message_text):
        return None
    codes = [int(c) for c in message_text.split()]
    if any(c > 255 for c in codes):
        return None
    return bytes(codes).decode(errors='replace')


def parse_payload(message_text):
    decoded = _decode_byte_codes(message_text)
    if decoded is not None:
        message_text = decoded

    # First attempt: standard JSON
    try:
        return json.loads(message_text)
    except json.JSONDecodeError:
        pass

    # Second attempt: fix unquoted keys by replacing word: with "word":
    try:
        fixed = re.sub(r'(\{|,)\s*(\w+)\s*:', r'\1"\2":', message_text)
        return json.loads(fixed)
    except json.JSONDecodeError:
        pass

    return {'raw': message_text}
