#!/usr/bin/env python3
# =============================================================================
# EGNSite MQTT bridge
#
# Long-running service (egnsite-mqtt.service) that connects the MQTT bus to the
# web front end. Two subscriptions:
#
#   raspi/to_django   sensor telemetry  -> Message table + live chart WebSocket
#   pendulum/status   rig state         -> live status badge WebSocket
#
# Run standalone, NOT via manage.py:
#     /opt/ed2/venv/bin/python mqtt_subscriber.py
# =============================================================================
import os
import sys

# Set up Django environment
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)
os.environ.setdefault('DJANGO_SETTINGS_MODULE', 'EGNSite.settings')

# start django
import django
django.setup()

# MQTT and Message import
import paho.mqtt.client as mqtt
from MatlabApp.models import Message

# Channel layer for WebSocket push
import json
import re
import logging
from asgiref.sync import async_to_sync
from channels.layers import get_channel_layer

channel_layer = get_channel_layer()

# --- Configuration -----------------------------------------------------------
# This service always runs on the same host as the broker, so loopback is the
# right default; deploy/.env can override.
BROKER = os.environ.get('MQTT_BROKER_HOST', '127.0.0.1')
PORT = int(os.environ.get('MQTT_BROKER_PORT', 1885))
TOPIC_TELEMETRY = os.environ.get('MQTT_TELEMETRY_TOPIC', 'raspi/to_django')
TOPIC_STATUS = os.environ.get('MQTT_STATUS_TOPIC', 'pendulum/status')

GROUP = 'sensor_data'
MAX_MESSAGES = 5000

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [MQTT BRIDGE] %(levelname)s: %(message)s',
    handlers=[logging.StreamHandler(sys.stdout)],
)
log = logging.getLogger(__name__)


# --- Connection callbacks ----------------------------------------------------
# NOTE: these use the paho-mqtt 2.x (VERSION2) callback signatures. They take
# `reason_code` and `properties`, unlike the 1.x signatures that took a bare
# `rc`. Reverting to the old shape will break silently -- paho calls the
# callback with the wrong number of arguments.
def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code == 0:
        log.info(f"Connected to MQTT broker at {BROKER}:{PORT}")
        # Subscribing inside on_connect (rather than after connect()) means
        # subscriptions are automatically restored after a reconnect.
        client.subscribe([(TOPIC_TELEMETRY, 0), (TOPIC_STATUS, 1)])
        log.info(f"Subscribed to '{TOPIC_TELEMETRY}' and '{TOPIC_STATUS}'")
        log.info("Waiting for messages from the Pi...")
    else:
        log.error(f"Connection failed: {reason_code}")


def on_disconnect(client, userdata, flags, reason_code, properties):
    log.warning(f"Disconnected from broker (reason={reason_code}) — reconnecting")


def parse_payload(message_text):
    # First attempt — standard JSON
    try:
        return json.loads(message_text)
    except json.JSONDecodeError:
        pass

    # Second attempt — fix unquoted keys by replacing word: with "word":
    try:
        fixed = re.sub(r'(\{|,)\s*(\w+)\s*:', r'\1"\2":', message_text)
        return json.loads(fixed)
    except json.JSONDecodeError:
        pass

    return {'raw': message_text}


def parse_status(payload):
    """Turn a pendulum/status payload into {status, detail} for the UI.

    dac_daemon.py emits: online, offline, homing, stopped,
    running:<ModelName>, error:<kind>:<detail>
    """
    text = payload.strip()

    if text.startswith('running:'):
        return {'status': 'running', 'detail': text.split(':', 1)[1]}
    if text.startswith('error:'):
        return {'status': 'error', 'detail': text.split(':', 1)[1]}
    if text in ('online', 'offline', 'homing', 'stopped'):
        return {'status': text, 'detail': ''}

    return {'status': 'unknown', 'detail': text}


# --- Message handling --------------------------------------------------------
def handle_telemetry(message_text):
    """Persist a sensor reading and push it to every open dashboard."""
    from django.db import connection

    # Force a fresh DB connection. The rig sits idle between lab sessions,
    # long enough for MySQL's wait_timeout to drop a pooled connection; without
    # this the first reading after a quiet period raises OperationalError.
    connection.close()

    try:
        Message.objects.create(message=message_text)

        # Keep only the most recent MAX_MESSAGES readings
        count = Message.objects.count()
        if count > MAX_MESSAGES:
            oldest_ids = Message.objects.order_by('timestamp').values_list(
                'id', flat=True)[:count - MAX_MESSAGES]
            Message.objects.filter(id__in=list(oldest_ids)).delete()

        log.info(f"Saved to Message table (total: {min(count, MAX_MESSAGES)})")
    except Exception as e:
        log.error(f"Error saving to database: {e}")

    try:
        payload = parse_payload(message_text)
        async_to_sync(channel_layer.group_send)(
            GROUP,
            {
                'type': 'sensor_update',
                'data': payload,
            },
        )
        log.info(f"Pushed sensor data to WebSocket group '{GROUP}'")
    except Exception as e:
        log.error(f"Error pushing sensor data to WebSocket: {e}")


def handle_status(message_text):
    """Push rig state to every open dashboard.

    Deliberately NOT written to the Message table -- that table backs the
    sensor charts and the data-history page, and status strings would pollute
    both. Status is transient and already retained by the broker.
    """
    parsed = parse_status(message_text)

    try:
        async_to_sync(channel_layer.group_send)(
            GROUP,
            {
                'type': 'rig_status',
                'status': parsed['status'],
                'detail': parsed['detail'],
            },
        )
        log.info(f"Rig status → {parsed['status']} {parsed['detail']}".rstrip())
    except Exception as e:
        log.error(f"Error pushing status to WebSocket: {e}")


def on_message(client, userdata, msg):
    message_text = msg.payload.decode(errors='replace')
    log.info(f"Received on '{msg.topic}': {message_text}")

    if msg.topic == TOPIC_STATUS:
        handle_status(message_text)
    else:
        handle_telemetry(message_text)


def main():
    # Explicitly request the paho 2.x callback API. Omitting it silently falls
    # back to CallbackAPIVersion.VERSION1, which still works on 2.x but is
    # deprecated and slated for removal -- at which point the old-style
    # callbacks would break. dac_daemon.py on the Pi already uses VERSION2.
    client = mqtt.Client(
        mqtt.CallbackAPIVersion.VERSION2,
        client_id='egnsite_bridge',
    )

    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_message = on_message

    # Back off between reconnect attempts instead of hammering the broker
    # while it is down.
    client.reconnect_delay_set(min_delay=1, max_delay=60)

    log.info("Starting MQTT bridge...")

    try:
        # connect_async + loop_forever keeps retrying if the broker is not up
        # yet at boot, rather than exiting.
        client.connect_async(BROKER, PORT, keepalive=60)
        client.loop_forever(retry_first_connection=True)
    except KeyboardInterrupt:
        log.info("Bridge stopped by user")
        client.disconnect()
    except Exception as e:
        log.error(f"Fatal error: {e}")
        raise


if __name__ == "__main__":
    main()
